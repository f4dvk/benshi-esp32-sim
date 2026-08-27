#pragma once
#include <Arduino.h>
#include <atomic>
#include <functional>
#include <string.h>
#include <math.h>

#include "config.h"
#include "SbcCodec.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#include "driver/i2s.h"
#include "driver/adc.h"

// ============================================================================
// Pont audio : canal RFCOMM audio (SBC) <-> DAC / ADC internes de l'ESP32.
//
//   Reception  (HTCommander -> HP)  : pushRadioSbc() -> decodeur SBC -> file
//     PCM de lecture -> I2S "built-in DAC" (GPIO25 / GPIO26).
//   Emission   (micro -> HTCommander): I2S "built-in ADC" (ADC1) -> file PCM
//     micro -> encodeur SBC -> onTxFrame() (trames AudioData) ; onTxEnd() a
//     la fin d'un appui PTT.
//
// Le meme peripherique I2S0 fait ADC + DAC en full-duplex (seul I2S0 est cable
// aux ADC/DAC internes de l'ESP32). Sample rate commun : AUDIO_SAMPLE_RATE_HZ.
// Modele : exemple ESP-IDF "i2s_adc_dac".
//
// ATTENTION : NON VALIDE SUR MATERIEL. Le mode ADC+DAC interne simultane de
// l'ESP32 est connu pour etre bruyant / sensible, et le brochage analogique
// (ampli HP, preampli + biais micro) est a faire cote materiel. Toute la
// chaine SBC + framing + files est en revanche independante du materiel.
// ============================================================================

class AudioBridge {
public:
    using TxFrameFn = std::function<void(const uint8_t*, size_t)>;
    using TxEndFn   = std::function<void()>;
    // (reception audio active ?, RSSI simule 0..15 derive du niveau PCM)
    using RxLevelFn = std::function<void(bool, uint8_t)>;

    void onTxFrame(TxFrameFn fn) { txFrameCb_ = std::move(fn); }
    void onTxEnd(TxEndFn fn)     { txEndCb_   = std::move(fn); }
    void onRxLevel(RxLevelFn fn) { rxLevelCb_ = std::move(fn); }

    bool begin() {
#if !AUDIO_BRIDGE_ENABLE
        Serial.println("[AUDIO] Pont audio desactive (AUDIO_BRIDGE_ENABLE=false)");
        return true;
#else
        sbcIn_ = xStreamBufferCreate(kSbcInBytes, 1);
        if (!sbcIn_) { Serial.println("[AUDIO] ERREUR: stream buffer SBC"); return false; }

        if (!decoder_.begin()) { Serial.println("[AUDIO] ERREUR: init decodeur SBC"); return false; }
        if (!encoder_.begin(AUDIO_SBC_BITPOOL)) { Serial.println("[AUDIO] ERREUR: init encodeur SBC"); return false; }
        Serial.printf("[AUDIO] SBC pret (trame ~%d octets, %u ech. PCM / trame)\n",
                      encoder_.frameLen(), (unsigned)sbc::kSamplesPerFrame);

        if (!startI2s()) return false;

#if (AUDIO_PTT_GPIO >= 0)
        pinMode(AUDIO_PTT_GPIO, INPUT_PULLUP);
        Serial.printf("[AUDIO] PTT sur GPIO%d (actif = masse)\n", AUDIO_PTT_GPIO);
#else
        Serial.println("[AUDIO] Pas de PTT (AUDIO_PTT_GPIO=-1) : micro muet, reception seule");
#endif

        xTaskCreatePinnedToCore(&AudioBridge::pumpTrampoline, "audio_pump", 4096, this, 6, &pumpTask_, 1);
        xTaskCreatePinnedToCore(&AudioBridge::rxTrampoline,   "audio_rx",   4096, this, 5, &rxTask_,   1);
        xTaskCreatePinnedToCore(&AudioBridge::txTrampoline,   "audio_tx",   4096, this, 5, &txTask_,   1);
        Serial.println("[AUDIO] Pont audio demarre");
        return true;
#endif
    }

    // --- Reception : appele depuis le dispatch RFCOMM (trame type 0x00/0x03) ---
    void pushRadioSbc(const uint8_t* sbc, size_t len) {
#if AUDIO_BRIDGE_ENABLE
        if (!sbcIn_ || !len) return;
        rxLastMs_.store(millis());
        // Depot non bloquant : si le buffer est plein on laisse tomber (mieux
        // vaut un trou audio qu'un blocage du callback Bluetooth).
        xStreamBufferSend(sbcIn_, sbc, len, 0);
#endif
    }

    // Fin de flux annoncee par HTCommander (trame type 0x01).
    void radioAudioEnd() {}

    bool micTransmitting() const { return txActive_.load(); }

private:
    // ---------------- File PCM (SPSC lock-free, capacite = puissance de 2) --
    template <size_t N>
    struct PcmRing {
        int16_t buf[N];
        std::atomic<uint32_t> head{0};   // ecrit par le producteur
        std::atomic<uint32_t> tail{0};   // ecrit par le consommateur
        size_t write(const int16_t* s, size_t n) {
            uint32_t h = head.load(std::memory_order_relaxed);
            uint32_t t = tail.load(std::memory_order_acquire);
            size_t freeSpace = N - 1 - ((h - t) & (N - 1));
            size_t w = n < freeSpace ? n : freeSpace;
            for (size_t i = 0; i < w; i++) buf[(h + i) & (N - 1)] = s[i];
            head.store((h + w) & (N - 1), std::memory_order_release);
            return w;
        }
        size_t read(int16_t* s, size_t n) {
            uint32_t t = tail.load(std::memory_order_relaxed);
            uint32_t h = head.load(std::memory_order_acquire);
            size_t avail = (h - t) & (N - 1);
            size_t r = n < avail ? n : avail;
            for (size_t i = 0; i < r; i++) s[i] = buf[(t + i) & (N - 1)];
            tail.store((t + r) & (N - 1), std::memory_order_release);
            return r;
        }
        size_t available() {
            return (head.load(std::memory_order_acquire) -
                    tail.load(std::memory_order_acquire)) & (N - 1);
        }
    };

    static constexpr size_t kFrame      = sbc::kSamplesPerFrame;   // 128
    static constexpr size_t kSbcInBytes = 4096;
    static constexpr size_t kPlayRing   = 8192;   // ~256 ms @ 32 kHz
    static constexpr size_t kMicRing    = 4096;

    // ---------------- I2S ------------------------------------------------
    bool startI2s() {
        int mode = I2S_MODE_MASTER;
#if AUDIO_DAC_ENABLE
        mode |= I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN;
#endif
#if AUDIO_ADC_ENABLE
        mode |= I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN;
#endif
        i2s_config_t cfg = {};
        cfg.mode = (i2s_mode_t)mode;
        cfg.sample_rate          = AUDIO_SAMPLE_RATE_HZ;
        cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
        cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
        cfg.communication_format = I2S_COMM_FORMAT_STAND_MSB;
        cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
        cfg.dma_buf_count        = 8;
        cfg.dma_buf_len          = kFrame;
        cfg.use_apll             = false;
        cfg.tx_desc_auto_clear   = true;

        esp_err_t e = i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
        if (e != ESP_OK) {
            Serial.printf("[AUDIO] i2s_driver_install -> %s\n", esp_err_to_name(e));
            return false;
        }

#if AUDIO_ADC_ENABLE
        i2s_set_adc_mode(ADC_UNIT_1, (adc1_channel_t)AUDIO_ADC_CHANNEL);
        adc1_config_channel_atten((adc1_channel_t)AUDIO_ADC_CHANNEL, ADC_ATTEN_DB_12);
#endif
        // pin = NULL -> initialise les 2 canaux DAC internes (GPIO25 + GPIO26).
        i2s_set_pin(I2S_NUM_0, nullptr);
#if AUDIO_DAC_ENABLE
        i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
#endif
#if AUDIO_ADC_ENABLE
        i2s_adc_enable(I2S_NUM_0);
#endif
        i2s_zero_dma_buffer(I2S_NUM_0);
        return true;
    }

    // ---------------- Tache "pompe" I2S (timing temps reel) --------------
    static void pumpTrampoline(void* self) { static_cast<AudioBridge*>(self)->pumpLoop(); }
    void pumpLoop() {
        int16_t  play[kFrame];
        uint16_t dac[kFrame * 2];        // stereo 16 bits pour le DAC
        uint16_t adc[kFrame];
        int16_t  mic[kFrame];
        size_t   n = 0;

        for (;;) {
#if AUDIO_ADC_ENABLE
            // ---- Lecture ADC (micro) : cadence aussi la boucle ----
            if (i2s_read(I2S_NUM_0, adc, sizeof(adc), &n, portMAX_DELAY) == ESP_OK && n) {
                size_t cnt = n / sizeof(uint16_t);
                for (size_t i = 0; i < cnt; i++) {
                    int raw = adc[i] & 0x0FFF;               // 12 bits utiles
#if AUDIO_MIC_DC_TRACK
                    micDc_ += (raw - micDc_) * 0.0015f;      // suivi lent du biais
#else
                    micDc_ = 2048.0f;
#endif
                    float s = (raw - micDc_) * (AUDIO_MIC_GAIN * 16.0f);
                    mic[i] = s > 32767.f ? 32767 : (s < -32768.f ? -32768 : (int16_t)s);
                }
                micRing_.write(mic, cnt);
            }
#endif
            // ---- Ecriture DAC (haut-parleur) ----
            size_t got = playRing_.read(play, kFrame);
            for (size_t i = 0; i < kFrame; i++) {
                int32_t p = (i < got) ? (int32_t)(play[i] * AUDIO_SPK_VOLUME) : 0;
                uint16_t u8 = (uint16_t)(((p >> 8) + 128) & 0xFF);   // 8 bits DAC
                uint16_t w  = (uint16_t)(u8 << 8);
                dac[2 * i]     = w;   // canal gauche  (GPIO26)
                dac[2 * i + 1] = w;   // canal droit   (GPIO25)
            }
            i2s_write(I2S_NUM_0, dac, sizeof(dac), &n, portMAX_DELAY);

#if !AUDIO_ADC_ENABLE
            // Sans ADC pour cadencer, i2s_write suffit (bloque sur le DMA TX).
#endif
        }
    }

    // ---------------- Tache decodage SBC -> file de lecture -------------
    static void rxTrampoline(void* self) { static_cast<AudioBridge*>(self)->rxLoop(); }
    void rxLoop() {
        static constexpr size_t kAcc = 1024;
        uint8_t acc[kAcc];
        size_t  accLen = 0;
        int16_t pcm[kFrame * 4];

        for (;;) {
            if (accLen < kAcc) {
                accLen += xStreamBufferReceive(sbcIn_, acc + accLen,
                                               kAcc - accLen, pdMS_TO_TICKS(50));
            }
            if (accLen == 0) continue;

            // Decode autant de trames completes que possible.
            size_t offset = 0;
            for (;;) {
                size_t inLeft = accLen - offset;
                if (inLeft == 0) break;
                size_t samples = decoder_.decode(acc + offset, &inLeft, pcm,
                                                 sizeof(pcm) / sizeof(pcm[0]));
                size_t consumed = (accLen - offset) - inLeft;
                if (samples) {
                    playRing_.write(pcm, samples);
                    for (size_t i = 0; i < samples; i++) {
                        int v = pcm[i] < 0 ? -pcm[i] : pcm[i];
                        lvlAccum_ += (uint32_t)v;
                    }
                    lvlCount_ += samples;
                }
                if (consumed == 0) break;          // trame incomplete : on attend la suite
                offset += consumed;
            }
            // Compacte le reliquat en tete de buffer.
            accLen -= offset;
            if (accLen && offset) memmove(acc, acc + offset, accLen);
            if (accLen == kAcc) accLen = 0;        // garde-fou : jamais de blocage

            updateRxLevel();
        }
    }

    // RSSI simule + etat squelch, pousses via rxLevelCb_ (throttle ~150 ms).
    void updateRxLevel() {
        uint32_t now = millis();
        bool active = (now - rxLastMs_.load()) < 300;
        if (active == rxWasActive_ && now - lastLvlMs_ < 150) return;

        uint8_t rssi = 0;
        if (active && lvlCount_) {
            uint32_t mean = (uint32_t)(lvlAccum_ / lvlCount_);   // |PCM| moyen, 0..32767
            float l = log2f((float)mean + 1.0f);                 // ~0..15
            rssi = l >= 15.f ? 15 : (uint8_t)(l + 0.5f);
            if (rssi < 1) rssi = 1;                              // actif => au moins 1
        }
        lvlAccum_ = 0;
        lvlCount_ = 0;
        lastLvlMs_ = now;

        if (active != rxWasActive_ || rssi != lastRssi_) {
            rxWasActive_ = active;
            lastRssi_    = rssi;
            if (rxLevelCb_) rxLevelCb_(active, rssi);
        }
    }

    // ---------------- Tache encodage micro -> onTxFrame ----------------
    static void txTrampoline(void* self) { static_cast<AudioBridge*>(self)->txLoop(); }
    void txLoop() {
        int16_t frame[kFrame];
        uint8_t sbcOut[sbc::kMaxSbcFrameLen];

        for (;;) {
            if (!pttPressed()) {
                if (txActive_.exchange(false) && txEndCb_) txEndCb_();
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            txActive_.store(true);
            if (micRing_.available() < kFrame) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
            micRing_.read(frame, kFrame);
            size_t len = encoder_.encodeFrame(frame, sbcOut, sizeof(sbcOut));
            if (len && txFrameCb_) txFrameCb_(sbcOut, len);
        }
    }

    bool pttPressed() const {
#if (AUDIO_PTT_GPIO >= 0)
        return digitalRead(AUDIO_PTT_GPIO) == LOW;
#else
        return false;
#endif
    }

    // ---------------- Donnees membres ---------------------------------
    sbc::Decoder decoder_;
    sbc::Encoder encoder_;

    StreamBufferHandle_t sbcIn_ = nullptr;
    PcmRing<kPlayRing> playRing_;
    PcmRing<kMicRing>  micRing_;

    TaskHandle_t pumpTask_ = nullptr, rxTask_ = nullptr, txTask_ = nullptr;

    TxFrameFn txFrameCb_;
    TxEndFn   txEndCb_;
    RxLevelFn rxLevelCb_;

    std::atomic<uint32_t> rxLastMs_{0};
    std::atomic<bool>     txActive_{false};
    float                 micDc_ = 2048.0f;

    // Etat "S-metre" simule (tache audio_rx uniquement).
    uint64_t lvlAccum_   = 0;
    uint32_t lvlCount_   = 0;
    uint32_t lastLvlMs_  = 0;
    uint8_t  lastRssi_   = 0;
    bool     rxWasActive_ = false;
};
