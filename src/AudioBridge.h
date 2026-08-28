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
#include "driver/dac.h"

// ============================================================================
// Pont audio : canal RFCOMM audio (SBC, Bluetooth) <-> poste réel (ex. Quansheng
// UV-K1) via le DAC / l'ADC internes de l'ESP32. Brochage aligné sur kv4p-ht
// (voir config.h section 5).
//
//   HTCommander -> poste (EMISSION) :
//     trames AudioData -> pushRadioSbc() -> décodeur SBC -> file PCM -> DAC
//     (GPIO25) -> entrée micro du poste. En parallèle : sortie PTT (GPIO18)
//     activée tant que des trames arrivent, relâchée après AUDIO_PTT_TAIL_MS.
//
//   poste -> HTCommander (RECEPTION) :
//     quand le squelch du poste s'ouvre (entrée SQ GPIO32, ou VOX si SQ=-1) :
//     ADC (GPIO34) <- sortie HP du poste -> encodeur SBC -> onTxFrame()
//     (trames AudioData) ; onTxEnd() (AudioEnd) quand le squelch se referme.
//     onRxLevel(actif, rssi) alimente is_sq / is_in_rx / RSSI du statut.
//
// Le même I2S0 fait ADC + DAC en full-duplex (seul I2S0 est câblé aux ADC/DAC
// internes). Half-duplex logique : pendant l'émission vers le poste, la capture
// ADC -> HTCommander est inhibée.
//
// Validé sur matériel (carte kv4p-ht v1 + SA818). ADC et DAC internes partagent
// l'I2S0 -> bascule half-duplex a chaque PTT (voir applyIoMode). L'etage
// analogique (atténuateur DAC->micro, polarisation HP->ADC) est cote materiel.
// ============================================================================

class AudioBridge {
public:
    using TxFrameFn = std::function<void(const uint8_t*, size_t)>;   // SBC -> HTCommander
    using TxEndFn   = std::function<void()>;                         // AudioEnd -> HTCommander
    using RxLevelFn = std::function<void(bool, uint8_t)>;            // (RX poste actif ?, RSSI 0..15)
    using TxStateFn = std::function<void(bool)>;                     // émission vers le poste ?

    void onTxFrame(TxFrameFn fn) { txFrameCb_ = std::move(fn); }
    void onTxEnd(TxEndFn fn)     { txEndCb_   = std::move(fn); }
    void onRxLevel(RxLevelFn fn) { rxLevelCb_ = std::move(fn); }
    void onTxState(TxStateFn fn) { txStateCb_ = std::move(fn); }
    void setChannelUp(bool up)   { channelUp_.store(up); }   // canal RFCOMM audio connecté ?

    // --- Mode données (canal APRS) : l'audio ne passe plus par le SBC mais par
    //     le TNC. L'ADC est routé vers dataRxCb_, le DAC est alimenté par
    //     dataTxAudio() (sortie du modulateur AFSK).
    using DataRxFn = std::function<void(const int16_t*, size_t)>;
    void onDataRxAudio(DataRxFn fn) { dataRxCb_ = std::move(fn); }
    void setDataMode(bool on)       { dataMode_.store(on); }
    bool dataMode() const           { return dataMode_.load(); }

    // Poussé par le modulateur AFSK (bloque si la file DAC est pleine -> cadence
    // la génération sur le temps réel).
    void dataTxAudio(const int16_t* pcm, size_t n) {
#if AUDIO_BRIDGE_ENABLE
        if (!dataTxSb_) return;
        dataTxLastMs_.store(millis());
        xStreamBufferSend(dataTxSb_, pcm, n * sizeof(int16_t), pdMS_TO_TICKS(500));
#endif
    }
    void dataTxEnd() { /* le PTT retombe via la traîne dans ctlLoop */ }
    bool dataTxActive() const { return (millis() - dataTxLastMs_.load()) < 300; }

    bool begin() {
#if !AUDIO_BRIDGE_ENABLE
        Serial.println("[AUDIO] Pont audio desactive (AUDIO_BRIDGE_ENABLE=false)");
        return true;
#else
        sbcIn_ = xStreamBufferCreate(kSbcInBytes, 1);
        if (!sbcIn_) { Serial.println("[AUDIO] ERREUR: stream buffer SBC"); return false; }
        dataTxSb_ = xStreamBufferCreate(4096, 1);   // ~64 ms de PCM du modulateur AFSK
        if (!dataTxSb_) { Serial.println("[AUDIO] ERREUR: stream buffer TNC"); return false; }

        if (!decoder_.begin()) { Serial.println("[AUDIO] ERREUR: init decodeur SBC"); return false; }
        if (!encoder_.begin(AUDIO_SBC_BITPOOL)) { Serial.println("[AUDIO] ERREUR: init encodeur SBC"); return false; }
        Serial.printf("[AUDIO] SBC pret (trame ~%d octets, %u ech. PCM / trame)\n",
                      encoder_.frameLen(), (unsigned)sbc::kSamplesPerFrame);

        setupGpio();
        {
            size_t pr = (size_t)AUDIO_RX_PREROLL_MS * AUDIO_SAMPLE_RATE_HZ / 1000;
            preroll_.cap = pr < kPreroll ? pr : kPreroll - 1;
        }
        // NB : l'I2S est installe DEPUIS la tache pumpLoop, pas ici. Le pilote
        // ADC interne prend un mutex RECURSIF (adc1_i2s_lock) qui doit etre
        // pris ET rendu par la MEME tache -> sinon assert au 1er passage TX.

        xTaskCreatePinnedToCore(&AudioBridge::pumpTrampoline, "audio_pump", 4096, this, 6, &pumpTask_, 1);
        xTaskCreatePinnedToCore(&AudioBridge::rxTrampoline,   "audio_rx",   4096, this, 5, &rxTask_,   1);
        xTaskCreatePinnedToCore(&AudioBridge::txTrampoline,   "audio_tx",   4096, this, 5, &txTask_,   1);
        xTaskCreatePinnedToCore(&AudioBridge::ctlTrampoline,  "audio_ctl",  3072, this, 4, &ctlTask_,  1);
        Serial.println("[AUDIO] Pont audio demarre");
        return true;
#endif
    }

    // Trame AudioData reçue de HTCommander (type 0x00 / 0x03) : audio à émettre
    // sur le poste. Appelé depuis le callback Bluetooth.
    void pushRadioSbc(const uint8_t* sbc, size_t len) {
#if AUDIO_BRIDGE_ENABLE
        if (!sbcIn_ || !len) return;
        lastRadioSbcMs_.store(millis());
        xStreamBufferSend(sbcIn_, sbc, len, 0);   // dépôt non bloquant
#endif
    }

    // Trame AudioEnd reçue de HTCommander (type 0x01) : fin d'émission.
    void radioAudioEnd() {
#if AUDIO_BRIDGE_ENABLE
        // Relâche le PTT après la traîne, sans prolonger davantage.
        uint32_t t = millis();
        if (t > AUDIO_PTT_TAIL_MS) t -= (AUDIO_PTT_TAIL_MS - 40);
        lastRadioSbcMs_.store(t);
#endif
    }

    bool txToRadio() const    { return hcTxActive_.load(); }
    bool rxFromRadio() const   { return micGateOpen_.load(); }
    bool squelchRaw() const    { return sqDbg_.load(); }
    uint32_t adcLevel() const  { return adcLevel_.load(); }

private:
    // ---------------- File PCM (SPSC lock-free, capacité = puissance de 2) --
    template <size_t N>
    struct PcmRing {
        int16_t buf[N];
        std::atomic<uint32_t> head{0};
        std::atomic<uint32_t> tail{0};
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

    // Fenetre glissante des N derniers echantillons (pre-roll RX).
    template <size_t N>
    struct Preroll {
        int16_t buf[N];
        size_t  wr = 0, fill = 0;
        size_t  cap = N;              // nb d'echantillons conserves (<= N)
        void push(const int16_t* s, size_t n) {
            for (size_t i = 0; i < n; i++) { buf[wr] = s[i]; wr = (wr + 1) & (N - 1); }
            fill = (fill + n < cap) ? fill + n : cap;
        }
        template <class Ring> void drainTo(Ring& r) {
            size_t start = (wr - fill) & (N - 1);
            for (size_t i = 0; i < fill; i++) {
                int16_t s = buf[(start + i) & (N - 1)];
                r.write(&s, 1);
            }
            fill = 0; wr = 0;
        }
        void clear() { fill = 0; wr = 0; }
    };

    static constexpr size_t kFrame      = sbc::kSamplesPerFrame;   // 128
    static constexpr size_t kSbcInBytes = 4096;
    static constexpr size_t kPlayRing   = 8192;   // ~256 ms @ 32 kHz
    static constexpr size_t kMicRing    = 8192;   // ~256 ms : pre-roll + audio vif
    static constexpr size_t kPreroll    = 8192;   // <= 256 ms @ 32 kHz (puissance de 2)

    // ---------------- GPIO (PTT / SQ / bouton / LED) ---------------------
    void setupGpio() {
#if (AUDIO_PTT_GPIO >= 0)
        pinMode(AUDIO_PTT_GPIO, OUTPUT);
        pttWrite(false);
        Serial.printf("[AUDIO] PTT sortie GPIO%d (actif %s)\n",
                      AUDIO_PTT_GPIO, AUDIO_PTT_ACTIVE_LOW ? "BAS" : "HAUT");
#endif
#if (AUDIO_SQ_GPIO >= 0)
        pinMode(AUDIO_SQ_GPIO, AUDIO_SQ_PULLUP ? INPUT_PULLUP : INPUT);
        Serial.printf("[AUDIO] Squelch entree GPIO%d (ouvert = %s)\n",
                      AUDIO_SQ_GPIO, AUDIO_SQ_ACTIVE_LOW ? "BAS" : "HAUT");
#else
        Serial.printf("[AUDIO] Pas de fil squelch -> VOX (seuil %d)\n", AUDIO_SQ_VOX_THRESH);
#endif
#if (AUDIO_PHYS_PTT_GPIO >= 0)
        pinMode(AUDIO_PHYS_PTT_GPIO, INPUT_PULLUP);
#endif
#if (AUDIO_STATUS_LED_GPIO >= 0)
        pinMode(AUDIO_STATUS_LED_GPIO, OUTPUT);
        digitalWrite(AUDIO_STATUS_LED_GPIO, LOW);
#endif
    }

    static void pttWrite(bool tx) {
#if (AUDIO_PTT_GPIO >= 0)
        bool low = tx ? AUDIO_PTT_ACTIVE_LOW : !AUDIO_PTT_ACTIVE_LOW;
        digitalWrite(AUDIO_PTT_GPIO, low ? LOW : HIGH);
#else
        (void)tx;
#endif
    }

    bool squelchHwOpen() {
#if (AUDIO_SQ_GPIO >= 0)
        int v = digitalRead(AUDIO_SQ_GPIO);
        return AUDIO_SQ_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
#else
        return vox_;   // mis à jour depuis adcLevel_ dans la tâche de contrôle
#endif
    }

    static bool physPttPressed() {
#if (AUDIO_PHYS_PTT_GPIO >= 0)
        return digitalRead(AUDIO_PHYS_PTT_GPIO) == LOW;
#else
        return false;
#endif
    }

    // ---------------- I2S half-duplex (ADC <-> DAC sur I2S0) ------------
    // Sur ESP32, ADC et DAC internes partagent I2S0 : on ne peut activer que
    // l'un OU l'autre. On (ré)installe le pilote I2S0 dans le bon sens à chaque
    // bascule PTT (comme kv4p-ht). Ces appels ne se font QUE dans pumpLoop().
    enum IoMode : uint8_t { IO_NONE, IO_RX, IO_TX };

    bool installRx() {
#if AUDIO_ADC_BIAS_ENABLE
        dac_output_enable(DAC_CHANNEL_2);                 // GPIO26 : polarisation entrée ADC
        dac_output_voltage(DAC_CHANNEL_2, AUDIO_ADC_BIAS_CODE);
#endif
        // Config alignée sur arduino-audio-tools (AnalogDriverESP32), la lib
        // audio de kv4p-ht : mono RX = ONLY_LEFT, communication_format = 0.
        i2s_config_t cfg = {};
        cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
        cfg.sample_rate          = AUDIO_SAMPLE_RATE_HZ;
        cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
        cfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
        cfg.communication_format = (i2s_comm_format_t)0;
        cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
        cfg.dma_buf_count        = 6;
        cfg.dma_buf_len          = kFrame;
        cfg.use_apll             = AUDIO_I2S_APLL;
        esp_err_t e = i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
        if (e != ESP_OK) { Serial.printf("[AUDIO] install RX -> %s\n", esp_err_to_name(e)); return false; }
        adc1_config_channel_atten((adc1_channel_t)AUDIO_ADC_CHANNEL, ADC_ATTEN_DB_12);
        i2s_set_adc_mode(ADC_UNIT_1, (adc1_channel_t)AUDIO_ADC_CHANNEL);
        i2s_adc_enable(I2S_NUM_0);
        i2s_zero_dma_buffer(I2S_NUM_0);
        return true;
    }

    bool installTx() {
#if AUDIO_ADC_BIAS_ENABLE
        dac_output_disable(DAC_CHANNEL_2);
#endif
        i2s_config_t cfg = {};
        cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN);
        cfg.sample_rate          = AUDIO_SAMPLE_RATE_HZ;
        cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
        cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
        cfg.communication_format = (i2s_comm_format_t)0;
        cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
        cfg.dma_buf_count        = 6;
        cfg.dma_buf_len          = kFrame;
        cfg.use_apll             = AUDIO_I2S_APLL;
        cfg.tx_desc_auto_clear   = true;
        esp_err_t e = i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
        if (e != ESP_OK) { Serial.printf("[AUDIO] install TX -> %s\n", esp_err_to_name(e)); return false; }
        i2s_set_pin(I2S_NUM_0, nullptr);                  // NULL -> DAC interne
        // GPIO25 seul : sur la carte kv4p-ht, GPIO26 fait partie du reseau de
        // polarisation ADC -> ne pas y sortir d'audio pendant l'emission.
        i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);       // GPIO25 = AUDIO_OUT
        i2s_set_sample_rates(I2S_NUM_0, AUDIO_SAMPLE_RATE_HZ);
        i2s_zero_dma_buffer(I2S_NUM_0);
        return true;
    }

    // TOUJOURS appelee depuis pumpLoop() (contrainte du mutex ADC recursif).
    void applyIoMode(IoMode want) {
        if (ioMode_ == want) return;

        if (ioMode_ == IO_RX) i2s_adc_disable(I2S_NUM_0);   // rend adc1_i2s_lock (meme tache)
        if (ioMode_ != IO_NONE) {
            i2s_driver_uninstall(I2S_NUM_0);
            vTaskDelay(pdMS_TO_TICKS(5));                    // laisse l'ISR/DMA se poser
        }
        bool ok = (want == IO_TX) ? installTx() : installRx();  // i2s_driver_install demarre l'I2S
        ioMode_ = ok ? want : IO_NONE;
#if AUDIO_DEBUG
        Serial.printf("[AUDIO] I2S0 -> %s%s\n",
                      want == IO_TX ? "SORTIE DAC (emission)" : "ENTREE ADC (reception)",
                      ok ? "" : "  !! ECHEC");
#endif
    }

    // ---------------- Tâche "pompe" I2S (timing temps réel) -------------
    static void pumpTrampoline(void* self) { static_cast<AudioBridge*>(self)->pumpLoop(); }
    void pumpLoop() {
        uint16_t adc[kFrame];
        int16_t  mic[kFrame];
        int16_t  play[kFrame];
        uint16_t dac[kFrame * 2];
        size_t   n = 0;

        for (;;) {
            applyIoMode(wantTx_.load() ? IO_TX : IO_RX);

            if (ioMode_ == IO_RX) {
                if (i2s_read(I2S_NUM_0, adc, sizeof(adc), &n, pdMS_TO_TICKS(30)) == ESP_OK && n) {
                    size_t cnt = n / sizeof(uint16_t);
                    uint32_t absSum = 0;
                    bool gate = micGateOpen_.load();
                    for (size_t i = 0; i < cnt; i++) {
                        int raw = adc[i] & 0x0FFF;
#if AUDIO_MIC_DC_TRACK
                        micDc_ += (raw - micDc_) * 0.0015f;
#else
                        micDc_ = 2048.0f;
#endif
                        // Gain TOTAL direct : 16 = "unité" (pleine échelle ADC
                        // 12 bits -> pleine échelle int16), comme le Boost(16.0)
                        // de kv4p-ht. > 16 = amplification (risque d'écrêtage).
                        float s = (raw - micDc_) * (float)AUDIO_MIC_GAIN;
                        int16_t v;
                        if (s >= 32767.f)       { v = 32767;  micClip_.fetch_add(1); }
                        else if (s <= -32768.f) { v = -32768; micClip_.fetch_add(1); }
                        else                      v = (int16_t)s;
                        mic[i] = v;
                        absSum += (uint32_t)(v < 0 ? -v : v);
                    }
                    if (cnt) {
                        uint32_t mean = absSum / cnt;
                        adcEnv_ += ((float)mean - adcEnv_) * 0.25f;
                        adcLevel_.store((uint32_t)adcEnv_);
                        adcSamples_.fetch_add(cnt);
                    }
                    if (dataMode_.load()) {
                        // Canal APRS : l'ADC va au démodulateur TNC, pas au SBC.
                        if (dataRxCb_) dataRxCb_(mic, cnt);
                    } else {
                        // Pre-roll : sur le front d'ouverture, on injecte d'abord
                        // les ~AUDIO_RX_PREROLL_MS captées AVANT (le squelch du
                        // SA818 s'ouvre après le préambule d'un paquet APRS).
                        if (gate && !gatePrev_) preroll_.drainTo(micRing_);
                        if (gate) micRing_.write(mic, cnt);
                        else      preroll_.push(mic, cnt);
                    }
                    gatePrev_ = gate;
                }
            } else if (ioMode_ == IO_TX) {
                size_t got;
                if (dataMode_.load()) {
                    size_t br = xStreamBufferReceive(dataTxSb_, play,
                                                     kFrame * sizeof(int16_t), 0);
                    got = br / sizeof(int16_t);
                } else {
                    got = playRing_.read(play, kFrame);
                }
                for (size_t i = 0; i < kFrame; i++) {
                    float in = (i < got) ? (play[i] * AUDIO_SPK_VOLUME) : 0.f;
#if AUDIO_DAC_LOWPASS
                    dacLp_ += (in - dacLp_) * (float)AUDIO_DAC_LP_ALPHA;   // adoucit l'escalier 8 bits
                    in = dacLp_;
#endif
                    int32_t p = (int32_t)in;
                    if (p > 32767) p = 32767; else if (p < -32768) p = -32768;
                    uint16_t w = (uint16_t)(p + 0x8000);           // 16b non signé, DAC = 8 MSB
                    dac[2 * i] = w;
                    dac[2 * i + 1] = w;                            // GPIO25
                }
                i2s_write(I2S_NUM_0, dac, sizeof(dac), &n, pdMS_TO_TICKS(50));
            } else {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }

    // ---------------- Tâche décodage SBC -> file de lecture (-> DAC) ----
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

            size_t offset = 0;
            for (;;) {
                size_t inLeft = accLen - offset;
                if (inLeft == 0) break;
                size_t samples = decoder_.decode(acc + offset, &inLeft, pcm,
                                                 sizeof(pcm) / sizeof(pcm[0]));
                size_t consumed = (accLen - offset) - inLeft;
                if (samples) playRing_.write(pcm, samples);
                if (consumed == 0) break;
                offset += consumed;
            }
            accLen -= offset;
            if (accLen && offset) memmove(acc, acc + offset, accLen);
            if (accLen == kAcc) accLen = 0;
        }
    }

    // ---------------- Tâche encodage micro -> onTxFrame (-> HTCommander) --
    static void txTrampoline(void* self) { static_cast<AudioBridge*>(self)->txLoop(); }
    void txLoop() {
        int16_t frame[kFrame];
        uint8_t sbcOut[sbc::kMaxSbcFrameLen];

        for (;;) {
            if (!micGateOpen_.load()) { vTaskDelay(pdMS_TO_TICKS(15)); continue; }
            if (micRing_.available() < kFrame) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
            micRing_.read(frame, kFrame);
            size_t len = encoder_.encodeFrame(frame, sbcOut, sizeof(sbcOut));
            if (len) {
                encFrames_.fetch_add(1);
                if (txFrameCb_) txFrameCb_(sbcOut, len);
            }
        }
    }

public:
    uint32_t encFramesTake() { return encFrames_.exchange(0); }  // trace : trames SBC produites
    uint32_t adcSamplesTake() { return adcSamples_.exchange(0); } // trace : ech. ADC lus (=> Hz reel)
    uint32_t micClipTake()   { return micClip_.exchange(0); }     // trace : ech. ADC ecretes
private:

    // ---------------- Tâche de contrôle : PTT, squelch, statut ----------
    static void ctlTrampoline(void* self) { static_cast<AudioBridge*>(self)->ctlLoop(); }
    void ctlLoop() {
        bool     sqStable      = false;
        bool     sqRawPrev     = false;
        uint32_t sqRawRoseMs   = 0;   // date de la derniere montee de sqRaw
        uint32_t sqRawOpenMs   = 0;   // derniere fois que sqRaw etait vrai
        bool     txPrev        = false;
        bool     rxGatePrev    = false;
        bool     sigPrev       = false;
        uint32_t lastRssiMs    = 0;
        uint8_t  rssiPrev      = 0;

        for (;;) {
            uint32_t now = millis();

            // --- PTT vers le poste : actif tant que HTCommander envoie de la
            //     phonie, OU tant que le modulateur TNC produit de l'audio ---
            bool tx = ((now - lastRadioSbcMs_.load()) < (uint32_t)AUDIO_PTT_TAIL_MS)
                      || dataTxActive();
            wantTx_.store(tx);                         // pumpLoop bascule I2S0 ADC<->DAC
            if (tx != txPrev) {
                txPrev = tx;
                hcTxActive_.store(tx);
                pttWrite(tx);
                if (txStateCb_) txStateCb_(tx);
#if AUDIO_DEBUG
                Serial.printf("[AUDIO] PTT %s  (GPIO%d = %s)\n",
                              tx ? "ON (HTCommander emet)" : "OFF",
                              (int)AUDIO_PTT_GPIO,
                              (AUDIO_PTT_GPIO < 0) ? "n/a"
                                  : ((tx == AUDIO_PTT_ACTIVE_LOW) ? "LOW" : "HIGH"));
#endif
            }

            // --- VOX (si pas de fil squelch) : hystérésis + traîne 400 ms ---
#if (AUDIO_SQ_GPIO < 0)
            uint32_t lvl = adcLevel_.load();
            if (lvl > (uint32_t)(AUDIO_SQ_VOX_THRESH / 2)) voxLoudMs_ = now;
            if (!vox_ && lvl > (uint32_t)AUDIO_SQ_VOX_THRESH) vox_ = true;
            else if (vox_ && now - voxLoudMs_ > 400)          vox_ = false;
#endif
            // --- Squelch : attaque rapide, RELACHEMENT retardé (traîne) ---
            // Le pin SQ du SA818 clignote pendant une réception AFSK/APRS :
            // sans traîne on hache l'audio en dizaines de fragments.
            bool sqRaw = squelchHwOpen() || physPttPressed();
            sqDbg_.store(sqRaw);
            if (sqRaw && !sqRawPrev) sqRawRoseMs = now;
            if (sqRaw) sqRawOpenMs = now;
            sqRawPrev = sqRaw;

            if (!sqStable && sqRaw && (now - sqRawRoseMs) >= (uint32_t)AUDIO_SQ_ATTACK_MS)
                sqStable = true;
            else if (sqStable && !sqRaw && (now - sqRawOpenMs) >= (uint32_t)AUDIO_SQ_HANG_MS)
                sqStable = false;

            // --- Capture ADC -> HTCommander (jamais pendant l'émission) ---
            // Squelch "ouvert en permanence" : soit AUDIO_RX_ALWAYS, soit le
            // squelch du module réglé à 0 (RF_MODULE_SQUELCH). Comme kv4p-ht :
            // le pilote pousse TOUJOURS l'audio à l'appli, qui décide. Le pin SQ
            // matériel n'est alors qu'une indication (RSSI / statut).
#if AUDIO_RX_ALWAYS || (RF_MODULE_SQUELCH == 0)
            bool rxGate = channelUp_.load() && !tx;   // capture audio en continu
#else
            bool rxGate = sqStable && !tx;
#endif
            bool rxOpened = rxGate && !rxGatePrev;
            bool rxClosed = !rxGate && rxGatePrev;
            rxGatePrev = rxGate;
            if (rxOpened || rxClosed) micGateOpen_.store(rxGate);
            if (rxClosed && txEndCb_) txEndCb_();               // AudioEnd -> HTCommander
#if AUDIO_DEBUG
            if (rxOpened) Serial.printf("[AUDIO] RX : capture ADC ouverte (niveau %lu)\n",
                                        (unsigned long)adcLevel_.load());
            if (rxClosed) Serial.println("[AUDIO] RX : capture ADC fermée -> AudioEnd");
#endif

            // --- Signal présent (pour is_sq / is_in_rx / RSSI) : suit TOUJOURS
            //     le pin SQ débruité, même en capture continue -> l'indicateur
            //     RX et le S-mètre de HTCommander restent corrects.
            bool sig = sqStable && !tx;
            bool sigOn  = sig && !sigPrev;
            bool sigOff = !sig && sigPrev;
            sigPrev = sig;
            if (sig) {
                float l = log2f((float)adcLevel_.load() + 1.0f);
                uint8_t rssi = l >= 15.f ? 15 : (uint8_t)(l + 0.5f);
                if (rssi < 1) rssi = 1;
                if (sigOn || (rssi != rssiPrev && now - lastRssiMs > 150)) {
                    rssiPrev = rssi; lastRssiMs = now;
                    if (rxLevelCb_) rxLevelCb_(true, rssi);
                }
            } else if (sigOff) {
                rssiPrev = 0;
                if (rxLevelCb_) rxLevelCb_(false, 0);
            }

            // --- LED d'état : fixe en émission, clignote sur signal reçu ---
#if (AUDIO_STATUS_LED_GPIO >= 0)
            bool led = tx || (sig && ((now / 120) & 1));
            digitalWrite(AUDIO_STATUS_LED_GPIO, led ? HIGH : LOW);
#endif
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // ---------------- Données membres ---------------------------------
    sbc::Decoder decoder_;
    sbc::Encoder encoder_;

    StreamBufferHandle_t sbcIn_ = nullptr;
    StreamBufferHandle_t dataTxSb_ = nullptr;
    PcmRing<kPlayRing> playRing_;
    PcmRing<kMicRing>  micRing_;

    DataRxFn dataRxCb_;
    std::atomic<bool>     dataMode_{false};
    std::atomic<uint32_t> dataTxLastMs_{0};

    TaskHandle_t pumpTask_ = nullptr, rxTask_ = nullptr, txTask_ = nullptr, ctlTask_ = nullptr;

    TxFrameFn txFrameCb_;
    TxEndFn   txEndCb_;
    RxLevelFn rxLevelCb_;
    TxStateFn txStateCb_;

    std::atomic<uint32_t> lastRadioSbcMs_{0};   // dernier AudioData reçu de HTCommander
    std::atomic<uint32_t> adcLevel_{0};         // |PCM| moyen lissé (0..32767)
    std::atomic<bool>     hcTxActive_{false};   // émission vers le poste en cours
    std::atomic<bool>     micGateOpen_{false};  // capture ADC -> HTCommander ouverte
    std::atomic<bool>     sqDbg_{false};        // état brut du squelch (trace)
    std::atomic<bool>     channelUp_{false};    // canal RFCOMM audio connecté
    std::atomic<uint32_t> encFrames_{0};        // trames SBC produites par txLoop
    std::atomic<uint32_t> adcSamples_{0};       // echantillons ADC lus (=> Hz reel)
    std::atomic<uint32_t> micClip_{0};          // echantillons ADC ecretes
    std::atomic<bool>     wantTx_{false};       // sens I2S0 demandé (pumpLoop applique)
    IoMode                ioMode_ = IO_NONE;    // sens I2S0 courant (pumpLoop uniquement)

    float    micDc_ = 2048.0f;
    float    adcEnv_ = 0.0f;
    float    dacLp_ = 0.0f;
    bool     gatePrev_ = false;
    Preroll<kPreroll> preroll_;
    bool     vox_ = false;
    uint32_t voxLoudMs_ = 0;
};
