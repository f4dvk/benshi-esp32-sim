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
    // Autorisation de capture RX (mode UV-K1 piloté : suit le "signal reçu" du
    // GET_STATUS série ; par défaut true = pas de gating externe).
    void setRxAllow(bool a)      { rxAllow_.store(a); }

    // --- Mode données (canal APRS) : l'audio ne passe plus par le SBC mais par
    //     le TNC. L'ADC est routé vers dataRxCb_, le DAC est alimenté par
    //     dataTxAudio() (sortie du modulateur AFSK).
    using DataRxFn = std::function<void(const int16_t*, size_t)>;
    void onDataRxAudio(DataRxFn fn) { dataRxCb_ = std::move(fn); }
    void setDataMode(bool on)       { dataMode_.store(on); }
    bool dataMode() const           { return dataMode_.load(); }

    // --- Capture pour l'analyseur de spectre de l'écran de façade ---
    // Anneau des derniers échantillons ADC (audio reçu du poste), alimenté par
    // la tâche audio. La tâche d'affichage en prend une copie et fait la FFT.
    static const size_t kSpecN = 256;
    void copySpectrumPcm(int16_t* out) {
        portENTER_CRITICAL(&specMux_);
        size_t h = specHead_;
        for (size_t i = 0; i < kSpecN; i++) out[i] = specBuf_[(h + i) & (kSpecN - 1)];
        portEXIT_CRITICAL(&specMux_);
    }

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
    bool dataTxActive() const {
        return (millis() - dataTxLastMs_.load()) < (uint32_t)AUDIO_DATA_TX_HANG_MS;
    }

    bool begin() {
#if !AUDIO_BRIDGE_ENABLE
        Serial.println("[AUDIO] Pont audio desactive (AUDIO_BRIDGE_ENABLE=false)");
        return true;
#else
        sbcIn_ = xStreamBufferCreate(kSbcInBytes, 1);
        if (!sbcIn_) { Serial.println("[AUDIO] ERREUR: stream buffer SBC"); return false; }
        dataTxSb_ = xStreamBufferCreate(3072, 1);   // ~48 ms de PCM du modulateur AFSK
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
        // L'AGC tourne par échantillon ADC -> au débit I2S réel.
        agcAtk_ = 1.0f - expf(-1.0f / (AUDIO_AGC_ATTACK_MS  * 1e-3f * AUDIO_I2S_RATE));
        agcRel_ = 1.0f - expf(-1.0f / (AUDIO_AGC_RELEASE_MS * 1e-3f * AUDIO_I2S_RATE));
#if AUDIO_I2S_RATE != AUDIO_SAMPLE_RATE_HZ
        rsDown_.init((float)AUDIO_I2S_RATE, (float)AUDIO_SAMPLE_RATE_HZ);   // ADC -> SBC
        rsUp_.init((float)AUDIO_SAMPLE_RATE_HZ, (float)AUDIO_I2S_RATE);     // SBC -> DAC
#endif
        // NB : l'I2S est installe DEPUIS la tache pumpLoop, pas ici. Le pilote
        // ADC interne prend un mutex RECURSIF (adc1_i2s_lock) qui doit etre
        // pris ET rendu par la MEME tache -> sinon assert au 1er passage TX.

        xTaskCreatePinnedToCore(&AudioBridge::pumpTrampoline, "audio_pump", 4608, this, 6, &pumpTask_, 1);
        xTaskCreatePinnedToCore(&AudioBridge::rxTrampoline,   "audio_rx",   3584, this, 5, &rxTask_,   1);
        xTaskCreatePinnedToCore(&AudioBridge::txTrampoline,   "audio_tx",   3072, this, 5, &txTask_,   1);
        xTaskCreatePinnedToCore(&AudioBridge::ctlTrampoline,  "audio_ctl",  2560, this, 4, &ctlTask_,  1);
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
    float    agcGain() const    { return agcGain_; }

private:
    // ---------------- Ré-échantillonneur linéaire (streaming) --------------
    // Interpolation linéaire ; suffisant pour de la voix + du SBC, et le démod
    // AFSK tourne au débit natif (pas ré-échantillonné). ratio = fIn / fOut.
    struct Resampler {
        float   ratio = 1.5f, frac = 0.0f;
        int16_t s0 = 0, s1 = 0;
        void init(float fin, float fout) { ratio = fin / fout; frac = 0; s0 = s1 = 0; }
        // consomme `n` échantillons d'entrée, écrit la sortie (cap), renvoie le
        // nombre d'échantillons produits.
        size_t process(const int16_t* in, size_t n, int16_t* out, size_t cap) {
            size_t o = 0;
            for (size_t i = 0; i < n; i++) {
                s0 = s1; s1 = in[i];
                while (frac < 1.0f) {
                    if (o >= cap) return o;
                    out[o++] = (int16_t)(s0 + (s1 - s0) * frac);
                    frac += ratio;
                }
                frac -= 1.0f;
            }
            return o;
        }
    };

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
    static constexpr size_t kSbcInBytes = 2048;
    static constexpr size_t kPlayRing   = 4096;   // ~128 ms @ 32 kHz (puissance de 2)
    static constexpr size_t kMicRing    = 8192;   // pre-roll + audio vif (puissance de 2)
    static constexpr size_t kPreroll    = 4096;   // ~128 ms @ 32 kHz (puissance de 2)

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
        cfg.sample_rate          = AUDIO_I2S_RATE;
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
        cfg.sample_rate          = AUDIO_I2S_RATE;
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
        i2s_set_sample_rates(I2S_NUM_0, AUDIO_I2S_RATE);
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
        int16_t  mic[kFrame];        // ADC (après gain/AGC), au débit I2S
        int16_t  rawSpec[kFrame];    // ADC AVANT AGC (analyseur de spectre écran)
        int16_t  pcmOut[kFrame * 2]; // PCM à envoyer au DAC (débit I2S)
        uint16_t dac[kFrame * 3];    // stéréo, marge pour un éventuel sur-échantillonnage
#if AUDIO_I2S_RATE != AUDIO_SAMPLE_RATE_HZ
        int16_t  micDs[kFrame];      // ADC ré-échantillonné vers le débit SBC
        int16_t  playIn[kFrame];     // PCM décodeur SBC avant sur-échantillonnage
#endif
        size_t   n = 0;

        for (;;) {
            applyIoMode(wantTx_.load() ? IO_TX : IO_RX);

            if (ioMode_ == IO_RX) {
                if (i2s_read(I2S_NUM_0, adc, sizeof(adc), &n, pdMS_TO_TICKS(30)) == ESP_OK && n) {
                    size_t cnt = n / sizeof(uint16_t);
                    uint32_t absSum = 0;
                    bool gate = micGateOpen_.load();
#if AUDIO_AGC_ENABLE
                    // Nouvelle salve : on repart d'un gain médian (sinon, si une
                    // salve forte précédente a plaqué le gain à 1, la suivante
                    // met AUDIO_AGC_RELEASE_MS à revenir audible).
                    if (gate && !gatePrev_) {
                        agcGain_   = (float)AUDIO_MIC_GAIN;
                        agcEnvPk_  = 0.0f;
                    }
                    gatePrev_ = gate;
#endif
                    for (size_t i = 0; i < cnt; i++) {
                        int raw = adc[i] & 0x0FFF;
#if AUDIO_MIC_DC_TRACK
                        micDc_ += (raw - micDc_) * 0.0015f;
#else
                        micDc_ = 2048.0f;
#endif
                        float pre = (float)raw - micDc_;
                        // Spectre écran : signal AVANT AGC (sinon l'AGC monte à
                        // fond sur le silence et le spectre "colle" en haut).
                        {
                            float pg = pre * 16.0f;
                            rawSpec[i] = pg > 32767.f ? 32767
                                       : (pg < -32768.f ? -32768 : (int16_t)pg);
                        }
#if AUDIO_AGC_ENABLE
                        float s = pre * agcGain_;
                        float rect = s < 0 ? -s : s;
                        // suiveur de crête : montée ~2 ms, descente ~60 ms
                        agcEnvPk_ += (rect - agcEnvPk_) * (rect > agcEnvPk_ ? 0.03f : 0.0009f);
                        // Gain ajusté seulement squelch OUVERT et au-dessus du bruit ;
                        // sinon gelé (pas d'emballement entre deux trames).
                        if (gate && agcEnvPk_ > (float)AUDIO_AGC_NOISE) {
                            float desired = agcGain_ * (float)AUDIO_AGC_TARGET / agcEnvPk_;
                            agcGain_ += (desired - agcGain_) *
                                        (desired < agcGain_ ? agcAtk_ : agcRel_);
                            if (agcGain_ < (float)AUDIO_AGC_MIN_GAIN) agcGain_ = AUDIO_AGC_MIN_GAIN;
                            if (agcGain_ > (float)AUDIO_AGC_MAX_GAIN) agcGain_ = AUDIO_AGC_MAX_GAIN;
                        }
#else
                        float s = pre * (float)AUDIO_MIC_GAIN;
#endif
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
                        pushSpec_(rawSpec, cnt);   // -> analyseur de spectre (écran)
                    }
                    // --- Chaîne SBC (phonie / flux audio vers HTCommander) ---
                    // Active sur tous les canaux ; sur le canal APRS seulement
                    // si TNC_ALSO_STREAM_AUDIO.
#if TNC_ALSO_STREAM_AUDIO
                    bool doSbc = true;
#else
                    bool doSbc = !dataMode_.load();
#endif
                    if (doSbc) {
#if AUDIO_I2S_RATE != AUDIO_SAMPLE_RATE_HZ
                        // ADC (débit I2S) -> débit SBC avant l'encodeur.
                        size_t dn = rsDown_.process(mic, cnt, micDs, kFrame);
                        const int16_t* sbcSrc = micDs;
#else
                        // ADC déjà au débit SBC : chemin direct, latence mini.
                        size_t dn = cnt;
                        const int16_t* sbcSrc = mic;
#endif
                        if (gate && !gatePrev_) preroll_.drainTo(micRing_);
                        if (gate) micRing_.write(sbcSrc, dn);
                        else      preroll_.push(sbcSrc, dn);
                    }
                    // --- Chaîne TNC (démodulateur AFSK), en plus, sur le canal APRS ---
                    // Le démodulateur tourne au débit natif I2S (= AFSK_SAMPLE_RATE).
                    if (dataMode_.load() && dataRxCb_) dataRxCb_(mic, cnt);
                    gatePrev_ = gate;
                }
            } else if (ioMode_ == IO_TX) {
                size_t outN;
                bool dataTx = dataTxActive();
                if (dataTx) {
                    // Modulateur AFSK en cours (TNC TX) : débit natif (= débit
                    // I2S), envoyé directement au DAC. Amplitude déjà réglée par
                    // TNC_TX_GAIN dans le modulateur -> pas de AUDIO_SPK_VOLUME.
                    size_t br = xStreamBufferReceive(dataTxSb_, pcmOut,
                                                     kFrame * sizeof(int16_t), 0);
                    size_t got = br / sizeof(int16_t);
                    for (size_t i = got; i < kFrame; i++) pcmOut[i] = 0;
                    outN = kFrame;
                } else {
#if AUDIO_I2S_RATE != AUDIO_SAMPLE_RATE_HZ
                    // Phonie : décodeur SBC -> sur-échantillonnage vers le débit I2S.
                    size_t inWant = (size_t)((uint32_t)kFrame * AUDIO_SAMPLE_RATE_HZ / AUDIO_I2S_RATE);
                    size_t got = playRing_.read(playIn, inWant);
                    for (size_t i = got; i < inWant; i++) playIn[i] = 0;
                    outN = rsUp_.process(playIn, inWant, pcmOut, (kFrame * 3) / 2);  // <= dac[]
#else
                    // Phonie : décodeur SBC déjà au débit I2S -> DAC direct.
                    size_t got = playRing_.read(pcmOut, kFrame);
                    for (size_t i = got; i < kFrame; i++) pcmOut[i] = 0;
                    outN = kFrame;
#endif
                }
                float vol = dataTx ? 1.0f : (float)AUDIO_SPK_VOLUME;
                for (size_t i = 0; i < outN; i++) {
                    float in = pcmOut[i] * vol;
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
                i2s_write(I2S_NUM_0, dac, outN * 2 * sizeof(uint16_t), &n, pdMS_TO_TICKS(50));
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
            // squelch du module réglé à 0 (RF_MODULE_SQUELCH), soit le mode
            // UV-K1 piloté (le POSTE fait le squelch/CTCSS, l'ESP capte en
            // continu). Comme kv4p-ht : le pilote pousse TOUJOURS l'audio à
            // l'appli, qui décide. Le pin SQ matériel n'est alors qu'une
            // indication (RSSI / statut).
#if AUDIO_RX_ALWAYS || (RF_MODULE_SQUELCH == 0) || RF_MODULE_UVK5_ENABLE
            // Capture "en continu" MAIS soumise à rxAllow_ : en mode UV-K1
            // piloté, le transport la ferme quand le GET_STATUS série dit
            // "pas de signal" (sinon HTCommander voit du RX en permanence).
            bool rxGate = channelUp_.load() && !tx && rxAllow_.load();
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

    // Anneau de capture pour l'analyseur de spectre (écrit par la tâche audio,
    // lu par la tâche d'affichage sous section critique — copie rapide).
    void pushSpec_(const int16_t* s, size_t n) {
        portENTER_CRITICAL(&specMux_);
        for (size_t i = 0; i < n; i++) {
            specBuf_[specHead_] = s[i];
            specHead_ = (specHead_ + 1) & (kSpecN - 1);
        }
        portEXIT_CRITICAL(&specMux_);
    }
    portMUX_TYPE specMux_ = portMUX_INITIALIZER_UNLOCKED;
    int16_t      specBuf_[kSpecN] = {0};
    size_t       specHead_ = 0;

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
    std::atomic<bool>     rxAllow_{true};       // gating RX externe (UV-K1 piloté)
    std::atomic<uint32_t> encFrames_{0};        // trames SBC produites par txLoop
    std::atomic<uint32_t> adcSamples_{0};       // echantillons ADC lus (=> Hz reel)
    std::atomic<uint32_t> micClip_{0};          // echantillons ADC ecretes
    std::atomic<bool>     wantTx_{false};       // sens I2S0 demandé (pumpLoop applique)
    IoMode                ioMode_ = IO_NONE;    // sens I2S0 courant (pumpLoop uniquement)

    float    micDc_ = 2048.0f;
    float    adcEnv_ = 0.0f;
    bool     gatePrev_ = false;   // détection front d'ouverture RX (reset AGC)
    float    dacLp_ = 0.0f;
    float    agcGain_ = (float)AUDIO_MIC_GAIN;
    float    agcEnvPk_ = 0.0f;
    float    agcAtk_ = 0.02f, agcRel_ = 0.0005f;   // coeffs calculés dans begin()
    bool     gatePrev_ = false;
#if AUDIO_I2S_RATE != AUDIO_SAMPLE_RATE_HZ
    Resampler rsDown_;   // ADC (débit I2S) -> SBC
    Resampler rsUp_;     // SBC -> DAC (débit I2S)
#endif
    Preroll<kPreroll> preroll_;
    bool     vox_ = false;
    uint32_t voxLoudMs_ = 0;
};
