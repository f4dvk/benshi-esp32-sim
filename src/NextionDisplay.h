#pragma once
#include <Arduino.h>
#include <stdarg.h>
#include "config.h"

#if DISPLAY_HAS_NEXTION

#include <functional>
#include "RadioFace.h"
#include "AudioSpectrum.h"

// ============================================================================
// Écran de façade sur afficheur "intelligent" NEXTION (NX4827T043, 480x272).
//
// Le Nextion fait tout le rendu ; l'ESP ne fait qu'ENVOYER des commandes ASCII
// sur un UART (Serial1), terminées par 0xFF 0xFF 0xFF. Affichage PASSIF : on
// n'exploite pas le tactile (les octets reçus sont simplement ignorés/drainés).
//
// L'INTERFACE (.HMI) est à créer dans le Nextion Editor et à flasher à part.
// Composants attendus dans "page0" (voir README) :
//   tFreq tChan tMode tStat tSq tPwr tBt tGps tUtc  : Text
//   jSig                                            : Progress bar (0..100)
//   sSpec                                           : Waveform, id = NEXTION_WAVE_ID
// Les couleurs .pco sont en RGB565.
// ============================================================================

// NEXTION_WAVE_ID / _W / _H viennent de config.h (repli si absents).
#ifndef NEXTION_WAVE_ID
#define NEXTION_WAVE_ID 10
#endif
#ifndef NEXTION_WAVE_W
#define NEXTION_WAVE_W  272
#endif
#ifndef NEXTION_WAVE_H
#define NEXTION_WAVE_H  64
#endif

class NextionDisplay {
public:
    using PcmSource = std::function<void(int16_t*)>;
    void setPcmSource(PcmSource fn) { pcm_ = std::move(fn); }

    bool begin() {
        if (ESP.getFreeHeap() < 15000) {
            Serial.println("[NEXT] tas trop juste -> ecran non demarre");
            return false;
        }
        mtx_ = xSemaphoreCreateMutex();
        ser_ = &Serial1;
        // Un Nextion neuf est à 9600 bauds : on tente de le passer à NEXTION_BAUD
        // (sans effet s'il y est déjà), puis on rouvre au bon débit.
        if (NEXTION_BAUD != 9600) {
            ser_->begin(9600, SERIAL_8N1, NEXTION_RX_GPIO, NEXTION_TX_GPIO);
            delay(120);
            cmd("");
            cmdf("bauds=%d", NEXTION_BAUD);   // persistant (débit inchangé pour l'instant)
            ser_->flush(); delay(50);
            cmdf("baud=%d", NEXTION_BAUD);    // bascule immédiate
            ser_->flush(); delay(50);
            ser_->end();
        }
        ser_->begin(NEXTION_BAUD, SERIAL_8N1, NEXTION_RX_GPIO, NEXTION_TX_GPIO);
        delay(150);                       // laisser le Nextion démarrer
        cmd("");                          // clôt une commande partielle éventuelle
        cmd("bkcmd=0");                   // pas d'accusé -> on ne lit rien
        cmd("page 0");
        cmdf("cle %d,0", NEXTION_WAVE_ID);   // vide la waveform
        Serial.printf("[NEXT] Nextion sur Serial1 RX=%d TX=%d @ %d bauds\n",
                      NEXTION_RX_GPIO, NEXTION_TX_GPIO, NEXTION_BAUD);
        xTaskCreatePinnedToCore(&NextionDisplay::trampoline, "nextion", 4096, this, 1, nullptr, 1);
        return true;
    }

    void set(const RadioFace& f) {
        if (!mtx_) return;
        xSemaphoreTake(mtx_, portMAX_DELAY);
        pending_ = f;
        xSemaphoreGive(mtx_);
    }

private:
    static void trampoline(void* s) { static_cast<NextionDisplay*>(s)->loop(); }

    void loop() {
        uint32_t lastSpec = 0;
        for (;;) {
            RadioFace f;
            xSemaphoreTake(mtx_, portMAX_DELAY);
            f = pending_;
            xSemaphoreGive(mtx_);
            render(f);
#if DISPLAY_SPECTRUM
            if (pcm_ && millis() - lastSpec >= NEXTION_SPECTRUM_MS) {
                lastSpec = millis();
                sendSpectrum(f.sqOpen && !f.tx);
            }
#endif
            drainRx();
            vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH_MS));
        }
    }

    // ---- envoi bas niveau ------------------------------------------------
    void cmd(const char* s) {
        ser_->print(s);
        static const uint8_t end[3] = {0xFF, 0xFF, 0xFF};
        ser_->write(end, 3);
    }
    void cmdf(const char* fmt, ...) {
        char b[72]; va_list a; va_start(a, fmt);
        vsnprintf(b, sizeof(b), fmt, a); va_end(a);
        cmd(b);
    }
    void setTxt(const char* obj, const char* v) { cmdf("%s.txt=\"%s\"", obj, v); }
    void setVal(const char* obj, int v)         { cmdf("%s.val=%d", obj, v); }
    void setPco(const char* obj, uint16_t rgb565) { cmdf("%s.pco=%u", obj, rgb565); }
    void drainRx() { while (ser_->available()) ser_->read(); }

    // ---- rendu des champs (envoi des seuls changements) -----------------
    static const uint16_t C_RED = 0xF800, C_GRN = 0x07E0, C_AMB = 0xFD20;
    static const uint16_t C_CYN = 0x07FF, C_GRY = 0x8410, C_WHT = 0xFFFF;

    void render(const RadioFace& f) {
        char b[40];
        bool F = first_;

        if (F || f.rxMHz != c_.rxMHz) {
            snprintf(b, sizeof(b), "%.4f", f.rxMHz);
            setTxt("tFreq", b);
            setPco("tFreq", f.tx ? C_RED : C_WHT);
        } else if (f.tx != c_.tx) {
            setPco("tFreq", f.tx ? C_RED : C_WHT);
        }
        if (F || f.channelId != c_.channelId || strcmp(f.channel, c_.channel)) {
            snprintf(b, sizeof(b), "M%02u %s", f.channelId, f.channel);
            setTxt("tChan", b);
        }
        if (F || f.wide != c_.wide) {
            snprintf(b, sizeof(b), "FM  %s", f.wide ? "WIDE 25k" : "NARR 12k");
            setTxt("tMode", b);
        }
        if (F || f.sMeter != c_.sMeter)
            setVal("jSig", (int)f.sMeter * 100 / 9);

        if (F || f.tx != c_.tx || f.txAprs != c_.txAprs || f.sqOpen != c_.sqOpen) {
            const char* s; uint16_t col;
            if (f.tx && f.txAprs) { s = "TX APRS"; col = C_RED; }
            else if (f.tx)        { s = "TX";      col = C_RED; }
            else if (f.sqOpen)    { s = "RX";      col = C_GRN; }
            else                  { s = "STBY";    col = C_GRY; }
            setTxt("tStat", s);
            setPco("tStat", col);
        }
        if (F || f.sqOpen != c_.sqOpen) {
            setTxt("tSq", f.sqOpen ? "SQ" : "");
        }
        if (F || f.highPower != c_.highPower) {
            setTxt("tPwr", f.highPower ? "HI" : "LO");
            setPco("tPwr", f.highPower ? C_AMB : C_CYN);
        }
        if (F || f.bt != c_.bt) {
            setTxt("tBt", "BT");
            setPco("tBt", f.bt ? C_CYN : C_GRY);
        }
        if (F || f.gpsFix != c_.gpsFix || f.gpsSats != c_.gpsSats) {
            if (f.gpsFix >= 2) snprintf(b, sizeof(b), "GPS %uD %02u", f.gpsFix, f.gpsSats);
            else               snprintf(b, sizeof(b), "GPS --");
            setTxt("tGps", b);
            setPco("tGps", f.gpsFix >= 2 ? C_GRN : C_GRY);
        }
        if (F || strcmp(f.utc, c_.utc)) {
            snprintf(b, sizeof(b), "UTC %s", f.utc[0] ? f.utc : "--:--:--");
            setTxt("tUtc", b);
        }

        c_ = f;
        first_ = false;
    }

#if DISPLAY_SPECTRUM
    // ---- spectre audio -> composant Waveform (mode "addt", transparent) --
    static const int SP_N = 256;              // = AudioBridge::kSpecN
    static const int SP_BARS = 24;            // points du spectre (interpolés sur la waveform)
    static const int SP_MAXBIN = 24;          // ~3 kHz @ 32 kHz

    void sendSpectrum(bool live) {
        if (live) {
            pcm_(specPcm_);
            spectrum_.compute(specPcm_, specVal_, SP_BARS, SP_MAXBIN);
            specEmpty_ = false;
        } else {
            bool any = false;
            for (int i = 0; i < SP_BARS; i++) {
                if (specVal_[i]) any = true;
                specVal_[i] = specVal_[i] > 32 ? (uint8_t)(specVal_[i] - 32) : 0;
            }
            if (!any) {
                if (specEmpty_) return;
                specEmpty_ = true;
            }
        }

        uint8_t buf[NEXTION_WAVE_W];
        for (int x = 0; x < NEXTION_WAVE_W; x++) {
            int bi = (SP_BARS > 1) ? x * (SP_BARS - 1) / (NEXTION_WAVE_W - 1) : 0;
            buf[x] = (uint8_t)((int)specVal_[bi] * (NEXTION_WAVE_H - 1) / 255);
        }

        // addt <id>,<ch>,<qty> : le Nextion passe en réception transparente et
        // attend <qty> octets bruts. On lui laisse ~8 ms puis on envoie tout
        // (plus robuste que d'attendre l'octet 0xFE, qui peut manquer et
        // laisser l'écran désynchronisé).
        drainRx();
        cmdf("addt %d,0,%d", NEXTION_WAVE_ID, NEXTION_WAVE_W);
        delay(8);
        ser_->write(buf, NEXTION_WAVE_W);
        ser_->flush();
    }
#endif

    SemaphoreHandle_t mtx_ = nullptr;
    HardwareSerial*   ser_ = nullptr;
    RadioFace pending_, c_;
    bool      first_ = true;
    PcmSource pcm_;
#if DISPLAY_SPECTRUM
    AudioSpectrum<SP_N> spectrum_;
    int16_t   specPcm_[SP_N];
    uint8_t   specVal_[SP_BARS] = {0};
    bool      specEmpty_ = true;
#endif
};

#endif  // DISPLAY_HAS_NEXTION
