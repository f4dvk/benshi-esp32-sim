#pragma once
#include <Arduino.h>
#include <new>
#include <functional>
#include "config.h"

#if DISPLAY_ENABLE

#include <Wire.h>
#include "RadioFace.h"
#include "RadioDisplay.h"     // vide si DISPLAY_HAS_ILI9225 est faux
#include "NextionDisplay.h"   // vide si DISPLAY_HAS_NEXTION est faux

// ============================================================================
// Aiguilleur d'écran de façade.
//
// DISPLAY_DRIVER = ILI9225 / NEXTION -> le pilote est fixé à la compilation.
// DISPLAY_DRIVER = AUTO              -> détection au démarrage :
//     - MCP23017 qui répond sur l'I2C           -> pilote ILI9225
//     - "comok" reçu sur l'UART Nextion         -> pilote Nextion
//     - rien                                    -> aucun pilote (0 octet de tas)
//
// Le pilote détecté est alloué sur le TAS (via start(), différé), pas en .bss :
// "pas d'écran" ne coûte donc rien, et un seul pilote occupe la mémoire.
// ============================================================================

class FaceDisplay {
public:
    enum Kind { NONE, ILI9225, NEXTION };
    using PcmSource = std::function<void(int16_t*)>;

    void setPcmSource(PcmSource fn) { pcm_ = std::move(fn); }

    // Sonde le matériel (une seule fois) et mémorise le type d'écran.
    Kind detect() {
        if (detected_) return kind_;
        detected_ = true;
#if DISPLAY_DRIVER == DISPLAY_DRIVER_ILI9225
        kind_ = ILI9225;
#elif DISPLAY_DRIVER == DISPLAY_DRIVER_NEXTION
        kind_ = NEXTION;
#else
        if      (probeIli9225()) kind_ = ILI9225;
        else if (probeNextion()) kind_ = NEXTION;
        else                     kind_ = NONE;
#endif
        Serial.printf("[DISP] ecran detecte : %s\n",
                      kind_ == ILI9225 ? "ILI9225 (MCP23017)" :
                      kind_ == NEXTION ? "Nextion (UART)" : "aucun");
        return kind_;
    }

    Kind kind() const { return kind_; }

    // Alloue et démarre le pilote détecté. À appeler une fois, en différé
    // (après publication du service SDP). Renvoie false si aucun / échec.
    bool start() {
#if DISPLAY_HAS_ILI9225
        if (kind_ == ILI9225) {
            ili_ = new (std::nothrow) RadioDisplay();
            if (!ili_) { Serial.println("[DISP] alloc pilote ILI9225 impossible"); return false; }
            ili_->setPcmSource(pcm_);
            return ili_->begin();
        }
#endif
#if DISPLAY_HAS_NEXTION
        if (kind_ == NEXTION) {
            nex_ = new (std::nothrow) NextionDisplay();
            if (!nex_) { Serial.println("[DISP] alloc pilote Nextion impossible"); return false; }
            nex_->setPcmSource(pcm_);
            return nex_->begin();
        }
#endif
        return false;
    }

    void set(const RadioFace& f) {
#if DISPLAY_HAS_ILI9225
        if (ili_) { ili_->set(f); return; }
#endif
#if DISPLAY_HAS_NEXTION
        if (nex_) { nex_->set(f); return; }
#endif
        (void)f;
    }

private:
#if DISPLAY_DRIVER == DISPLAY_DRIVER_AUTO
    // MCP23017 (donc écran ILI9225) présent sur l'I2C ?
    static bool probeIli9225() {
        Wire.begin(DISPLAY_I2C_SDA, DISPLAY_I2C_SCL, 100000);
        Wire.beginTransmission(MCP23017_ADDR);
        bool ok = (Wire.endTransmission() == 0);
        Wire.end();                       // libère 21/22 pour un éventuel Nextion
        return ok;
    }
    // Un Nextion répond "comok" à la commande "connect" (aux 2 débits usuels).
    static bool probeNextion() {
        const int bauds[2] = { NEXTION_BAUD, 9600 };
        static const uint8_t req[] =
            { 0xFF, 0xFF, 0xFF, 'c','o','n','n','e','c','t', 0xFF, 0xFF, 0xFF };
        for (int i = 0; i < 2; i++) {
            Serial1.begin(bauds[i], SERIAL_8N1, NEXTION_RX_GPIO, NEXTION_TX_GPIO);
            delay(30);
            while (Serial1.available()) Serial1.read();
            Serial1.write(req, sizeof(req));
            Serial1.flush();
            const char* want = "comok";
            int m = 0;
            uint32_t t0 = millis();
            while (millis() - t0 < 250) {
                while (Serial1.available()) {
                    char c = (char)Serial1.read();
                    m = (c == want[m]) ? m + 1 : (c == want[0] ? 1 : 0);
                    if (m == 5) { Serial1.end(); return true; }
                }
                delay(4);
            }
            Serial1.end();
        }
        return false;
    }
#endif  // AUTO

    Kind      kind_ = NONE;
    bool      detected_ = false;
    PcmSource pcm_;
#if DISPLAY_HAS_ILI9225
    RadioDisplay*   ili_ = nullptr;
#endif
#if DISPLAY_HAS_NEXTION
    NextionDisplay* nex_ = nullptr;
#endif
};

#endif  // DISPLAY_ENABLE
