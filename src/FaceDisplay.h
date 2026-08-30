#pragma once
#include <Arduino.h>
#include <new>
#include <functional>
#include "config.h"

#if DISPLAY_ENABLE

#include <SPI.h>
#include "RadioFace.h"
#include "RadioDisplay.h"      // ILI9225 (vide si DISPLAY_HAS_ILI9225 faux)
#include "RadioDisplay341.h"   // ILI9341 (vide si DISPLAY_HAS_ILI9341 faux)

// ============================================================================
// Aiguilleur d'écran de façade (SPI direct, plus de MCP23017).
//
// DISPLAY_DRIVER = ILI9225 / ILI9341 -> pilote fixé à la compilation.
// DISPLAY_DRIVER = AUTO              -> détection au démarrage par lecture de
//   l'ID sur le bus SPI :  0x9341 -> ILI9341 (+ tactile si présent),
//                          0x9225 -> ILI9225,  sinon -> aucun pilote.
//
// Le pilote détecté est alloué sur le TAS (start(), différé), pas en .bss.
// ============================================================================

class FaceDisplay {
public:
    enum Kind { NONE, ILI9225, ILI9341 };
    using PcmSource   = std::function<void(int16_t*)>;
    using TouchAction = std::function<void(int)>;

    void setPcmSource(PcmSource fn)     { pcm_ = std::move(fn); }
    void setTouchAction(TouchAction fn) { touchCb_ = std::move(fn); }

    Kind detect() {
        if (detected_) return kind_;
        detected_ = true;
        // UN seul bus SPI (HSPI), initialisé ici, réutilisé par les sondes ET
        // le pilote -> pas de cycle new/begin/end/delete de SPIClass.
        spi_.begin(DISPLAY_SPI_SCK, DISPLAY_SPI_MISO, DISPLAY_SPI_MOSI, -1);
#if DISPLAY_DRIVER == DISPLAY_DRIVER_ILI9225
        kind_ = ILI9225;
#elif DISPLAY_DRIVER == DISPLAY_DRIVER_ILI9341
        kind_ = ILI9341;
#else
        uint32_t id41 = Ili9341::probeId(&spi_);
        Serial.printf("[DISP] ID SPI (0xD3) = 0x%06X\n", id41);
        if ((id41 & 0xFFFF) == 0x9341) { kind_ = ILI9341; }
        else {
            uint32_t id25 = Ili9225::probeId(&spi_);
            Serial.printf("[DISP] ID SPI (reg0) = 0x%04X\n", id25);
            if (id25 == 0x9225) {
                kind_ = ILI9225;
            } else if (DISPLAY_AUTO_FALLBACK_ILI9225) {
                kind_ = ILI9225;
                Serial.println("[DISP] pas d'ID lisible -> ILI9225 par defaut (repli)");
            } else {
                kind_ = NONE;
            }
        }
#endif
        Serial.printf("[DISP] ecran detecte : %s\n",
                      kind_ == ILI9225 ? "ILI9225" :
                      kind_ == ILI9341 ? "ILI9341 (SPI)" : "aucun");
        return kind_;
    }
    Kind kind() const { return kind_; }

    bool start() {
#if DISPLAY_HAS_ILI9225
        if (kind_ == ILI9225) {
            ili_ = new (std::nothrow) RadioDisplay();
            if (!ili_) { Serial.println("[DISP] alloc ILI9225 impossible"); return false; }
            ili_->setPcmSource(pcm_);
            return ili_->begin(&spi_);
        }
#endif
#if DISPLAY_HAS_ILI9341
        if (kind_ == ILI9341) {
            d341_ = new (std::nothrow) RadioDisplay341();
            if (!d341_) { Serial.println("[DISP] alloc ILI9341 impossible"); return false; }
            d341_->setPcmSource(pcm_);
            d341_->setTouchAction(touchCb_);
            return d341_->begin(&spi_);
        }
#endif
        return false;
    }

    void set(const RadioFace& f) {
#if DISPLAY_HAS_ILI9225
        if (ili_) { ili_->set(f); return; }
#endif
#if DISPLAY_HAS_ILI9341
        if (d341_) { d341_->set(f); return; }
#endif
        (void)f;
    }

private:
    SPIClass    spi_{HSPI};
    Kind        kind_ = NONE;
    bool        detected_ = false;
    PcmSource   pcm_;
    TouchAction touchCb_;
#if DISPLAY_HAS_ILI9225
    RadioDisplay*    ili_  = nullptr;
#endif
#if DISPLAY_HAS_ILI9341
    RadioDisplay341* d341_ = nullptr;
#endif
};

#endif  // DISPLAY_ENABLE
