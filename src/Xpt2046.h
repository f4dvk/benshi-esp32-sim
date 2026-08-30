#pragma once
#include <Arduino.h>
#include "config.h"

#if (DISPLAY_HAS_ILI9341 && TOUCH_ENABLE)

#include <SPI.h>

// ============================================================================
// Pilote minimal tactile résistif XPT2046 (SPI, mode 0, <= 2 MHz).
//
// Bus SPI PARTAGÉ avec l'ILI9341 (mêmes SCK/MOSI/MISO), CS dédié (TOUCH_CS_PIN).
// present() sonde la puce : si elle ne répond pas, le tactile est désactivé et
// la tâche ne le lit jamais.
// ============================================================================

#ifndef TOUCH_CAL_X0
#define TOUCH_CAL_X0  300     // brut à gauche
#define TOUCH_CAL_X1  3800    // brut à droite
#define TOUCH_CAL_Y0  300     // brut en haut
#define TOUCH_CAL_Y1  3800    // brut en bas
#endif

class Xpt2046 {
public:
    // spi = le même SPIClass que l'ILI9341 (bus partagé).
    void begin(SPIClass* spi) {
        spi_ = spi;
        pinMode(TOUCH_CS_PIN, OUTPUT);
        digitalWrite(TOUCH_CS_PIN, HIGH);
        if (TOUCH_IRQ_PIN >= 0) pinMode(TOUCH_IRQ_PIN, INPUT_PULLUP);
    }

    // Détection : 2 lectures X/Y d'affilée. Puce présente = valeurs stables et
    // non bloquées à 0 / 4095 (MISO flottant = bruit aléatoire).
    bool present() {
        int x1 = readCh(0xD1), x2 = readCh(0xD1);
        int y1 = readCh(0x91), y2 = readCh(0x91);
        readCh(0x00);   // power-down
        bool stable = (abs(x1 - x2) < 300) && (abs(y1 - y2) < 300);
        bool stuck  = (x1 <= 1 && y1 <= 1) || (x1 >= 4094 && y1 >= 4094);
        return stable && !stuck;
    }

    // Renvoie true si un appui est détecté ; (sx, sy) = coordonnées écran.
    bool read(int& sx, int& sy) {
        if (TOUCH_IRQ_PIN >= 0 && digitalRead(TOUCH_IRQ_PIN)) return false;  // PENIRQ haut = pas d'appui
        int z1 = readCh(0xB1);
        int z2 = readCh(0xC1);
        int z  = z1 + (4095 - z2);
        if (z < 400) { readCh(0x00); return false; }
        // médiane de 3 lectures par axe (anti-bruit)
        int xr = med3(readCh(0xD1), readCh(0xD1), readCh(0xD1));
        int yr = med3(readCh(0x91), readCh(0x91), readCh(0x91));
        readCh(0x00);
        long x = (long)(xr - TOUCH_CAL_X0) * 320 / (TOUCH_CAL_X1 - TOUCH_CAL_X0);
        long y = (long)(yr - TOUCH_CAL_Y0) * 240 / (TOUCH_CAL_Y1 - TOUCH_CAL_Y0);
        sx = constrain((int)x, 0, 319);
        sy = constrain((int)y, 0, 239);
        return true;
    }

private:
    int readCh(uint8_t ctrl) {
        spi_->beginTransaction(SPISettings(TOUCH_SPI_HZ, MSBFIRST, SPI_MODE0));
        digitalWrite(TOUCH_CS_PIN, LOW);
        spi_->transfer(ctrl);
        uint8_t hi = spi_->transfer(0), lo = spi_->transfer(0);
        digitalWrite(TOUCH_CS_PIN, HIGH);
        spi_->endTransaction();
        return ((hi << 8) | lo) >> 3;   // 12 bits
    }
    static int med3(int a, int b, int c) {
        if (a > b) { int t = a; a = b; b = t; }
        if (b > c) { int t = b; b = c; c = t; }
        if (a > b) { int t = a; a = b; b = t; }
        return b;
    }

    SPIClass* spi_ = nullptr;
};

#endif  // DISPLAY_HAS_ILI9341 && TOUCH_ENABLE
