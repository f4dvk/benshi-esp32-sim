#pragma once
#include <Arduino.h>

// ============================================================================
// Instantané de l'état radio poussé vers l'écran de façade, quel que soit le
// pilote (ILI9225 via MCP23017, ou Nextion via UART).
// ============================================================================

struct RadioFace {
    double  rxMHz      = 0.0;
    uint8_t channelId  = 0;
    char    channel[12] = {0};
    bool    wide       = true;
    bool    highPower  = true;
    uint8_t sMeter     = 0;      // 0..9
    bool    sqOpen     = false;
    bool    tx         = false;
    bool    txAprs     = false;  // émission = balise APRS autonome
    bool    bt         = false;
    uint8_t gpsFix     = 0;      // 0 / 2 / 3
    uint8_t gpsSats    = 0;
    char    utc[9]     = {0};    // "HH:MM:SS" (GPS), vide si indisponible
};
