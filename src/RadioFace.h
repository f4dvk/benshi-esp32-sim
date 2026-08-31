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
    uint8_t sMeter     = 0;      // 0..15 : 0..9 = S0..S9, 10..15 = S9+ (jusqu'à ~+40 dB)
    bool    sqOpen     = false;
    bool    tx         = false;
    bool    txAprs     = false;  // émission = balise APRS autonome
    bool    bt         = false;
    uint8_t gpsFix     = 0;      // 0 / 2 / 3
    uint8_t gpsSats    = 0;
    char    utc[9]     = {0};    // "HH:MM:SS" (GPS), vide si indisponible
    char    callsign[12] = {0};  // indicatif HTCommander (onglet Licence), "F4DVK-7" ; vide si non réglé
    int8_t  shift      = 0;      // décalage TX/RX : +1 = shift +, -1 = shift -, 0 = simplex
    uint8_t tone       = 0;      // CTCSS : 0 = aucun, 1 = TX seul ("T"), 2 = TX+RX ("CT")

    // Double veille (UV-K1 piloté) : les deux VFO sont affichés, la grande
    // fréquence (rxMHz / channel) suit le VFO en cours de réception.
    bool    dualWatch  = false;
    uint8_t channelIdB = 0;
    char    channelB[12] = {0};
    double  rxMHzB     = 0.0;
    bool    activeIsB  = false;  // VFO en cours de réception = B
};
