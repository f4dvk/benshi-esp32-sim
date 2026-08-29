#pragma once
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include "config.h"

// ============================================================================
// Construction d'une trame APRS de position (AX.25 UI), 100 % côté ESP.
//
//   [adresse DEST 7o][adresse SOURCE 7o][digi 7o]*N [0x03][0xF0][champ info]
//
// Le champ info est une position SANS horodatage, format APRS :
//   ! DDMM.mmN <table> DDDMM.mmW <symbole> <commentaire>
//
// La FCS (CRC AX.25), le préambule de fanions et le bit-stuffing sont ajoutés
// par le modulateur AFSK (esp32-afsk) -> NE PAS les mettre ici.
// ============================================================================

namespace aprs {

// --- Encodage d'un champ d'adresse AX.25 (6 caractères + octet SSID) --------
//   chars : décalés d'1 bit vers la gauche, complétés par des espaces
//   octet SSID : 0b0110_SSSS_E  (E = 1 sur la DERNIÈRE adresse de la liste)
//                bit 7 = C (dest) / H "répété" (digi) ; 0 pour une émission
inline void putAddr(uint8_t* p, const char* call, int ssid, bool last, bool cbit) {
    size_t n = call ? strnlen(call, 6) : 0;
    for (size_t i = 0; i < 6; i++) {
        char c = (i < n) ? (char)toupper((unsigned char)call[i]) : ' ';
        p[i] = (uint8_t)((uint8_t)c << 1);
    }
    if (ssid < 0)  ssid = 0;
    if (ssid > 15) ssid = 15;
    p[6] = (uint8_t)(0x60 | ((ssid & 0x0F) << 1) | (last ? 0x01 : 0x00) | (cbit ? 0x80 : 0x00));
}

// "WIDE1-1" -> call="WIDE1", ssid=1 ; "WIDE2" -> ssid 0. Renvoie false si vide.
inline bool parsePathToken(const char* tok, size_t len, char call[7], int& ssid) {
    while (len && (*tok == ' ')) { tok++; len--; }
    while (len && (tok[len - 1] == ' ')) len--;
    if (!len) return false;
    size_t dash = len;
    for (size_t i = 0; i < len; i++) if (tok[i] == '-') { dash = i; break; }
    size_t cl = dash < 6 ? dash : 6;
    memset(call, 0, 7);
    memcpy(call, tok, cl);
    ssid = 0;
    if (dash < len) ssid = atoi(tok + dash + 1);
    return true;
}

// Position (degrés décimaux) -> "DDMM.mmN" (lat) ou "DDDMM.mmW" (lon).
inline void formatLat(double lat, char out[9]) {
    char hemi = lat >= 0 ? 'N' : 'S';
    double a = fabs(lat);
    int deg = (int)a;
    double minutes = (a - deg) * 60.0;
    if (minutes >= 59.995) { minutes = 0.0; deg++; }
    snprintf(out, 9, "%02d%05.2f%c", deg, minutes, hemi);
}
inline void formatLon(double lon, char out[10]) {
    char hemi = lon >= 0 ? 'E' : 'W';
    double a = fabs(lon);
    int deg = (int)a;
    double minutes = (a - deg) * 60.0;
    if (minutes >= 59.995) { minutes = 0.0; deg++; }
    snprintf(out, 10, "%03d%05.2f%c", deg, minutes, hemi);
}

// Écrit la trame complète dans `out` (capacité `cap`). Renvoie sa longueur,
// ou 0 en cas de dépassement / configuration invalide.
// Paramètres de balise : réglables depuis HTCommander (voir AprsConfig).
struct BeaconParams {
    const char* callsign  = APRS_CALLSIGN;
    int         ssid      = APRS_SSID;
    const char* path      = APRS_PATH;         // "" = direct
    char        symTable  = APRS_SYMBOL_TABLE;
    char        symCode   = APRS_SYMBOL;
    const char* comment   = APRS_COMMENT;
    const char* dest      = APRS_DEST;         // TOCALL, non configurable par HTCommander
};

inline size_t buildPositionFrame(uint8_t* out, size_t cap,
                                 const BeaconParams& bp, double lat, double lon) {
    size_t o = 0;

    // 1) Adresse destination (TOCALL) : bit C = 1
    if (o + 7 > cap) return 0;
    putAddr(out + o, bp.dest, 0, false, true);
    o += 7;

    // 2) Adresse source (notre indicatif) : bit C = 0. Dernière adresse si
    //    aucun digipeater dans le chemin.
    const char* path = bp.path ? bp.path : "";
    bool hasPath = path[0] != '\0';
    if (o + 7 > cap) return 0;
    putAddr(out + o, bp.callsign, bp.ssid, !hasPath, false);
    o += 7;

    // 3) Digipeaters du chemin
    if (hasPath) {
        const char* p = path;
        while (*p) {
            const char* comma = strchr(p, ',');
            size_t len = comma ? (size_t)(comma - p) : strlen(p);
            char call[7]; int ssid;
            bool last = !comma;
            if (parsePathToken(p, len, call, ssid)) {
                if (o + 7 > cap) return 0;
                putAddr(out + o, call, ssid, last, false);
                o += 7;
            }
            if (!comma) break;
            p = comma + 1;
        }
        // Marque la vraie dernière adresse (bit E) au cas où le dernier token
        // serait vide.
        if (o >= 7) out[o - 1] |= 0x01;
    }

    // 4) Control (UI) + PID (pas de couche 3)
    if (o + 2 > cap) return 0;
    out[o++] = 0x03;
    out[o++] = 0xF0;

    // 5) Champ info : position sans horodatage
    char latS[9], lonS[10];
    formatLat(lat, latS);
    formatLon(lon, lonS);
    char info[160];
    int n = snprintf(info, sizeof(info), "!%s%c%s%c%s",
                     latS, bp.symTable, lonS, bp.symCode,
                     bp.comment ? bp.comment : "");
    if (n < 0) return 0;
    if (n >= (int)sizeof(info)) n = (int)sizeof(info) - 1;   // commentaire tronqué
#if (APRS_FIXED_ALT_M >= 0)
    // Altitude APRS : " /A=nnnnnn" en pieds
    if (n < (int)sizeof(info) - 12) {
        long feet = lroundf((float)APRS_FIXED_ALT_M * 3.28084f);
        int m = snprintf(info + n, sizeof(info) - n, " /A=%06ld", feet);
        if (m > 0) n += (m < (int)sizeof(info) - n ? m : (int)sizeof(info) - n - 1);
    }
#endif
    if (o + (size_t)n > cap) return 0;
    memcpy(out + o, info, (size_t)n);
    o += (size_t)n;

    return o;
}

}  // namespace aprs
