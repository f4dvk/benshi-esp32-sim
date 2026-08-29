#pragma once
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"

// ============================================================================
// Lecteur NMEA minimal pour balise APRS "tracker" (position dynamique).
//
// Analyse $--RMC (position + validité) et $--GGA (position) sur un UART libre
// (Serial1, GPIO configurables). Ne dépend d'aucune bibliothèque.
//
//   gps.begin();                 // dans setup()
//   gps.poll();                  // souvent (boucle Arduino)
//   double lat, lon;
//   if (gps.fix(lat, lon)) { ... position valide et récente ... }
// ============================================================================

#if APRS_GPS_ENABLE

class GpsNmea {
public:
    void begin() {
        Serial1.begin(APRS_GPS_BAUD, SERIAL_8N1, APRS_GPS_RX_GPIO, APRS_GPS_TX_GPIO);
        Serial.printf("[GPS] NMEA sur Serial1 RX=GPIO%d @ %d bauds\n",
                      APRS_GPS_RX_GPIO, APRS_GPS_BAUD);
    }

    void poll() {
        while (Serial1.available()) {
            char c = (char)Serial1.read();
            if (c == '\n' || c == '\r') {
                if (len_ > 6) parseLine();
                len_ = 0;
            } else if (len_ < sizeof(line_) - 1) {
                line_[len_++] = c;
            } else {
                len_ = 0;   // ligne trop longue -> on jette
            }
        }
    }

    // Renvoie true si un fix valide date de moins de APRS_GPS_FIX_MAX_AGE_S.
    bool fix(double& lat, double& lon) const {
        if (!haveFix_) return false;
        if (millis() - fixMs_ > (uint32_t)APRS_GPS_FIX_MAX_AGE_S * 1000UL) return false;
        lat = lat_; lon = lon_;
        return true;
    }
    bool    hasFix() const { double a, b; return fix(a, b); }
    uint8_t sats()   const { return hasFix() ? sats_ : 0; }
    uint8_t fixType() const { return hasFix() ? fixType_ : 0; }   // 0/2/3

private:
    // Découpe en champs séparés par des virgules (destructif sur line_).
    int split(char* fields[], int maxf) {
        int n = 0;
        char* p = line_;
        fields[n++] = p;
        while (*p && n < maxf) {
            if (*p == ',') { *p = '\0'; fields[n++] = p + 1; }
            p++;
        }
        return n;
    }

    // NMEA "ddmm.mmmm" + hémisphère -> degrés décimaux.
    static double nmeaToDeg(const char* v, const char* hemi) {
        if (!v || !*v) return NAN;
        double raw = atof(v);
        double deg = floor(raw / 100.0);
        double minutes = raw - deg * 100.0;
        double d = deg + minutes / 60.0;
        if (hemi && (*hemi == 'S' || *hemi == 'W')) d = -d;
        return d;
    }

    void parseLine() {
        if (line_[0] != '$') return;
        // type = 3 lettres après le "talker" (ex. GPRMC, GNRMC) -> line_[3..5]
        const char* t = line_ + 3;
        char* f[20];
        int nf = split(f, 20);

        if (!strncmp(t, "RMC", 3) && nf >= 7) {
            bool valid = (f[2][0] == 'A');
            double lat = nmeaToDeg(f[3], f[4]);
            double lon = nmeaToDeg(f[5], f[6]);
            if (valid && !isnan(lat) && !isnan(lon)) store(lat, lon);
        } else if (!strncmp(t, "GGA", 3) && nf >= 8) {
            int quality = atoi(f[6]);
            sats_ = (uint8_t)atoi(f[7]);
            double lat = nmeaToDeg(f[2], f[3]);
            double lon = nmeaToDeg(f[4], f[5]);
            if (quality > 0 && !isnan(lat) && !isnan(lon)) store(lat, lon);
        } else if (!strncmp(t, "GSA", 3) && nf >= 3) {
            fixType_ = (uint8_t)atoi(f[2]);   // 1=aucun, 2=2D, 3=3D
        }
    }

    void store(double lat, double lon) {
        lat_ = lat; lon_ = lon;
        fixMs_ = millis();
        if (!haveFix_) Serial.printf("[GPS] fix acquis : %.6f, %.6f\n", lat, lon);
        haveFix_ = true;
    }

    char     line_[100];
    size_t   len_ = 0;
    double   lat_ = 0, lon_ = 0;
    uint32_t fixMs_ = 0;
    bool     haveFix_ = false;
    uint8_t  sats_ = 0;
    uint8_t  fixType_ = 0;
};

#endif  // APRS_GPS_ENABLE
