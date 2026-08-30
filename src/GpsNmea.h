#pragma once
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"

// ============================================================================
// Lecteur NMEA minimal pour balise APRS "tracker" (position dynamique).
//
// Analyse $--RMC (position + validité) et $--GGA (position) sur Serial1
// (UART1 matériel, RX seul, APRS_GPS_RX_GPIO). Le checksum NMEA est vérifié.
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
        Serial1.begin(APRS_GPS_BAUD, SERIAL_8N1, APRS_GPS_RX_GPIO, -1);
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

    // Heure UTC (issue du GPS) si reçue depuis moins de APRS_GPS_FIX_MAX_AGE_S.
    bool utc(uint8_t& h, uint8_t& m, uint8_t& s) const {
        if (!haveUtc_) return false;
        if (millis() - utcMs_ > (uint32_t)APRS_GPS_FIX_MAX_AGE_S * 1000UL) return false;
        h = utcH_; m = utcM_; s = utcS_;
        return true;
    }

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
        // Checksum NMEA "$....*HH" : rejette une trame corrompue.
        char* star = strchr(line_, '*');
        if (star && star[1] && star[2]) {
            uint8_t cs = 0;
            for (char* p = line_ + 1; p < star; p++) cs ^= (uint8_t)*p;
            if (cs != (uint8_t)strtol(star + 1, nullptr, 16)) return;
            *star = '\0';
        }
        // type = 3 lettres après le "talker" (ex. GPRMC, GNRMC) -> line_[3..5]
        const char* t = line_ + 3;
        char* f[20];
        int nf = split(f, 20);

        if (!strncmp(t, "RMC", 3) && nf >= 7) {
            parseUtc(f[1]);
            bool valid = (f[2][0] == 'A');
            double lat = nmeaToDeg(f[3], f[4]);
            double lon = nmeaToDeg(f[5], f[6]);
            if (valid && !isnan(lat) && !isnan(lon)) store(lat, lon);
        } else if (!strncmp(t, "GGA", 3) && nf >= 8) {
            parseUtc(f[1]);
            int quality = atoi(f[6]);
            sats_ = (uint8_t)atoi(f[7]);
            double lat = nmeaToDeg(f[2], f[3]);
            double lon = nmeaToDeg(f[4], f[5]);
            if (quality > 0 && !isnan(lat) && !isnan(lon)) store(lat, lon);
        } else if (!strncmp(t, "GSA", 3) && nf >= 3) {
            fixType_ = (uint8_t)atoi(f[2]);   // 1=aucun, 2=2D, 3=3D
        }
    }

    // Champ NMEA "hhmmss.sss" -> heure UTC.
    void parseUtc(const char* v) {
        if (!v) return;
        size_t n = 0; while (v[n] && v[n] != '.') n++;
        if (n < 6) return;
        for (int i = 0; i < 6; i++) if (v[i] < '0' || v[i] > '9') return;
        utcH_ = (v[0] - '0') * 10 + (v[1] - '0');
        utcM_ = (v[2] - '0') * 10 + (v[3] - '0');
        utcS_ = (v[4] - '0') * 10 + (v[5] - '0');
        if (utcH_ > 23 || utcM_ > 59 || utcS_ > 59) return;
        utcMs_ = millis();
        haveUtc_ = true;
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
    uint8_t  utcH_ = 0, utcM_ = 0, utcS_ = 0;
    uint32_t utcMs_ = 0;
    bool     haveUtc_ = false;
};

#endif  // APRS_GPS_ENABLE
