#pragma once
#include <Arduino.h>
#include <math.h>
#include "config.h"

// ============================================================================
// Pilote minimal SA818 / DRA818 (module radio UART, protocole "AT+DMO...").
//
// Implémentation autonome (aucune dépendance ajoutée) des quelques commandes
// utiles, calquée sur la lib fatpat/arduino-dra818 employée par kv4p-ht :
//   AT+DMOCONNECT            -> handshake / détection
//   AT+DMOSETGROUP=...       -> bande passante, fréquences TX/RX, CTCSS, squelch
//   AT+DMOSETVOLUME=n        -> volume HP (1..8)
//   AT+SETFILTER=p,h,l       -> filtres (0 = actif)
//   RSSI?                    -> niveau reçu (SA818 uniquement)
// UART : 9600 8N1. Réponses du type "+DMOxxx:0\r\n" (0 = OK).
//
// Sert au "mode SA818" : si un module répond au démarrage, l'ESP pilote un
// vrai module radio (retune réel sur les fréquences des canaux de config.h)
// au lieu de simuler + passerelle GPIO vers un poste externe.
// ============================================================================

class Sa818 {
public:
    // Ouvre l'UART, alimente le module (PD haut) et tente `probes` handshakes.
    // Renvoie true si le module répond.
    bool begin(int rxPin, int txPin, int pdPin, int probes) {
        pdPin_ = pdPin;
        if (pdPin_ >= 0) {
            pinMode(pdPin_, OUTPUT);
            digitalWrite(pdPin_, HIGH);            // HIGH = module alimenté (kv4p-ht)
        }
        ser_ = &Serial2;
        ser_->begin(9600, SERIAL_8N1, rxPin, txPin);
        ser_->setTimeout(60);
        delay(500);                               // laisse le module démarrer

        for (int i = 0; i < probes; i++) {
            flushRx();
            if (sendExpectOk("AT+DMOCONNECT\r\n", 1000)) {
                present_ = true;
                Serial.printf("[SA818] Module RF détecté (tentative %d/%d)\n", i + 1, probes);
                return true;
            }
            Serial.printf("[SA818] Pas de réponse (%d/%d)\n", i + 1, probes);
            delay(600);
        }
        present_ = false;
        return false;
    }

    bool present() const { return present_; }

    // Fréquences en MHz, CTCSS en Hz (0 = aucune), squelch 0..8 (0 = ouvert).
    bool tune(double rxMHz, double txMHz, double rxCtcssHz, double txCtcssHz,
              bool wide, int squelch) {
        if (!present_) return false;
        char bufTx[10], bufRx[10], cmd[64];
        dtostrf(txMHz, 8, 4, bufTx);
        dtostrf(rxMHz, 8, 4, bufRx);
        if (squelch < 0) squelch = 0;
        if (squelch > 8) squelch = 8;
        snprintf(cmd, sizeof(cmd), "AT+DMOSETGROUP=%d,%s,%s,%04d,%d,%04d\r\n",
                 wide ? 1 : 0, bufTx, bufRx,
                 ctcssIndex(txCtcssHz), squelch, ctcssIndex(rxCtcssHz));
        flushRx();
        bool ok = sendExpectOk(cmd, 1000);
        Serial.printf("[SA818] SETGROUP %s%s", cmd, ok ? "" : "  -> ECHEC\n");
        return ok;
    }

    bool setVolume(int v) {
        if (!present_) return false;
        if (v < 1) v = 1;
        if (v > 8) v = 8;
        char cmd[24];
        snprintf(cmd, sizeof(cmd), "AT+DMOSETVOLUME=%d\r\n", v);
        flushRx();
        return sendExpectOk(cmd, 500);
    }

    // pre/high/low : true = filtre ACTIF (la trame AT envoie l'inverse).
    bool setFilters(bool pre, bool high, bool low) {
        if (!present_) return false;
        char cmd[28];
        snprintf(cmd, sizeof(cmd), "AT+SETFILTER=%d,%d,%d\r\n",
                 pre ? 0 : 1, high ? 0 : 1, low ? 0 : 1);
        flushRx();
        return sendExpectOk(cmd, 500);
    }

    // Renvoie 0..255, ou -1 si indisponible. Attention : monopolise l'UART
    // ~60 ms, à ne pas appeler en rafale pendant la réception audio.
    int readRssi() {
        if (!present_) return -1;
        flushRx();
        ser_->print("RSSI?\r\n");
        String r = ser_->readStringUntil('\n');   // "RSSI=NNN"
        int eq = r.indexOf('=');
        if (eq < 0) return -1;
        int v = r.substring(eq + 1).toInt();
        return (v >= 0 && v <= 255) ? v : -1;
    }

    void powerDown(bool pd) {
        if (pdPin_ >= 0) digitalWrite(pdPin_, pd ? LOW : HIGH);
    }

private:
    HardwareSerial* ser_ = nullptr;
    int  pdPin_ = -1;
    bool present_ = false;

    void flushRx() { if (ser_) while (ser_->available()) ser_->read(); }

    // Envoie `cmd` et renvoie true si une ligne de réponse contient ":0"
    // (":1" = échec explicite). Tolère un éventuel écho de la commande.
    bool sendExpectOk(const char* cmd, uint32_t timeoutMs) {
        if (!ser_) return false;
        ser_->print(cmd);
        uint32_t start = millis();
        String line;
        while (millis() - start < timeoutMs) {
            while (ser_->available()) {
                char c = (char)ser_->read();
                if (c == '\n') {
                    if (line.indexOf(":0") >= 0) return true;
                    if (line.indexOf(":1") >= 0) return false;
                    line = "";
                } else if (c != '\r') {
                    line += c;
                    if (line.length() > 48) line.remove(0, line.length() - 48);
                }
            }
            delay(2);
        }
        return false;
    }

    // Hz -> index CTCSS 0001..0038 (0 si aucune / non trouvée). Table EIA 38 tons.
    static int ctcssIndex(double hz) {
        if (hz < 60.0) return 0;
        static const double kTones[38] = {
            67.0, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8,
            97.4, 100.0, 103.5, 107.2, 110.9, 114.8, 118.8, 123.0, 127.3, 131.8,
            136.5, 141.3, 146.2, 151.4, 156.7, 162.2, 167.9, 173.8, 179.9, 186.2,
            192.8, 203.5, 210.7, 218.1, 225.7, 233.6, 241.8, 250.3
        };
        int best = 0;
        double bestErr = 3.0;                     // tolérance 3 Hz
        for (int i = 0; i < 38; i++) {
            double e = fabs(kTones[i] - hz);
            if (e < bestErr) { bestErr = e; best = i + 1; }
        }
        return best;
    }
};
