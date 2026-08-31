#pragma once
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "config.h"
#include "UvK5Link.h"

// ============================================================================
// Pilote "haut niveau" d'un poste Quansheng UV-K1 / UV-K5 V3 tournant le
// firmware "mode hôte" (voir firmware/uv-k1-k5v3/). S'appuie sur UvK5Link pour
// le codec de trame.
//
// Alternative au SA818 : si aucun SA818 n'est détecté, l'ESP tente ce poste sur
// l'UART RF à 38400 bauds. L'audio reste analogique (HP du poste -> ADC ESP,
// DAC ESP -> micro du poste), comme le mode "UV-K1 passerelle".
//
//   uvk5.begin();                      // dans setup(), après avoir libéré l'UART
//   if (uvk5.present()) { ... }
//   uvk5.applyVfo(0, params);          // retune
//   uvk5.ptt(true / false);            // émission
//   uvk5.poll();                       // dans loop() : keepalive + statut
//   const UvK5::Status& s = uvk5.lastStatus();
// ============================================================================

#if RF_MODULE_UVK5_ENABLE

class UvK5 {
public:
    struct RfParams {
        double  rxMHz = 0, txMHz = 0;
        double  rxCtcssHz = 0, txCtcssHz = 0;   // 0 = aucune (DCS pas encore géré)
        bool    wide = true;
        uint8_t power = 7;        // OUTPUT_POWER_* : 7 = HIGH, 6 = MID, 1..5 = LOW
        uint8_t squelch = 0;      // 0 = AF permanent (données) ; 1..9 = phonie
        uint8_t modulation = RF_MODULE_UVK5_MODULATION;   // 0 FM, 1 AM, 2 USB
        bool    flatAudio = false; // true = bypass pré/dé-emphase + HPF/LPF AF +
                                   // compander (AFSK/APRS) ; = bit emph_bypass du canal
    };

    struct Status {
        uint8_t  func = 0;        // 0 FG, 1 TX, 2 MONITOR, 3 INCOMING, 4 RX
        uint16_t rssiRaw = 0;     // registre BK4819 (9 bits)
        int16_t  rssiDbm = 0;
        uint8_t  sMeter = 0;      // 0..15 (mappé comme kv4p-ht)
        bool     sig = false;     // signal reçu présent (squelch + CTCSS OK)
        bool     tx = false;
        uint16_t batterymV = 0;
        uint32_t stamp = 0;       // millis() de la dernière lecture réussie
    };

    // `ser` doit être libre (Serial2.end() d'abord si le SA818 l'utilisait).
    bool begin(HardwareSerial* ser = &Serial2) {
        link_.begin(ser, RF_MODULE_UART_RX, RF_MODULE_UART_TX, RF_MODULE_UVK5_BAUD);
        delay(50);
        char fw[24] = {0};
        for (int i = 0; i < 3 && !present_; i++) {
            if (link_.probe(fw, sizeof(fw))) present_ = true;
            else delay(150);
        }
        if (!present_) return false;
        strlcpy(fw_, fw, sizeof(fw_));
        present_ = enterHost(true);
        Serial.printf("[UVK5] poste %s, mode hote %s\n",
                      fw_, present_ ? "actif" : "REFUSE");
        return present_;
    }

    bool        present()  const { return present_; }
    const char* firmware() const { return fw_; }
    const Status& lastStatus() const { return last_; }

    // 0x0631 SET_VFO
    bool applyVfo(uint8_t vfo, const RfParams& p) {
        uint32_t rxF = hzU(p.rxMHz), txF = hzU(p.txMHz);
        uint8_t rxCT = 0, rxCode = 0, txCT = 0, txCode = 0;
        if (p.rxCtcssHz > 1.0) { rxCT = 1; rxCode = ctcssIdx(p.rxCtcssHz); }
        if (p.txCtcssHz > 1.0) { txCT = 1; txCode = ctcssIdx(p.txCtcssHz); }
        uint8_t b[20];
        b[0] = vfo & 1;
        wr32(b + 1, rxF);
        wr32(b + 5, txF);
        b[9]  = p.modulation;
        b[10] = p.wide ? 0 : 1;          // CHANNEL_BANDWIDTH : 0 large, 1 étroit
        b[11] = p.power;
        b[12] = rxCT; b[13] = rxCode;
        b[14] = txCT; b[15] = txCode;
        b[16] = 0; b[17] = 0;            // step (0 = inchangé)
        b[18] = p.squelch;
        b[19] = p.flatAudio ? 0x01 : 0x00;   // flags b0 : audio plat (firmware >= H15)
        return cmdOk(0x0631, b, sizeof(b));
    }

    // 0x0632 SET_RADIO
    bool setRadio(uint8_t txVfo, uint8_t dualWatch, uint8_t crossBand = 0,
                  uint8_t txLock = 0) {
        uint8_t b[4] = { (uint8_t)(txVfo & 1), dualWatch, crossBand, txLock };
        return cmdOk(0x0632, b, sizeof(b));
    }

    // 0x0633 PTT — envoi SANS attendre l'ack : le keying du PA peut perturber
    // la liaison série du poste (RF / courant), et un blocage de plusieurs
    // centaines de ms dans la boucle Arduino au moment du PTT est risqué.
    // L'état TX réel se lit dans le GET_STATUS suivant (flag bit 1).
    bool ptt(bool on) {
        uint8_t b = on ? 1 : 0;
        return link_.sendFrame(0x0633, &b, 1);
    }

    // 0x0635 MONITOR (force l'AF ouvert / le coupe)
    bool monitor(bool on) {
        uint8_t b = on ? 1 : 0;
        return cmdOk(0x0635, &b, 1);
    }

    // 0x0636 RECALL_CH (rappel d'une mémoire de la radio)
    bool recallChannel(uint8_t vfo, uint16_t ch) {
        uint8_t b[3] = { (uint8_t)(vfo & 1), (uint8_t)(ch & 0xFF), (uint8_t)(ch >> 8) };
        return cmdOk(0x0636, b, sizeof(b));
    }

    // 0x0634 GET_STATUS
    bool getStatus(Status& s) {
        uint16_t rid = 0;
        uint8_t  d[24];
        size_t   n = sizeof(d);
        if (!link_.command(0x0634, nullptr, 0, rid, d, n, 200) || rid != 0x0634 || n < 11)
            return false;
        s.func      = d[0];
        s.rssiRaw   = d[1] | ((uint16_t)d[2] << 8);
        s.rssiDbm   = (int16_t)(d[5] | ((uint16_t)d[6] << 8));
        uint8_t flg = d[10];
        s.tx        = (flg & 2) != 0;
        s.sig       = (flg & 8) != 0;
        s.batterymV = d[8] | ((uint16_t)d[9] << 8);
        // S-mètre 0..15 à partir du RSSI brut (plancher ~ -135 dBm, +6 dB / cran).
        int sm = (s.rssiDbm + 135) / 6;
        if (sm < 0) sm = 0; if (sm > 15) sm = 15;
        s.sMeter    = s.sig ? (uint8_t)(sm < 1 ? 1 : sm) : 0;
        s.stamp     = millis();
        return true;
    }

    // À appeler depuis loop() : keepalive (maintient le watchdog du firmware) +
    // rafraîchit lastStatus(). Perd le poste après plusieurs échecs consécutifs.
    void poll() {
        if (!present_) return;
        uint32_t now = millis();
        if (now - lastPollMs_ < (uint32_t)RF_MODULE_UVK5_POLL_MS) return;
        lastPollMs_ = now;
        Status s;
        if (getStatus(s)) { last_ = s; fails_ = 0; }
        else if (++fails_ >= 40) {    // ~10 s d'échecs (tolère la RF pendant un TX voisin)
            present_ = false;
            Serial.println("[UVK5] poste perdu (pas de reponse GET_STATUS)");
        }
    }

    // Sortie propre du mode hôte (rend la radio autonome).
    void end() { if (present_) enterHost(false); }

private:
    bool enterHost(bool on) {
        uint8_t b = on ? 1 : 0;
        uint16_t rid = 0; uint8_t d[24]; size_t n = sizeof(d);
        if (!link_.command(0x0630, &b, 1, rid, d, n, 500) || rid != 0x0630 || n < 2)
            return false;
        if (n >= 18) Serial.printf("[UVK5] proto H%u\n", d[1]);
        return on ? (d[0] != 0) : true;
    }

    bool cmdOk(uint16_t id, const uint8_t* body, size_t n) {
        uint16_t rid = 0; uint8_t d[8]; size_t rn = sizeof(d);
        if (!link_.command(id, body, n, rid, d, rn, 200)) return false;
        return rid == id && rn >= 1 && d[0] != 0;
    }

    static void wr32(uint8_t* p, uint32_t v) {
        p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
    }
    static uint32_t hzU(double mhz) {
        return (mhz > 1.0) ? (uint32_t)llround(mhz * 1e5) : 0;   // unités de 10 Hz
    }
    // Hz -> index dans CTCSS_Options du firmware (0-indexé). 0 si non trouvé.
    static uint8_t ctcssIdx(double hz) {
        static const uint16_t t[50] = {
            670, 693, 719, 744, 770, 797, 825, 854, 885, 915,
            948, 974, 1000, 1035, 1072, 1109, 1148, 1188, 1230, 1273,
            1318, 1365, 1413, 1462, 1514, 1567, 1598, 1622, 1655, 1679,
            1713, 1738, 1773, 1799, 1835, 1862, 1899, 1928, 1966, 1995,
            2035, 2065, 2107, 2181, 2257, 2291, 2336, 2418, 2503, 2541
        };
        uint16_t v = (uint16_t)lround(hz * 10.0);
        uint8_t best = 0; uint16_t err = 0xFFFF;
        for (uint8_t i = 0; i < 50; i++) {
            uint16_t e = (t[i] > v) ? (t[i] - v) : (v - t[i]);
            if (e < err) { err = e; best = i; }
        }
        return best;
    }

    UvK5Link link_;
    Status   last_;
    char     fw_[24] = {0};
    bool     present_ = false;
    uint8_t  fails_ = 0;
    uint32_t lastPollMs_ = 0;
};

#endif  // RF_MODULE_UVK5_ENABLE
