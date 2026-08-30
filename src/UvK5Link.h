#pragma once
#include <Arduino.h>
#include <string.h>
#include "config.h"

// ============================================================================
// Couche liaison du protocole série Quansheng UV-K5 / UV-K5 V3 / UV-K1
// (firmware egzumer / F4HWN, cf. App/app/uart.c + App/driver/crc.c de l'amont).
//
// Trame (hôte -> poste) :
//   AB CD                      en-tête (Header.ID = 0xCDAB, little-endian)
//   SS SS                      Header.Size = longueur de la charge interne (LE)
//   <charge interne>           SS octets : [ID:u16 LE][Size:u16 LE][données]
//   CC CC                      CRC-16/XMODEM sur la charge interne (LE)
//   DC BA                      pied
// La charge interne ET le CRC sont masqués (XOR octet à octet avec Obfuscation,
// indice i % 16).
//
// Trame (poste -> hôte) : identique, sauf que les 2 octets à la place du CRC
// sont un simple bourrage (Obfuscation[(Size+n)%16] ^ 0xFF) — pas de CRC en
// réponse (le firmware n'en émet pas).
//
// Aucune session/authentification : le firmware route sur l'ID de commande.
//   0x0514  init de session      -> réponse 0x0515 (chaîne version sur 16 o)
//   0x0601  lecture registre BK4819   (option ENABLE_UART_RW_BK_REGS)
//   0x0602  écriture registre BK4819  (sans réponse)
//   0x06xx  commandes "mode hôte" ajoutées par le fork (voir firmware/uv-k1-k5v3)
// ============================================================================

class UvK5Link {
public:
    // `ser` doit être libre (appeler ser->end() si un autre pilote l'utilisait).
    bool begin(HardwareSerial* ser, int rxPin, int txPin, uint32_t baud = 38400) {
        ser_ = ser;
        ser_->setRxBufferSize(512);
        ser_->begin(baud, SERIAL_8N1, rxPin, txPin);
        ser_->setTimeout(50);
        return true;
    }

    // Envoie 0x0514 et attend une réponse 0x0515 bien formée -> poste présent.
    bool probe(char* fwOut = nullptr, size_t fwCap = 0) {
        uint8_t ts[4];
        uint32_t t = millis();
        ts[0] = t; ts[1] = t >> 8; ts[2] = t >> 16; ts[3] = t >> 24;
        for (int i = 0; i < 3; i++) {
            uint16_t rid = 0;
            uint8_t  rd[48];
            size_t   rl = sizeof(rd);
            if (command(0x0514, ts, 4, rid, rd, rl, 400) && rid == 0x0515) {
                char v[17] = {0};
                memcpy(v, rd, rl < 16 ? rl : 16);
                Serial.printf("[UVK5] poste detecte, firmware \"%s\"\n", v);
                if (fwOut && fwCap) strlcpy(fwOut, v, fwCap);
                return true;
            }
            delay(150);
        }
        return false;
    }

    // Lecture d'un registre BK4819 (0x0601). Utile pour un test de sanité du
    // lien avec un firmware compilé seulement avec ENABLE_UART_RW_BK_REGS.
    bool readBkReg(uint8_t reg, uint16_t& value) {
        uint16_t rid = 0;
        uint8_t  rd[8];
        size_t   rl = sizeof(rd);
        if (!command(0x0601, &reg, 1, rid, rd, rl, 300)) return false;
        if (rid != 0x0601 || rl < 3) return false;
        value = rd[1] | ((uint16_t)rd[2] << 8);   // {reg:u8, value:u16 LE}
        return true;
    }

    // Écriture d'un registre BK4819 (0x0602, sans réponse).
    bool writeBkReg(uint8_t reg, uint16_t value) {
        uint8_t b[3] = { reg, (uint8_t)(value & 0xFF), (uint8_t)(value >> 8) };
        return sendFrame(0x0602, b, 3);
    }

    // --- Coeur du codec ----------------------------------------------------

    // Envoie une commande et lit la réponse.
    //   body / bodyLen : données APRÈS l'en-tête interne (peut être nul).
    //   replyData / replyLen : en entrée = capacité, en sortie = longueur reçue
    //     (données après l'en-tête interne de la réponse).
    // Renvoie false sur timeout ou trame invalide.
    bool command(uint16_t id, const uint8_t* body, size_t bodyLen,
                 uint16_t& replyId, uint8_t* replyData, size_t& replyLen,
                 uint32_t timeoutMs = 300) {
        flushRx();
        if (!sendFrame(id, body, bodyLen)) return false;
        uint8_t inner[256];
        int n = recvFrame(inner, sizeof(inner), timeoutMs);
        if (n < 4) return false;
        replyId = inner[0] | ((uint16_t)inner[1] << 8);
        uint16_t innerSize = inner[2] | ((uint16_t)inner[3] << 8);
        size_t dlen = (size_t)n - 4;
        if (innerSize < dlen) dlen = innerSize;
        if (dlen > replyLen) dlen = replyLen;
        memcpy(replyData, inner + 4, dlen);
        replyLen = dlen;
        return true;
    }

    // Envoie une commande sans attendre de réponse.
    bool sendFrame(uint16_t id, const uint8_t* body, size_t bodyLen) {
        if (!ser_ || bodyLen > 240) return false;
        uint8_t inner[4 + 240 + 2];
        size_t payLen = 4 + bodyLen;                 // en-tête interne + données
        inner[0] = id & 0xFF;      inner[1] = id >> 8;
        inner[2] = bodyLen & 0xFF; inner[3] = bodyLen >> 8;
        if (body && bodyLen) memcpy(inner + 4, body, bodyLen);
        uint16_t crc = crc16Xmodem(inner, payLen);
        inner[payLen]     = crc & 0xFF;
        inner[payLen + 1] = crc >> 8;
        size_t obfLen = payLen + 2;
        for (size_t i = 0; i < obfLen; i++) inner[i] ^= kObf[i & 15];

        uint8_t hdr[4]  = { 0xAB, 0xCD, (uint8_t)(payLen & 0xFF), (uint8_t)(payLen >> 8) };
        uint8_t tail[2] = { 0xDC, 0xBA };
        ser_->write(hdr, 4);
        ser_->write(inner, obfLen);
        ser_->write(tail, 2);
        ser_->flush();
        return true;
    }

    HardwareSerial* raw() { return ser_; }

    static uint16_t crc16Xmodem(const uint8_t* p, size_t n) {
        uint16_t crc = 0;
        for (size_t i = 0; i < n; i++) {
            crc ^= (uint16_t)p[i] << 8;
            for (int b = 0; b < 8; b++)
                crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
        return crc;
    }

private:
    // Lit une trame de réponse. Renvoie la longueur de charge interne
    // désmasquée écrite dans `out` (en-tête interne compris), -1 si timeout.
    int recvFrame(uint8_t* out, size_t outCap, uint32_t timeoutMs) {
        uint32_t start = millis();
        uint8_t  st = 0;             // 0 AB, 1 CD, 2 len0, 3 len1, 4 data
        uint16_t sz = 0, need = 0, got = 0;
        static uint8_t buf[260];
        while (millis() - start < timeoutMs) {
            while (ser_->available()) {
                uint8_t c = (uint8_t)ser_->read();
                switch (st) {
                    case 0: if (c == 0xAB) st = 1; break;
                    case 1: st = (c == 0xCD) ? 2 : 0; break;
                    case 2: sz = c; st = 3; break;
                    case 3:
                        sz |= (uint16_t)c << 8;
                        if (sz < 4 || sz > sizeof(buf) - 4) { st = 0; break; }
                        need = sz + 2 /*bourrage/CRC*/ + 2 /*DC BA*/;
                        got = 0; st = 4;
                        break;
                    case 4:
                        buf[got++] = c;
                        if (got < need) break;
                        st = 0;
                        if (buf[need - 2] != 0xDC || buf[need - 1] != 0xBA) break;
                        {
                            size_t n = sz;
                            if (n > outCap) n = outCap;
                            for (size_t i = 0; i < n; i++) out[i] = buf[i] ^ kObf[i & 15];
                            return (int)n;
                        }
                }
            }
            delay(1);
        }
        return -1;
    }

    void flushRx() { if (ser_) while (ser_->available()) ser_->read(); }

    static const uint8_t kObf[16];
    HardwareSerial* ser_ = nullptr;
};

inline const uint8_t UvK5Link::kObf[16] = {
    0x16, 0x6C, 0x14, 0xE6, 0x2E, 0x91, 0x0D, 0x40,
    0x21, 0x35, 0xD5, 0x40, 0x13, 0x03, 0xE9, 0x80
};
