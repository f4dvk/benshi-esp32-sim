#pragma once
#include <Arduino.h>
#include <BluetoothSerial.h>
#include "config.h"

// ============================================================================
// Solution de repli à UN SEUL canal RFCOMM (BluetoothSerial standard),
// utilisée seulement si USE_DUAL_RFCOMM_SERVERS est à false dans config.h.
// Ne respecte pas la séparation "commande / audio sur canaux distincts"
// documentée dans le protocole, mais reste utile pour valider rapidement
// qu'un canal RFCOMM audio fonctionne, avant de passer à DualRfcommServers.h.
// ============================================================================

// ============================================================================
// Solution de repli à UN SEUL canal RFCOMM (BluetoothSerial standard),
// utilisée seulement si USE_DUAL_RFCOMM_SERVERS est à false dans config.h.
// Ne respecte pas la séparation "commande / audio sur canaux distincts"
// documentée dans le protocole (tout est multiplexé sur le même canal),
// mais reste utile pour valider rapidement qu'un canal RFCOMM audio
// fonctionne, avant de passer à DualRfcommServers.h.
//
// Framing audio (doc protocole) :
//   - chaque message est encadré par 0x7E ... 0x7E
//   - à l'intérieur, tout octet 0x7E ou 0x7D est "byte-stuffé" :
//       0x7E -> 0x7D 0x5E
//       0x7D -> 0x7D 0x5D
//   - premier octet après désescaping = Type (0x00 AudioData / 0x01 AudioEnd
//     / 0x02 AudioAck), le reste = payload SBC (ou padding).
//
// NOTE IMPORTANTE SUR LE CODEC :
// Le protocole encode l'audio en SBC (Sub-band Codec, 32 kHz), le même codec
// que l'A2DP Bluetooth. Ecrire un codec SBC complet ne rentre pas dans ce
// squelette : utilise une lib existante, par exemple la lib SBC embarquée
// dans "pschatzmann/ESP32-A2DP" (elle expose oi_codec_sbc en C, réutilisable
// même hors A2DP), ou la lib "arduino-libsbc" si tu la trouves packagée pour
// PlatformIO. Branche ton encodeur/décodeur dans encodeAudioFrame()/
// onAudioDataReceived() ci-dessous (marqués TODO).
// ============================================================================

class BenshiAudioLink {
public:
    void begin() {
        // Si un appairage d'une VRAIE radio existe déjà côté PC/téléphone
        // sous ce même nom, supprime-le pour éviter un verrou nom/MAC en cache.
        bt_.begin(BT_CLASSIC_NAME);
        Serial.printf("[SPP] RFCOMM audio prêt sous le nom '%s'\n", BT_CLASSIC_NAME);
    }

    void loop() {
        while (bt_.available()) {
            uint8_t b = bt_.read();
            feedRxByte(b);
        }
    }

    // Appelé par ton code applicatif (boucle I2S mic) avec un paquet SBC
    // déjà encodé (TODO: brancher un vrai encodeur SBC ici).
    void sendAudioData(const uint8_t* sbcPayload, size_t len) {
        std::vector<uint8_t> frame;
        frame.push_back(0x00); // Type = AudioData
        frame.insert(frame.end(), sbcPayload, sbcPayload + len);
        writeFramed(frame);
    }

    void sendAudioEnd() {
        std::vector<uint8_t> frame = { 0x01 };
        writeFramed(frame);
    }

private:
    // ---- Emission : escaping + délimiteurs 0x7E ----------------------------
    void writeFramed(const std::vector<uint8_t>& payload) {
        bt_.write(0x7E);
        for (uint8_t b : payload) {
            if (b == 0x7E) { bt_.write(0x7D); bt_.write(0x5E); }
            else if (b == 0x7D) { bt_.write(0x7D); bt_.write(0x5D); }
            else bt_.write(b);
        }
        bt_.write(0x7E);
    }

    // ---- Réception : désescaping + reconstruction de trame -----------------
    void feedRxByte(uint8_t b) {
        if (b == 0x7E) {
            if (!rxBuf_.empty()) {
                dispatchFrame(rxBuf_);
            }
            rxBuf_.clear();
            escapeNext_ = false;
            return;
        }
        if (b == 0x7D) { escapeNext_ = true; return; }
        if (escapeNext_) { b ^= 0x20; escapeNext_ = false; }
        rxBuf_.push_back(b);
    }

    void dispatchFrame(const std::vector<uint8_t>& frame) {
        if (frame.empty()) return;
        uint8_t type = frame[0];
        const uint8_t* data = frame.data() + 1;
        size_t len = frame.size() - 1;

        switch (type) {
            case 0x00: // AudioData (SBC) reçu depuis HTCommander (micro app / PTT)
                onAudioDataReceived(data, len);
                break;
            case 0x01: // AudioEnd
                onAudioEndReceived();
                break;
            case 0x02: // AudioAck
                onAudioAckReceived();
                break;
            default:
                Serial.printf("[SPP] Type de trame audio inconnu: 0x%02X\n", type);
        }
    }

    // TODO : décoder le SBC ici et pousser les PCM vers ta sortie I2S
    // (haut-parleur simulant la réception radio).
    void onAudioDataReceived(const uint8_t* sbc, size_t len) {
        Serial.printf("[SPP] AudioData reçu (%u octets SBC)\n", (unsigned)len);
        // decodeSBCtoI2S(sbc, len);
    }

    void onAudioEndReceived() {
        Serial.println("[SPP] Fin de transmission audio (AudioEnd)");
    }

    void onAudioAckReceived() {
        Serial.println("[SPP] AudioAck reçu");
    }

    BluetoothSerial bt_;
    std::vector<uint8_t> rxBuf_;
    bool escapeNext_ = false;
};
