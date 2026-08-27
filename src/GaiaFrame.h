#pragma once
#include <Arduino.h>
#include <vector>

// ============================================================================
// GaiaFrame : encapsulation des BenshiMessage quand ils transitent par
// RFCOMM (Bluetooth Classic) plutôt que par BLE.
//
//   Octet 0 : Start of Frame = 0xFF
//   Octet 1 : Version = 0x01
//   Octet 2 : Flags (bit0 = checksum présent)
//   Octet 3 : Payload Length = taille du BODY du message (donc EXCLUT les
//             4 octets d'en-tête command_group/is_reply/command)
//   Octets 4..: Data = les 4 octets d'en-tête + le body du message
//               (donc taille réelle = 4 + Payload Length)
//   Dernier octet (optionnel) : checksum 8 bits, présent seulement si le
//   bit0 de Flags est set.
// ============================================================================

namespace GaiaFrame {

    inline uint8_t checksum8(const uint8_t* data, size_t len) {
        uint8_t sum = 0;
        for (size_t i = 0; i < len; i++) sum ^= data[i]; // XOR simple (non documenté précisément -> à ajuster si besoin, voir README)
        return sum;
    }

    // encodedMessage = sortie de encodeMessage() (4 octets d'en-tête + body)
    inline std::vector<uint8_t> encode(const std::vector<uint8_t>& encodedMessage, bool withChecksum = false) {
        std::vector<uint8_t> out;
        uint8_t bodyLen = encodedMessage.size() >= 4 ? (encodedMessage.size() - 4) : 0;

        out.push_back(0xFF);
        out.push_back(0x01);
        out.push_back(withChecksum ? 0x01 : 0x00);
        out.push_back(bodyLen);
        out.insert(out.end(), encodedMessage.begin(), encodedMessage.end());

        if (withChecksum) {
            out.push_back(checksum8(out.data(), out.size()));
        }
        return out;
    }

    // Tente de décoder une trame GaiaFrame complète depuis un buffer accumulé.
    // Retourne le nombre d'octets consommés (0 si trame incomplète, -1 si
    // erreur de format -> l'appelant doit alors purger le buffer).
    // encodedMessageOut reçoit les 4+N octets du BenshiMessage encodé.
    inline int tryDecode(const uint8_t* buf, size_t len, std::vector<uint8_t>& encodedMessageOut) {
        if (len < 4) return 0; // pas assez pour lire l'en-tête GaiaFrame
        if (buf[0] != 0xFF) return -1;
        if (buf[1] != 0x01) return -1;

        bool hasChecksum = (buf[2] & 0x01) != 0;
        uint8_t bodyLen = buf[3];
        size_t dataLen = 4 + bodyLen; // 4 octets d'en-tête Message + body
        size_t frameLen = 4 /*header gaia*/ + dataLen + (hasChecksum ? 1 : 0);

        if (len < frameLen) return 0; // encore incomplet

        encodedMessageOut.assign(buf + 4, buf + 4 + dataLen);
        return (int)frameLen;
    }

} // namespace GaiaFrame
