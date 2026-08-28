#pragma once
#include <Arduino.h>
#include <vector>
#include "BitStream.h"
#include "config.h"
#include "RadioState.h"

// ============================================================================
// Message Benshi : en-tête de 4 octets, aligné sur l'octet (16+1+15 = 32 bits)
//   command_group : 16 bits
//   is_reply      : 1 bit
//   command       : 15 bits
//   body          : le reste
// ============================================================================

namespace CommandGroup {
    static const uint16_t BASIC    = 2;
    static const uint16_t EXTENDED = 10;
}

namespace BasicCommand {
    static const uint16_t GET_DEV_INFO           = 4;
    static const uint16_t READ_STATUS            = 5;
    static const uint16_t REGISTER_NOTIFICATION   = 6;
    static const uint16_t EVENT_NOTIFICATION      = 9;
    static const uint16_t READ_SETTINGS           = 10;
    static const uint16_t WRITE_SETTINGS          = 11;
    static const uint16_t READ_RF_CH              = 13;
    static const uint16_t WRITE_RF_CH             = 14;
    static const uint16_t GET_HT_STATUS           = 20;
    static const uint16_t GET_VOLUME              = 22;
    static const uint16_t SET_VOLUME              = 23;
    static const uint16_t HT_SEND_DATA            = 31;
    static const uint16_t READ_BSS_SETTINGS       = 33;
    static const uint16_t WRITE_BSS_SETTINGS      = 34;
    static const uint16_t SET_PHONE_STATUS        = 51;
    static const uint16_t WRITE_REGION_NAME       = 59;
    static const uint16_t SET_REGION              = 60;
    static const uint16_t READ_REGION_NAME        = 73;
    static const uint16_t GET_POSITION            = 76;
}

namespace ReplyStatus {
    static const uint8_t SUCCESS            = 0;
    static const uint8_t NOT_SUPPORTED      = 1;
    static const uint8_t INVALID_PARAMETER  = 5;
    static const uint8_t INCORRECT_STATE    = 6;
}

namespace EventType {
    static const uint8_t HT_STATUS_CHANGED   = 1;
    static const uint8_t DATA_RXD            = 2;
    static const uint8_t HT_CH_CHANGED       = 5;
    static const uint8_t HT_SETTINGS_CHANGED = 6;
}

struct BenshiMessage {
    uint16_t command_group;
    bool     is_reply;
    uint16_t command;
    std::vector<uint8_t> body;
};

// ---- (dé)sérialisation de l'en-tête 4 octets --------------------------
inline std::vector<uint8_t> encodeMessage(const BenshiMessage& msg) {
    std::vector<uint8_t> out;
    out.push_back((msg.command_group >> 8) & 0xFF);
    out.push_back(msg.command_group & 0xFF);
    uint8_t b2 = (msg.is_reply ? 0x80 : 0x00) | ((msg.command >> 8) & 0x7F);
    out.push_back(b2);
    out.push_back(msg.command & 0xFF);
    out.insert(out.end(), msg.body.begin(), msg.body.end());
    return out;
}

inline bool decodeMessage(const uint8_t* data, size_t len, BenshiMessage& out) {
    if (len < 4) return false;
    out.command_group = (data[0] << 8) | data[1];
    out.is_reply = (data[2] & 0x80) != 0;
    out.command  = ((data[2] & 0x7F) << 8) | data[3];
    out.body.assign(data + 4, data + len);
    return true;
}

// ============================================================================
// Construction des réponses. DEV_INFO et READ_STATUS viennent de config.h ;
// canaux / réglages / région viennent de l'état RUNTIME (RadioState), qui est
// initialisé depuis config.h puis modifiable par HTCommander.
// ============================================================================
namespace BenshiReplies {

    // ---- GET_DEV_INFO -----------------------------------------------------
    // Disposition EXACTE attendue par HTCommander (RadioDevInfo.fromBytes,
    // src/lib/radio/radio_models.dart). msg[] = [grp(2)][cmd(2)][status][body].
    //   msg[5]      vendor_id
    //   msg[6..7]   product_id (big-endian)
    //   msg[8]      hw_ver
    //   msg[9..10]  soft_ver (big-endian)
    //   msg[11]     bit7 support_radio, bit6 support_medium_power,
    //               bit5 fixed_loc_speaker_vol, bit4 not_support_soft_power_ctrl,
    //               bit3 have_no_speaker, bit2 have_hm_speaker,
    //               bits[1:0] = region_count[5:4]
    //   msg[12]     bits[7:4] = region_count[3:0], bit3 support_noaa,
    //               bit2 gmrs, bit1 support_vfo, bit0 support_dmr
    //   msg[13]     channel_count   <-- HTCommander lit le nombre de canaux ICI
    //   msg[14]     bits[7:4] = freq_range_count
    // L'ancienne version omettait l'octet msg[12] -> channel_count se
    // retrouvait en msg[12] et freq_range_count (0x10) etait lu comme
    // channel_count = 16, d'ou les lectures readRfCh[8..15] en echec.
    inline std::vector<uint8_t> devInfo() {
        BitWriter w;
        w.writeBits(ReplyStatus::SUCCESS, 8);
        w.writeBits(DEV_INFO.vendor_id, 8);          // msg[5]
        w.writeBits(DEV_INFO.product_id, 16);        // msg[6..7]
        w.writeBits(DEV_INFO.hw_ver, 8);             // msg[8]
        w.writeBits(DEV_INFO.soft_ver, 16);          // msg[9..10]

        // msg[11]
        w.writeBits(DEV_INFO.support_radio ? 1 : 0, 1);
        w.writeBits(DEV_INFO.support_medium_pw ? 1 : 0, 1);
        w.writeBits(0, 1);  // fixed_loc_speaker_vol
        w.writeBits(0, 1);  // not_support_soft_power_ctrl
        w.writeBits(0, 1);  // have_no_speaker
        w.writeBits(0, 1);  // have_hm_speaker
        w.writeBits((DEV_INFO.region_count >> 4) & 0x03, 2); // region_count[5:4]

        // msg[12]
        w.writeBits(DEV_INFO.region_count & 0x0F, 4);        // region_count[3:0]
        w.writeBits(0, 1);  // support_noaa
        w.writeBits(0, 1);  // gmrs
        w.writeBits(0, 1);  // support_vfo
        w.writeBits(DEV_INFO.support_dmr ? 1 : 0, 1);

        w.writeBits(DEV_INFO.channel_count, 8);              // msg[13]

        w.writeBits(DEV_INFO.freq_range_count & 0x0F, 4);    // msg[14] bits[7:4]
        w.writeBits(0, 4);                                   // padding octet plein
        return w.bytes();
    }

    // ---- READ_STATUS (alimentation) --------------------------------------
    // Requête HTCommander : body = type sur 16 bits big-endian
    //   1 = battery_level (0..15), 2 = battery_voltage (V x100, 16 bits),
    //   3 = rc_battery_level, 4 = battery_level_as_percentage (0..100).
    // Réponse : [status][type(16b BE)][valeur]. On ECHO le type demandé,
    // sinon HTCommander ne sait pas à quoi rattacher la valeur.
    inline std::vector<uint8_t> readStatus(uint16_t type) {
        std::vector<uint8_t> b;
        b.push_back(ReplyStatus::SUCCESS);
        b.push_back((type >> 8) & 0xFF);
        b.push_back(type & 0xFF);
        switch (type) {
            case 2: // battery_voltage, V x100 sur 16 bits -> 8.20 V
                b.push_back(0x03); // 0x0334 = 820
                b.push_back(0x34);
                break;
            case 1: // battery_level (echelle interne, ~0..15)
            case 3: // rc_battery_level
                b.push_back(15);
                break;
            case 4: // pourcentage
            default:
                b.push_back(100);
                break;
        }
        return b;
    }

    // ---- READ_RF_CH ---------------------------------------------------------
    // Disposition (struct de 25 octets APRES l'octet status), voir
    // RadioState::encodeChannelStruct et RadioChannelInfo.fromBytes/toByteArray
    // de HTCommander :
    //   [0]      channel_id
    //   [1..4]   tx : bits[31:30]=tx_mod, bits[29:0]=tx_freq (Hz), big-endian
    //   [5..8]   rx : idem
    //   [9..10]  tx_sub_audio (16 bits, CTCSS Hz x100 ; 0 = aucune)
    //   [11..12] rx_sub_audio
    //   [13]     bit7 scan, bit6 tx_at_max_power, bit5 talk_around,
    //            bit4 bandwidth (1=wide), bit3 pre_de_emph_bypass,
    //            bit2 sign, bit1 tx_at_med_power, bit0 tx_disable
    //   [14]     bit7 fixed_freq, bit6 fixed_bandwidth, bit5 fixed_tx_power,
    //            bit4 mute
    //   [15..24] nom (10 octets UTF-8)
    inline std::vector<uint8_t> rfChannel(const RadioState& st, uint8_t channelId,
                                          uint8_t& statusOut) {
        if (channelId >= CHANNEL_COUNT) {
            // On renvoie quand meme l'id demande en 2e octet : HTCommander
            // fait avancer sa file de lecture d'apres cet id (fromBytes lit
            // msg[5]), sinon la lecture reste bloquee sur un timeout.
            statusOut = ReplyStatus::INVALID_PARAMETER;
            return { ReplyStatus::INVALID_PARAMETER, channelId };
        }
        std::vector<uint8_t> b;
        b.reserve(1 + RadioState::CH_STRUCT_LEN);
        b.push_back(ReplyStatus::SUCCESS);
        const std::vector<uint8_t>& s = st.channelStruct(channelId);
        b.insert(b.end(), s.begin(), s.end());
        statusOut = ReplyStatus::SUCCESS;
        return b;
    }

    // ---- WRITE_RF_CH (accusé de réception) -----------------------------
    inline std::vector<uint8_t> writeRfChAck(uint8_t status, uint8_t channelId) {
        BitWriter w;
        w.writeBits(status, 8);
        w.writeBits(channelId, 8);
        return w.bytes();
    }

    // ---- READ_SETTINGS ---------------------------------------------------
    // Disposition attendue par HTCommander (RadioSettings.fromBytes). Corps de
    // 20 octets APRES le status (msg[5..24]) :
    //   msg[5]  bits[7:4] = channel_a[3:0], bits[3:0] = channel_b[3:0]
    //   msg[6]  bit7 scan, bit6 aghfp_call_mode, bits[5:4] double_channel,
    //           bits[3:0] squelch_level
    //   msg[7]  ... bits[3:1] = mic_gain ...
    //   msg[14] bits[7:4] = channel_a[7:4], bits[3:0] = channel_b[7:4]
    // On renvoie l'etat courant (modifiable via WRITE_SETTINGS).
    inline std::vector<uint8_t> settings(const RadioState& st) {
        std::vector<uint8_t> b;
        b.reserve(1 + RadioState::SETTINGS_STRUCT_LEN);
        b.push_back(ReplyStatus::SUCCESS);
        const std::vector<uint8_t>& s = st.settingsStruct();
        b.insert(b.end(), s.begin(), s.end());
        return b;
    }

    // ---- WRITE_SETTINGS (accusé) ---------------------------------------
    inline std::vector<uint8_t> writeSettingsAck(uint8_t status) {
        return { status };
    }

    // ---- GET_HT_STATUS (StatusExt) --------------------------------------
    // Disposition attendue par HTCommander (RadioHtStatus.fromBytes). Corps de
    // 4 octets APRES le status :
    //   msg[5] bit7 is_power_on, bit6 is_in_tx, bit5 is_sq, bit4 is_in_rx,
    //          bits[3:2] double_channel, bit1 is_scan, bit0 is_radio
    //   msg[6] bits[7:4] curr_ch_id[3:0], bit3 is_gps_locked,
    //          bit2 is_hfp_connected, bit1 is_aoc_connected
    //   msg[7] bits[7:4] rssi, bits[3:0] curr_region[5:2]
    //   msg[8] bits[7:6] curr_region[1:0], bits[5:2] curr_ch_id[7:4]
    // Remplit les 4 octets de StatusExt dans out[0..3] (voir la disposition
    // ci-dessus). `rssi` : 0..15. `aocConn` : canal audio RFCOMM connecte.
    inline void packHtStatus(const RadioState& st, bool inTx, bool sqOpen,
                             uint8_t rssi, bool aocConn, uint8_t out[4]) {
        const uint8_t region = st.region() & 0x3F;
        const uint8_t chId   = st.activeChannelId();
        const uint8_t dualCh = st.doubleChannel() & 0x03;
        out[0] = (1u << 7)                        // is_power_on
               | ((inTx ? 1u : 0u) << 6)
               | ((sqOpen ? 1u : 0u) << 5)        // is_sq
               | ((sqOpen ? 1u : 0u) << 4)        // is_in_rx
               | (dualCh << 2)                    // double_channel (VFO actif)
               | ((st.scan() ? 1u : 0u) << 1);    // is_scan
        out[1] = ((chId & 0x0F) << 4)             // curr_ch_id[3:0]
               | ((aocConn ? 1u : 0u) << 1);      // is_aoc_connected
        out[2] = ((rssi & 0x0F) << 4)             // rssi
               | ((region >> 2) & 0x0F);          // curr_region[5:2]
        out[3] = ((region & 0x03) << 6)           // curr_region[1:0]
               | (((chId >> 4) & 0x0F) << 2);     // curr_ch_id[7:4]
    }

    // GET_HT_STATUS : [status][4 octets StatusExt].
    inline std::vector<uint8_t> htStatus(const RadioState& st, bool inTx, bool sqOpen,
                                         uint8_t rssi = 0, bool aocConn = false) {
        std::vector<uint8_t> b(1 + 4, 0);
        b[0] = ReplyStatus::SUCCESS;
        packHtStatus(st, inTx, sqOpen, rssi, aocConn, &b[1]);
        return b;
    }

    // EVENT_NOTIFICATION / HT_STATUS_CHANGED : [type=1][4 octets StatusExt].
    // Pas d'octet de statut : le corps colle a ce qu'emet la vraie VR-N76
    // (FF 01 00 05 00 02 00 09 01 XX XX XX XX).
    inline std::vector<uint8_t> htStatusChangedEvent(const RadioState& st, bool inTx,
                                                     bool sqOpen, uint8_t rssi, bool aocConn) {
        std::vector<uint8_t> b(1 + 4, 0);
        b[0] = EventType::HT_STATUS_CHANGED;
        packHtStatus(st, inTx, sqOpen, rssi, aocConn, &b[1]);
        return b;
    }

    // ---- REGISTER_NOTIFICATION (accusé) --------------------------------
    inline std::vector<uint8_t> registerNotifAck() {
        return { ReplyStatus::SUCCESS };
    }

    // ---- READ_REGION_NAME ----------------------------------------------
    // Reponse attendue par HTCommander (_handleReadRegionName) :
    //   body[0] status, body[1] region_id, body[2..11] nom UTF-8 (10 octets).
    inline std::vector<uint8_t> regionName(const RadioState& st, uint8_t regionId) {
        std::vector<uint8_t> b(2 + 10, 0);
        b[0] = ReplyStatus::SUCCESS;
        b[1] = regionId;
        String name = st.regionName(regionId);
        for (uint8_t i = 0; i < 10 && i < name.length(); i++) b[2 + i] = (uint8_t)name[i];
        return b;
    }

    // ---- WRITE_REGION_NAME / SET_REGION (accuse [status][region_id]) ----
    inline std::vector<uint8_t> regionAck(uint8_t status, uint8_t regionId) {
        return { status, regionId };
    }

    // ---- SET_PHONE_STATUS (accusé) --------------------------------------
    inline std::vector<uint8_t> phoneStatusAck() {
        return { ReplyStatus::SUCCESS };
    }

    // ---- Générique "non supporté" ---------------------------------------
    inline std::vector<uint8_t> notSupported() {
        return { ReplyStatus::NOT_SUPPORTED };
    }

} // namespace BenshiReplies
