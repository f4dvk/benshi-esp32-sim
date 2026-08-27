#pragma once
#include "BenshiProtocol.h"
#include "RadioState.h"
#include "config.h"

// ============================================================================
// Logique métier : quelle commande -> quelle réponse, en s'appuyant sur
// l'état RUNTIME (RadioState). Les commandes d'ÉCRITURE modifient cet état :
//
//   WRITE_RF_CH       éditer un canal mémoire (fréquences, CTCSS, nom, BW,
//                     TX interdit...)
//   WRITE_SETTINGS    changer le canal VFO A / VFO B actif, le mode double
//                     veille (VFO sélectionné), le squelch, le scan
//   SET_REGION        changer la région active
//   WRITE_REGION_NAME renommer une région
//
// Toutes ces modifications sont persistées en NVS (voir RadioState.h).
// HTCommander relit systématiquement après une écriture, donc l'UI se met à
// jour sans qu'on ait besoin d'émettre d'EVENT_NOTIFICATION.
// ============================================================================

class BenshiCommandHandler {
public:
    void begin() { state_.begin(); }

    // Traite un message entrant et renvoie true si une réponse doit être
    // envoyée (outMsg rempli).
    bool process(const BenshiMessage& in, BenshiMessage& outMsg) {
        using namespace BasicCommand;

        outMsg.command_group = in.command_group;
        outMsg.is_reply = true;
        outMsg.command = in.command;

        if (in.command_group != CommandGroup::BASIC) {
            outMsg.body = BenshiReplies::notSupported();
            return true;
        }

        if (in.command == GET_DEV_INFO) {
            outMsg.body = BenshiReplies::devInfo();

        } else if (in.command == READ_STATUS) {
            uint16_t statusType = in.body.size() >= 2
                                      ? (uint16_t)((in.body[0] << 8) | in.body[1])
                                      : 4;
            outMsg.body = BenshiReplies::readStatus(statusType);

        } else if (in.command == REGISTER_NOTIFICATION) {
            outMsg.body = BenshiReplies::registerNotifAck();

        } else if (in.command == READ_SETTINGS) {
            outMsg.body = BenshiReplies::settings(state_);

        } else if (in.command == WRITE_SETTINGS) {
            bool ok = state_.setSettingsStruct(in.body.data(), in.body.size());
            outMsg.body = BenshiReplies::writeSettingsAck(
                ok ? ReplyStatus::SUCCESS : ReplyStatus::INVALID_PARAMETER);

        } else if (in.command == READ_RF_CH) {
            uint8_t channelId = in.body.empty() ? 0 : in.body[0];
            uint8_t status;
            outMsg.body = BenshiReplies::rfChannel(state_, channelId, status);

        } else if (in.command == WRITE_RF_CH) {
            uint8_t channelId = in.body.empty() ? 0 : in.body[0];
            bool ok = state_.setChannelStruct(channelId, in.body.data(), in.body.size());
            outMsg.body = BenshiReplies::writeRfChAck(
                ok ? ReplyStatus::SUCCESS : ReplyStatus::INVALID_PARAMETER, channelId);

        } else if (in.command == SET_REGION) {
            uint8_t regionId = in.body.empty() ? 0 : in.body[0];
            state_.setRegion(regionId);
            outMsg.body = BenshiReplies::regionAck(ReplyStatus::SUCCESS, regionId);

        } else if (in.command == WRITE_REGION_NAME) {
            if (in.body.empty()) {
                outMsg.body = { ReplyStatus::INVALID_PARAMETER };
            } else {
                uint8_t regionId = in.body[0];
                char name[11] = {0};
                for (size_t i = 1; i < in.body.size() && i <= 10; i++) {
                    name[i - 1] = (char)in.body[i];
                }
                state_.setRegionName(regionId, String(name));
                outMsg.body = BenshiReplies::regionAck(ReplyStatus::SUCCESS, regionId);
            }

        } else if (in.command == READ_REGION_NAME) {
            uint8_t regionId = in.body.empty() ? 0 : in.body[0];
            outMsg.body = BenshiReplies::regionName(state_, regionId);

        } else if (in.command == GET_HT_STATUS) {
            outMsg.body = BenshiReplies::htStatus(state_, false, false);

        } else if (in.command == SET_PHONE_STATUS) {
            outMsg.body = BenshiReplies::phoneStatusAck();

        } else {
            Serial.printf("[CMD] Commande non gérée: group=%u cmd=%u\n",
                          in.command_group, in.command);
            outMsg.body = BenshiReplies::notSupported();
        }
        return true;
    }

private:
    RadioState state_;
};
