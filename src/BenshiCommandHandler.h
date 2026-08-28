#pragma once
#include <functional>
#include <atomic>
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
// EVENT_NOTIFICATION (HT_STATUS_CHANGED) : la vraie VR-N76 pousse ces trames
// dès qu'un paramètre change ET pendant la réception (squelch/RSSI). On les
// reproduit : HTCommander s'y abonne via REGISTER_NOTIFICATION juste après
// GET_DEV_INFO, et son indicateur RX / S-mètre en dépend.
// ============================================================================

class BenshiCommandHandler {
public:
    // Envoi d'un message NON sollicité sur le canal commande (branché par le
    // transport). Peut être appelé depuis n'importe quelle tâche.
    using NotifSink = std::function<void(const BenshiMessage&)>;

    void begin() { state_.begin(); }
    void onNotify(NotifSink sink) { sink_ = std::move(sink); }

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
            for (uint8_t t : in.body) {
                if (t < 16) registeredMask_ |= (uint16_t)(1u << t);
            }
            Serial.printf("[CMD] REGISTER_NOTIFICATION : masque = 0x%04X\n", registeredMask_);
            outMsg.body = BenshiReplies::registerNotifAck();

        } else if (in.command == READ_SETTINGS) {
            outMsg.body = BenshiReplies::settings(state_);

        } else if (in.command == WRITE_SETTINGS) {
            bool ok = state_.setSettingsStruct(in.body.data(), in.body.size());
            outMsg.body = BenshiReplies::writeSettingsAck(
                ok ? ReplyStatus::SUCCESS : ReplyStatus::INVALID_PARAMETER);
            if (ok) htStatusDirty_ = true;

        } else if (in.command == READ_RF_CH) {
            uint8_t channelId = in.body.empty() ? 0 : in.body[0];
            uint8_t status;
            outMsg.body = BenshiReplies::rfChannel(state_, channelId, status);

        } else if (in.command == WRITE_RF_CH) {
            uint8_t channelId = in.body.empty() ? 0 : in.body[0];
            bool ok = state_.setChannelStruct(channelId, in.body.data(), in.body.size());
            outMsg.body = BenshiReplies::writeRfChAck(
                ok ? ReplyStatus::SUCCESS : ReplyStatus::INVALID_PARAMETER, channelId);
            if (ok) htStatusDirty_ = true;

        } else if (in.command == SET_REGION) {
            uint8_t regionId = in.body.empty() ? 0 : in.body[0];
            state_.setRegion(regionId);
            outMsg.body = BenshiReplies::regionAck(ReplyStatus::SUCCESS, regionId);
            htStatusDirty_ = true;

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
            outMsg.body = BenshiReplies::htStatus(state_, inTx_.load(), sqOpen_.load(),
                                                  rssi_.load(), aocConnected_.load());

        } else if (in.command == SET_PHONE_STATUS) {
            outMsg.body = BenshiReplies::phoneStatusAck();

        } else {
            Serial.printf("[CMD] Commande non gérée: group=%u cmd=%u\n",
                          in.command_group, in.command);
            outMsg.body = BenshiReplies::notSupported();
        }
        return true;
    }

    // À appeler par le transport APRÈS avoir envoyé la réponse à la commande
    // courante : émet la notification de statut si une écriture l'a salie.
    void flushPendingNotifications() {
        if (htStatusDirty_) {
            htStatusDirty_ = false;
            emitHtStatusChanged();
        }
    }

    // --- Hooks du pont audio (contexte tâche) ---------------------------------
    // Réception depuis le poste : squelch/RX ouverts + RSSI 0..15 dérivé de l'ADC.
    void setAudioRx(bool active, uint8_t rssi) {
        bool changed = (active != sqOpen_.load()) || (rssi != rssi_.load());
        sqOpen_.store(active);
        rssi_.store(active ? rssi : 0);
        if (changed) emitHtStatusChanged();
    }
    // Émission vers le poste (HTCommander envoie de l'audio) : is_in_tx.
    void setAudioTx(bool tx) {
        if (tx == inTx_.load()) return;
        inTx_.store(tx);
        emitHtStatusChanged();
    }
    // Canal audio RFCOMM connecté / déconnecté.
    void setAudioConnected(bool c) {
        if (c == aocConnected_.load()) return;
        aocConnected_.store(c);
        emitHtStatusChanged();
    }

    void emitHtStatusChanged() {
        if (!sink_) return;
        if (!(registeredMask_ & (1u << EventType::HT_STATUS_CHANGED))) return;
        BenshiMessage m;
        m.command_group = CommandGroup::BASIC;
        m.is_reply      = false;
        m.command       = BasicCommand::EVENT_NOTIFICATION;
        m.body = BenshiReplies::htStatusChangedEvent(state_, inTx_.load(), sqOpen_.load(),
                                                     rssi_.load(), aocConnected_.load());
        sink_(m);
    }

private:
    RadioState state_;
    NotifSink  sink_;

    uint16_t registeredMask_ = 0;   // bit t = type de notification t enregistré
    bool     htStatusDirty_  = false;

    std::atomic<bool>    sqOpen_{false};
    std::atomic<uint8_t> rssi_{0};
    std::atomic<bool>    aocConnected_{false};
    std::atomic<bool>    inTx_{false};
};
