#pragma once
#include <functional>
#include <atomic>
#include "BenshiProtocol.h"
#include "RadioState.h"
#include "AprsConfig.h"
#include "Sa818.h"
#include "config.h"
#if RF_MODULE_UVK5_ENABLE
#include "UvK5.h"
#endif

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

    // Corps brut de HT_SEND_DATA (fragment AX.25 : [flags][data][chanId?]).
    using DataTxFn = std::function<void(const uint8_t*, size_t)>;

    void begin() { state_.begin(); aprs_.begin(); }
    void onNotify(NotifSink sink) { sink_ = std::move(sink); }
    void onDataTx(DataTxFn f)     { dataTxCb_ = std::move(f); }

    // Réglages APRS (BSS / path / position), pilotables depuis HTCommander et
    // lus par la balise autonome.
    AprsConfig& aprsConfig() { return aprs_; }

    // HTCommander a enregistré POSITION_CHANGE (File > GPS ou partage GPS série).
    // La vraie VR-N76 délègue alors le balisage à la radio -> notre balise
    // autonome doit tourner MÊME connectée, et pousser la position.
    bool positionShareWanted() const {
        return (registeredMask_ >> EventType::POSITION_CHANGE) & 1u;
    }

    // Pousse une notification POSITION_CHANGE (uniquement si enregistrée).
    void emitPositionChanged(int32_t latRaw, int32_t lonRaw) {
        emitEvent(EventType::POSITION_CHANGE,
                  BenshiReplies::positionChangedEvent(latRaw, lonRaw));
    }

    String  activeChannelName() { return state_.activeChannelName(); }
    uint8_t activeChannelId()   { return state_.activeChannelId(); }

    // Accès lecture seule pour l'écran de façade.
    RadioState::ActiveRf activeRf() { return state_.activeRf(); }
    uint8_t rssiRaw()  const { return rssi_.load(); }      // 0..15
    bool    sqOpen()   const { return sqOpen_.load(); }
    bool    inTx()     const { return inTx_.load(); }
    bool    aocConnected() const { return aocConnected_.load(); }
    uint8_t volume()   const { return volume_; }

    // Synchro GPS (module NMEA sur l'ESP) -> bit is_gps_locked du HT_STATUS,
    // affiché par HTCommander (barre d'état / carte).
    void setGpsLocked(bool l) {
        if (l == gpsLocked_.exchange(l)) return;
        emitHtStatusChanged();
    }

    // Émet une commande NON sollicitée sur le canal commande (ex. RX_DATA).
    void emitCommand(uint16_t cmd, const uint8_t* body, size_t len) {
        if (!sink_) return;
        BenshiMessage m;
        m.command_group = CommandGroup::BASIC;
        m.is_reply      = false;
        m.command       = cmd;
        m.body.assign(body, body + len);
        sink_(m);
    }

    // Mode SA818 : module RF réel à piloter (nullptr = mode "UV-K1" simulé).
    void setRfModule(Sa818* rf) { rf_ = rf; }

#if RF_MODULE_UVK5_ENABLE
    // Mode "UV-K1" : poste Quansheng piloté en série (mode hôte).
    void setUvK5(UvK5* u) { uvk5_ = u; }
#endif

    // Retune le module RF sur le canal actif (I/O UART bloquante ~ms).
    // À appeler depuis un contexte non critique (boucle Arduino), pas depuis
    // le callback Bluetooth.
    void syncRf() {
#if RF_MODULE_UVK5_ENABLE
        if (uvk5_ && uvk5_->present()) { syncUvK5(); return; }
#endif
        if (!rf_ || !rf_->present()) return;
        RadioState::ActiveRf rf = state_.activeRf();
        if (rf.tx_mhz < 1.0 || rf.rx_mhz < 1.0) {
            Serial.printf("[SA818] Canal actif %u : frequences invalides (tx=%.4f rx=%.4f) -> pas de retune\n",
                          state_.activeChannelId(), rf.tx_mhz, rf.rx_mhz);
            return;
        }
        // Niveau de squelch envoyé au module = celui réglé dans HTCommander
        // (WRITE_SETTINGS -> RadioState), borné 0..8 pour le SA818. 0 = squelch
        // ouvert (audio permanent, utile APRS/TNC). RF_MODULE_SQUELCH ne sert
        // plus qu'au choix COMPILE-TIME de la stratégie de capture (voir
        // AudioBridge : #if RF_MODULE_SQUELCH == 0).
        int sql = state_.squelch();
        if (sql > 8) sql = 8;
        bool ok = rf_->tune(rf.rx_mhz, rf.tx_mhz, rf.rx_ctcss_hz, rf.tx_ctcss_hz,
                            rf.wide, sql);
        // Filtres SA818 (pré/dé-emphase + HP + BP) : on suit STRICTEMENT le bit
        // pre_de_emph_bypass du canal actif, y compris sur le canal données
        // (APRS). Pour tester l'AFSK avec l'audio plat, mettre emph_bypass=true
        // sur le canal APRS dans config.h (ou via HTCommander). Aucun forçage.
        bool filt = !rf.emph_bypass;
        rf_->setFilters(filt, filt, filt);
        // Puissance : broche H/L du module (LOW = haute puissance, kv4p-ht).
#if (RF_MODULE_HL_GPIO >= 0)
        digitalWrite(RF_MODULE_HL_GPIO, rf.tx_at_max_power ? LOW : HIGH);
#endif
        Serial.printf("[SA818] Retune canal %u : RX %.4f / TX %.4f MHz, %s, CTCSS %.1f/%.1f, "
                      "squelch %d, filtres %s, puissance %s -> %s\n",
                      state_.activeChannelId(), rf.rx_mhz, rf.tx_mhz,
                      rf.wide ? "25kHz" : "12.5kHz", rf.rx_ctcss_hz, rf.tx_ctcss_hz,
                      sql, filt ? "ON" : "PLAT (bypass)",
                      rf.tx_at_max_power ? "HAUTE" : "basse",
                      ok ? "OK" : "ECHEC (module hors bande ? pas de reponse ?)");
    }

    // Cale TEMPORAIREMENT le module RF sur un autre canal mémoire (balise APRS
    // sur un canal dédié). N'affecte pas l'état "canal actif" -> restaurer avec
    // syncRf(). Sans effet en mode UV-K1. Renvoie true si le retune a eu lieu.
    bool syncRfToChannel(uint8_t id) {
        if (id >= CHANNEL_COUNT) return false;
        RadioState::ActiveRf rf = RadioState::decodeRf(state_.channelStruct(id));
        if (rf.tx_mhz < 1.0 || rf.rx_mhz < 1.0) return false;
#if RF_MODULE_UVK5_ENABLE
        if (uvk5_ && uvk5_->present()) return uvk5_->applyVfo(0, uvk5Params(rf));
#endif
        if (!rf_ || !rf_->present()) return false;
        int sql = state_.squelch(); if (sql > 8) sql = 8;
        bool ok = rf_->tune(rf.rx_mhz, rf.tx_mhz, rf.rx_ctcss_hz, rf.tx_ctcss_hz, rf.wide, sql);
        bool filt = !rf.emph_bypass;
        rf_->setFilters(filt, filt, filt);
        Serial.printf("[SA818] Balise : canal %u temporaire (TX %.4f MHz, %s) -> %s\n",
                      id, rf.tx_mhz, rf.wide ? "25kHz" : "12.5kHz", ok ? "OK" : "ECHEC");
        return ok;
    }

    // Canal de balise APRS réglé dans HTCommander (0 = canal courant).
    uint8_t autoShareLocCh() const { return state_.autoShareLocCh(); }

    // Paramètres RF / nom d'un canal mémoire quelconque (écran de façade :
    // afficher la fréquence du canal APRS pendant l'émission de la balise).
    RadioState::ActiveRf channelRf(uint8_t id) {
        RadioState::ActiveRf empty;
        return (id < CHANNEL_COUNT) ? RadioState::decodeRf(state_.channelStruct(id)) : empty;
    }
    String channelName(uint8_t id) { return state_.channelNameOf(id); }

    // Demande de changement de canal (tactile de l'écran ILI9341). Appliqué
    // depuis la boucle Arduino (pollRf) : retune + notification à HTCommander.
    void stepChannel(int delta) { pendingChanStep_.fetch_add(delta); }

    // Appelé périodiquement par le transport (boucle Arduino).
    void pollRf() {
        int step = pendingChanStep_.exchange(0);
        if (step) {
            int n = (int)state_.activeChannelId() + step;
            n %= CHANNEL_COUNT; if (n < 0) n += CHANNEL_COUNT;
            state_.setActiveChannel((uint8_t)n);
            Serial.printf("[CMD] tactile : canal actif -> %d\n", n);
            rfDirty_.store(true);
            emitHtStatusChanged();
        }
        if (rfDirty_.exchange(false)) syncRf();
        pollRssiSa818();
#if RF_MODULE_UVK5_ENABLE
        pollUvK5();
#endif
    }

    // MODE SA818 : interroge le RSSI réel du module ("RSSI?", 0..255) et le
    // mappe sur 0..15 pour le S-mètre du HT_STATUS Benshi — comme kv4p-ht.
    // En mode UV-K1 (pas de module), le RSSI reste celui dérivé du niveau
    // audio par le pont (setAudioRx). Uniquement pendant la réception.
    void pollRssiSa818() {
#if (RF_MODULE_RSSI_POLL_MS > 0)
        if (!rf_ || !rf_->present()) return;
        // Jamais pendant l'établissement de la connexion : "RSSI?" bloque l'UART
        // jusqu'à ~60 ms et retarderait les réponses aux commandes -> on attend
        // que le canal audio soit ouvert (handshake HTCommander terminé).
        if (!aocConnected_.load()) return;
        if (inTx_.load() || !sqOpen_.load()) return;   // signal reçu seulement
        uint32_t now = millis();
        if (now - lastRssiPollMs_ < (uint32_t)RF_MODULE_RSSI_POLL_MS) return;
        lastRssiPollMs_ = now;
        int v = rf_->readRssi();                        // 0..255, -1 si indispo
        if (v < 0) return;
        uint8_t r = (uint8_t)((v * 15 + 127) / 255);    // 0..255 -> 0..15
        if (r < 1) r = 1;                               // signal présent -> au moins 1
        sa818RssiValid_.store(true);
        if (sqOpen_.load() && r != rssi_.load()) {
            rssi_.store(r);
            emitHtStatusChanged();
        }
#endif
    }

#if RF_MODULE_UVK5_ENABLE
    // Traduit un canal Benshi -> paramètres RF du poste Quansheng.
    UvK5::RfParams uvk5Params(const RadioState::ActiveRf& rf) {
        UvK5::RfParams p;
        p.rxMHz = rf.rx_mhz;  p.txMHz = rf.tx_mhz;
        p.rxCtcssHz = rf.rx_ctcss_hz;  p.txCtcssHz = rf.tx_ctcss_hz;
        p.wide  = rf.wide;
        p.power = rf.tx_at_max_power ? 7 /*HIGH*/ : 4 /*LOW4*/;
        int sql = state_.squelch();  if (sql < 0) sql = 0;  if (sql > 9) sql = 9;
        p.squelch = (uint8_t)sql;
        return p;
    }

    void syncUvK5() {
        RadioState::ActiveRf rf = state_.activeRf();
        if (rf.tx_mhz < 1.0 || rf.rx_mhz < 1.0) {
            Serial.printf("[UVK5] canal %u : frequences invalides -> pas de retune\n",
                          state_.activeChannelId());
            return;
        }
        bool ok = uvk5_->applyVfo(0, uvk5Params(rf));
        uvk5_->setRadio(0, 0 /*dual watch off*/);
        Serial.printf("[UVK5] retune canal %u : RX %.4f / TX %.4f MHz, %s, "
                      "CTCSS %.1f/%.1f, squelch %d -> %s\n",
                      state_.activeChannelId(), rf.rx_mhz, rf.tx_mhz,
                      rf.wide ? "large" : "etroit", rf.rx_ctcss_hz, rf.tx_ctcss_hz,
                      state_.squelch(), ok ? "OK" : "ECHEC");
    }

    // Boucle Arduino : applique le PTT différé + keepalive + pousse le statut
    // (S-mètre / squelch) vers le HT_STATUS.
    void pollUvK5() {
        if (!uvk5_ || !uvk5_->present()) return;

        int8_t want = pendingPtt_.load();
        if (want >= 0) {
            if ((bool)want != pttApplied_) {
                pttApplied_ = (want != 0);
                // OFF : on répète l'ordre ~2 s (8 polls) : la trame PTT-OFF peut
                // se perdre à cause de la RF du PA sur la liaison série.
                pttResend_ = pttApplied_ ? 1 : 8;
            }
            if (pttResend_ > 0) {
                uvk5_->ptt(pttApplied_);
                pttResend_--;
            }
        }

        // Pas de GET_STATUS pendant l'émission : la RF du PA perturbe la
        // liaison série, la lecture échouerait et ferait "perdre" le poste.
        if (!pttApplied_)
            uvk5_->poll();   // GET_STATUS série (250 ms) : S-mètre, squelch, keepalive

        const UvK5::Status& s = uvk5_->lastStatus();
        // Statut périmé (> 1 s : liaison muette) -> on considère "pas de signal"
        // au lieu de garder la dernière valeur (sinon RX resté ouvert dans
        // HTCommander).
        bool stale = (s.stamp == 0) || (millis() - s.stamp > 1000);
        bool rx = !stale && s.sig && !pttApplied_;
        uint8_t r = rx ? (s.sMeter ? s.sMeter : 1) : 0;
#if AUDIO_DEBUG
        if (millis() - lastUvk5DbgMs_ > 1000) {
            lastUvk5DbgMs_ = millis();
            Serial.printf("[UVK5] func=%u sig=%d smeter=%u dBm=%d batt=%umV%s\n",
                          s.func, (int)s.sig, s.sMeter, s.rssiDbm, s.batterymV,
                          stale ? "  (STALE)" : "");
        }
#endif
        bool changed = (rx != sqOpen_.load()) || (r != rssi_.load());
        sqOpen_.store(rx);
        rssi_.store(r);
        if (changed) emitHtStatusChanged();
    }
#endif

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
            // Corps = liste d'octets de type (HTCommander en groupe parfois
            // plusieurs, précédés d'octets nuls) -> on traite tout.
            for (uint8_t t : in.body) if (t > 0 && t < 16) registeredMask_ |= (uint16_t)(1u << t);
            Serial.printf("[CMD] REGISTER_NOTIFICATION : masque = 0x%04X\n", registeredMask_);
            outMsg.body = BenshiReplies::registerNotifAck();

        } else if (in.command == CANCEL_NOTIFICATION) {
            for (uint8_t t : in.body) if (t > 0 && t < 16) registeredMask_ &= (uint16_t)~(1u << t);
            Serial.printf("[CMD] CANCEL_NOTIFICATION : masque = 0x%04X\n", registeredMask_);
            outMsg.body = BenshiReplies::registerNotifAck();

        } else if (in.command == READ_SETTINGS) {
            outMsg.body = BenshiReplies::settings(state_);

        } else if (in.command == WRITE_SETTINGS) {
            {
                uint8_t b0 = in.body.empty() ? 0 : in.body[0];
                uint8_t b5 = in.body.size() > 5 ? in.body[5] : 0;
                uint8_t b11 = in.body.size() > 11 ? in.body[11] : 0;
                Serial.printf("[STATE] WRITE_SETTINGS %u o : chA=%u chB=%u  b5=0x%02X b11=0x%02X"
                              "  auto_share_loc_ch=%u  (canal actif courant=%u)\n",
                              (unsigned)in.body.size(), (b0 >> 4) & 0x0F, b0 & 0x0F, b5, b11,
                              (uint8_t)((b5 & 0x1F) | ((b11 & 0x07) << 5)),
                              state_.activeChannelId());
            }
            bool ok = state_.setSettingsStruct(in.body.data(), in.body.size());
            outMsg.body = BenshiReplies::writeSettingsAck(
                ok ? ReplyStatus::SUCCESS : ReplyStatus::INVALID_PARAMETER);
            if (ok) {
                htStatusDirty_ = true; rfDirty_.store(true);   // VFO/canal -> retune
                // Canal de balise : conservé à part (AprsConfig), et ré-injecté
                // dans la structure pour survivre aux réécritures de HTCommander.
                aprs_.noteAutoShareLocCh(state_.autoShareLocCh());
                state_.setAutoShareLocCh(aprs_.beaconChannel());
            }

        } else if (in.command == READ_RF_CH) {
            uint8_t channelId = in.body.empty() ? 0 : in.body[0];
            uint8_t status;
            outMsg.body = BenshiReplies::rfChannel(state_, channelId, status);

        } else if (in.command == WRITE_RF_CH) {
            uint8_t channelId = in.body.empty() ? 0 : in.body[0];
            bool ok = state_.setChannelStruct(channelId, in.body.data(), in.body.size());
            outMsg.body = BenshiReplies::writeRfChAck(
                ok ? ReplyStatus::SUCCESS : ReplyStatus::INVALID_PARAMETER, channelId);
            if (ok) {
                htStatusDirty_ = true;
                if (channelId == state_.activeChannelId()) rfDirty_.store(true);  // canal actif édité -> retune
            }

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
                                                  rssi_.load(), aocConnected_.load(),
                                                  gpsLocked_.load());

        } else if (in.command == HT_SEND_DATA) {
            if (dataTxCb_) dataTxCb_(in.body.data(), in.body.size());
            outMsg.body = { ReplyStatus::SUCCESS };   // accusé -> HTCommander envoie le fragment suivant

        } else if (in.command == GET_VOLUME) {
            outMsg.body = { ReplyStatus::SUCCESS, volume_ };

        } else if (in.command == SET_VOLUME) {
            if (!in.body.empty()) {
                volume_ = in.body[0] & 0x0F;
                if (rf_ && rf_->present()) rf_->setVolume(volume_ ? volume_ : 1);
            }
            outMsg.body = { ReplyStatus::SUCCESS };

        } else if (in.command == SET_PHONE_STATUS) {
            outMsg.body = BenshiReplies::phoneStatusAck();

        } else if (in.command == READ_BSS_SETTINGS) {
            outMsg.body.assign(1, ReplyStatus::SUCCESS);
            outMsg.body.insert(outMsg.body.end(), aprs_.bss(),
                               aprs_.bss() + AprsConfig::BSS_LEN);
            Serial.printf("[APRS] READ_BSS_SETTINGS -> %u o (%s-%d)\n",
                          (unsigned)outMsg.body.size(),
                          aprs_.callsign().c_str(), aprs_.ssid());

        } else if (in.command == WRITE_BSS_SETTINGS) {
            Serial.printf("[APRS] WRITE_BSS_SETTINGS : %u o recus\n", (unsigned)in.body.size());
            aprs_.noteBssWrite();   // le WRITE_SETTINGS qui suit porte auto_share_loc_ch
            int r = aprs_.setBss(in.body.data(), in.body.size());
            outMsg.body = { r >= 0 ? ReplyStatus::SUCCESS : ReplyStatus::INVALID_PARAMETER };
            if (r == 1) bssDirty_ = true;   // notifie HTCommander uniquement si ça a changé
            else if (r < 0) Serial.println("[APRS]   -> REFUSE (taille < 46 ?)");

        } else if (in.command == GET_APRS_PATH) {
            outMsg.body.assign(1, ReplyStatus::SUCCESS);
            const String& p = aprs_.path();
            outMsg.body.insert(outMsg.body.end(), p.c_str(), p.c_str() + p.length());
            Serial.printf("[APRS] GET_APRS_PATH -> \"%s\"\n", p.c_str());

        } else if (in.command == SET_APRS_PATH) {
            char buf[64] = {0};
            size_t n = in.body.size() < sizeof(buf) - 1 ? in.body.size() : sizeof(buf) - 1;
            memcpy(buf, in.body.data(), n);
            Serial.printf("[APRS] SET_APRS_PATH : \"%s\"\n", buf);
            aprs_.setPath(String(buf));
            outMsg.body = { ReplyStatus::SUCCESS };

        } else if (in.command == SET_POSITION) {
            if (in.body.size() >= 6) {
                int32_t la = signExtend24((in.body[0] << 16) | (in.body[1] << 8) | in.body[2]);
                int32_t lo = signExtend24((in.body[3] << 16) | (in.body[4] << 8) | in.body[5]);
                Serial.printf("[APRS] SET_POSITION : %.5f, %.5f\n", la / 30000.0, lo / 30000.0);
                aprs_.setPositionRaw(la, lo);
                outMsg.body = { ReplyStatus::SUCCESS };
            } else {
                outMsg.body = { ReplyStatus::INVALID_PARAMETER };
            }

        } else if (in.command == GET_POSITION) {
            outMsg.body = BenshiReplies::position(aprs_.latRaw(), aprs_.lonRaw());
            Serial.printf("[APRS] GET_POSITION -> %.5f, %.5f\n", aprs_.lat(), aprs_.lon());

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
        if (bssDirty_) {
            bssDirty_ = false;
            // HTCommander NE relit PAS le BSS après une écriture : il ne se
            // rafraîchit QUE sur cette notification, et il ne s'y abonne pas
            // explicitement -> on l'émet toujours (comme la vraie VR-N76).
            emitEvent(EventType::BSS_SETTINGS_CHANGED, BenshiReplies::bssChangedEvent(),
                      /*force=*/true);
        }
    }

    // --- Hooks du pont audio (contexte tâche) ---------------------------------
    // Réception depuis le poste : squelch/RX ouverts + RSSI 0..15.
    // En mode SA818, dès qu'une lecture "RSSI?" a réussi, c'est elle qui fait
    // foi (pollRssiSa818) ; le niveau dérivé de l'ADC ne sert qu'en mode UV-K1
    // ou tant que le module n'a pas répondu.
    void setAudioRx(bool active, uint8_t rssi) {
#if RF_MODULE_UVK5_ENABLE
        // Mode UV-K1 piloté : is_sq / RSSI viennent du GET_STATUS série
        // (pollUvK5), qui fait autorité. On ignore l'estimation par le niveau
        // audio pour éviter que les deux se battent.
        if (uvk5_) return;
#endif
        uint8_t eff = !active ? 0
                    : (sa818RssiValid_.load() ? rssi_.load() : rssi);
        bool changed = (active != sqOpen_.load()) || (eff != rssi_.load());
        sqOpen_.store(active);
        rssi_.store(eff);
        if (!active) sa818RssiValid_.store(false);   // nouvelle salve -> reprise ADC en attendant "RSSI?"
        if (changed) emitHtStatusChanged();
    }
    // Émission vers le poste (HTCommander envoie de l'audio) : is_in_tx.
    void setAudioTx(bool tx) {
#if RF_MODULE_UVK5_ENABLE
        // Mode UV-K1 : le keying se fait par commande série (0x0633), appliquée
        // depuis la boucle Arduino (pollUvK5), pas depuis la tâche audio.
        if (uvk5_) pendingPtt_.store(tx ? 1 : 0);
#endif
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
                                                     rssi_.load(), aocConnected_.load(),
                                                     gpsLocked_.load());
        sink_(m);
    }

    // Émet une EVENT_NOTIFICATION. Par défaut seulement si le type est
    // enregistré ; `force` l'émet dans tous les cas.
    void emitEvent(uint8_t type, std::vector<uint8_t> body, bool force = false) {
        if (!sink_) return;
        if (!force && type < 16 && !(registeredMask_ & (1u << type))) return;
        BenshiMessage m;
        m.command_group = CommandGroup::BASIC;
        m.is_reply      = false;
        m.command       = BasicCommand::EVENT_NOTIFICATION;
        m.body          = std::move(body);
        sink_(m);
    }

    static int32_t signExtend24(int32_t v) {
        v &= 0x00FFFFFF;
        return (v & 0x00800000) ? (v - 0x01000000) : v;
    }

private:
    RadioState state_;
    AprsConfig aprs_;
    NotifSink  sink_;
    DataTxFn   dataTxCb_;
    Sa818*     rf_ = nullptr;
#if RF_MODULE_UVK5_ENABLE
    UvK5*      uvk5_ = nullptr;
    std::atomic<int8_t> pendingPtt_{-1};   // -1 aucun, 0/1 état PTT demandé
    bool       pttApplied_ = false;
    uint8_t    pttResend_  = 0;             // renvois restants de l'ordre PTT
    uint32_t   lastUvk5DbgMs_ = 0;
#endif

    uint16_t registeredMask_ = 0;   // bit t = type de notification t enregistré
    bool     htStatusDirty_  = false;
    bool     bssDirty_       = false;
    std::atomic<bool> rfDirty_{false};   // canal actif changé -> retune module RF
    std::atomic<int>  pendingChanStep_{0};   // pas de canal demandé par le tactile

    std::atomic<bool>    sqOpen_{false};
    std::atomic<uint8_t> rssi_{0};
    uint32_t             lastRssiPollMs_ = 0;    // MODE SA818 : dernier "RSSI?" (boucle Arduino seule)
    std::atomic<bool>    sa818RssiValid_{false}; // une lecture module a réussi
    std::atomic<bool>    aocConnected_{false};
    std::atomic<bool>    inTx_{false};
    std::atomic<bool>    gpsLocked_{false};
    uint8_t              volume_ = RF_MODULE_VOLUME;
};
