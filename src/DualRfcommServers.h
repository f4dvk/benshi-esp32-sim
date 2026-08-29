#pragma once
#include <Arduino.h>
#include <esp_spp_api.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_bt_api.h>
#include <vector>
#include "config.h"
#include "GaiaFrame.h"
#include "BenshiCommandHandler.h"
#include "VendorSdpRecord.h"
#include "AudioBridge.h"
#include "TncModem.h"
#include "AprsBeacon.h"
#include "GpsNmea.h"
#include "RadioDisplay.h"
#include "freertos/stream_buffer.h"

// ============================================================================
// MODULE LE PLUS EXPERIMENTAL DE CE PROJET - NON TESTE SUR MATERIEL REEL.
//
// Repartition des canaux RFCOMM, d'apres le CODE SOURCE de HTCommander
// (src/windows/runner/bluetooth_classic_plugin.cpp et
//  src/linux/runner/bluetooth_classic_plugin.cc) :
//   - canal COMMANDE/controle : service SDP SPP standard
//     00001101-0000-1000-8000-00805f9b34fb  (HTCommander : DoConnect ->
//     OpenRfcommSocket(..., {kSppUuid}))
//   - canal AUDIO (SBC)       : service SDP vendor "BS AOC"
//     39144315-32fa-40db-85ed-fbfeba2d86e6  (fallback : Generic Audio 0x1203)
//     (HTCommander : DoConnectAudio -> {kBsAocUuid, kGenericAudioUuid})
//
//   ATTENTION : c'est l'INVERSE de l'ancienne hypothese de ce projet
//   (qui supposait vendor=commande, SPP=audio). Corrige ici.
//
// DETECTION PAR HTCommander
// HTCommander identifie une "radio compatible" UNIQUEMENT par la presence du
// service SDP vendor 39144315-32fa-40db-85ed-fbfeba2d86e6 (ni la MAC, ni le
// nom, ni la Class of Device ne sont regardes - voir GetPairedDeviceList /
// bt_radio_service_uuids dans son code). esp_spp_start_srv() n'enregistre que
// des records SDP "SerialPort" 0x1101 ; ce service vendor est donc publie a
// part, via les primitives bas niveau SDP_* de Bluedroid (VendorSdpRecord.h),
// une fois les serveurs SPP demarres et le canal audio reel connu.
//
// -> Bascule USE_DUAL_RFCOMM_SERVERS a false dans config.h si ce module pose
//    probleme ; le firmware retombera sur un unique canal RFCOMM demultiplexe
//    par contenu (AudioRfcomm.h seul) - mais sans service vendor, donc non
//    detecte par HTCommander.
// ============================================================================

class DualRfcommServers {
public:
    // rf != nullptr && rf->present() -> mode SA818 (module RF réel piloté).
    bool begin(Sa818* rf = nullptr) {
        instance_ = this;
        handler_.setRfModule(rf);
        rf_ = rf;
        if (!audioMtx_)  audioMtx_  = xSemaphoreCreateMutex();
        if (!audioTxSb_) audioTxSb_ = xStreamBufferCreate(kAudioSbBytes, 1);

        // 0) Etat runtime de la radio (canaux / reglages / region), charge
        //    depuis la NVS ou, a defaut, depuis config.h.
        handler_.begin();

        // --------------------------------------------------------------
        // 1) Controleur radio.
        // ATTENTION : le sdkconfig d'Arduino-ESP32 est compile en
        // CONFIG_BTDM_CTRL_MODE_BTDM (dual mode). esp_bt_controller_enable()
        // EXIGE que le mode passe soit EXACTEMENT celui du sdkconfig, sinon
        // il renvoie ESP_ERR_INVALID_ARG et la radio ne demarre jamais.
        // On passe donc par btStart() (helper du core Arduino, le meme
        // qu'utilise BluetoothSerial) qui choisit le bon mode tout seul et
        // attend la transition IDLE -> INITED.
        // --------------------------------------------------------------
        if (!btStart()) {
            Serial.println("[SPP-DUAL] ERREUR: btStart() a echoue (controleur BT)");
            return false;
        }

        // 2) Pile hote Bluedroid.
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
            if (!check("esp_bluedroid_init", esp_bluedroid_init())) return false;
        }
        if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
            if (!check("esp_bluedroid_enable", esp_bluedroid_enable())) return false;
        }

        check("esp_bt_dev_set_device_name", esp_bt_dev_set_device_name(BT_CLASSIC_NAME));

        // 2bis) DECOUVERTE / APPAIRAGE PLUS RAPIDES.
        // Le controleur BT de l'ESP32 est en "modem sleep" par defaut
        // (CONFIG_BTDM_CTRL_MODEM_SLEEP) : il s'endort entre deux fenetres de
        // scan et ne repond donc pas toujours au 1er inquiry de l'hote -> il
        // faut parfois relancer le scan plusieurs fois. On desactive ce
        // sommeil : la radio ecoute en permanence, la decouverte et la
        // connexion sont quasi immediates. Cout : conso plus elevee (sans
        // importance pour un simulateur alimente en USB).
        if (check("esp_bt_sleep_disable", esp_bt_sleep_disable())) {
            Serial.println("[BT] Modem-sleep desactive (decouverte/connexion rapides)");
        }

        // 3) GAP + appairage.
        // Le callback GAP est indispensable pour repondre aux evenements
        // d'appairage SSP. Sans lui, l'appairage depuis un PC / telephone
        // moderne reste bloque et la connexion RFCOMM n'aboutit jamais.
        check("esp_bt_gap_register_callback",
              esp_bt_gap_register_callback(&DualRfcommServers::gapCallbackTrampoline));

        // IO capability = NoInputNoOutput. Consequence CLE : l'ESP32 annonce
        // qu'il n'a ni ecran ni clavier, donc le modele d'association SSP est
        // TOUJOURS "Just Works" - AUCUN code numerique n'est affiche ni a
        // confirmer cote PC/telephone (Bluedroid auto-accepte en interne et
        // ne notifie meme pas l'appli). C'est le comportement de la vraie
        // VR-N76. Si tu vois encore un code a 6 chiffres a confirmer, c'est
        // que ce firmware-ci n'est pas celui qui tourne : reflashe.
        // (Pas de garde #if : CONFIG_BT_SSP_ENABLED est toujours actif ici,
        //  et une garde mal evaluee par le preprocesseur retirerait ce reglage.)
        {
            esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
            check("esp_bt_gap_set_security_param(IOCAP=NONE)",
                  esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(uint8_t)));
            Serial.println("[GAP] IO capability = NoInputNoOutput -> appairage 'Just Works'");
        }
        // PIN fixe "0000" au cas (tres improbable) ou un appareil ne ferait
        // que du legacy pairing.
        {
            esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
            esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin);
        }

        // 4) SPP. esp_spp_init() est ASYNCHRONE : les serveurs RFCOMM doivent
        //    etre demarres depuis ESP_SPP_INIT_EVT, pas juste apres l'appel
        //    (sinon esp_spp_start_srv() renvoie ESP_ERR_INVALID_STATE et
        //    aucun canal n'est publie en SDP).
        check("esp_spp_register_callback",
              esp_spp_register_callback(&DualRfcommServers::sppCallbackTrampoline));
        if (!check("esp_spp_init", esp_spp_init(ESP_SPP_MODE_CB))) return false;

        const uint8_t* mac = esp_bt_dev_get_address();
        if (mac) {
            Serial.printf("[SPP-DUAL] MAC Bluetooth : %02X:%02X:%02X:%02X:%02X:%02X\n",
                          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        }
        Serial.println("[SPP-DUAL] Init lancee, attente de ESP_SPP_INIT_EVT...");

        // 5) Notifications non sollicitees (EVENT_NOTIFICATION) sur le canal
        //    commande : statut apres ecriture, squelch/RSSI pendant la reception.
        handler_.onNotify([this](const BenshiMessage& m) { sendCommandReply(m); });

        // 6) Pont audio SBC <-> DAC/ADC internes. L'encodeur micro envoie ses
        //    trames sur le canal RFCOMM audio via sendAudioData()/sendAudioEnd().
        audio_.onTxFrame([this](const uint8_t* sbc, size_t len) { sendAudioData(sbc, len); });
        audio_.onTxEnd([this]() { sendAudioEnd(); });
        // Reception depuis le poste -> is_sq / is_in_rx / rssi.
        audio_.onRxLevel([this](bool active, uint8_t rssi) {
            handler_.setAudioRx(active, rssi);
        });
        // Emission vers le poste (HTCommander envoie de l'audio) -> is_in_tx.
        audio_.onTxState([this](bool tx) { handler_.setAudioTx(tx); });
        if (!audio_.begin()) {
            Serial.println("[SPP-DUAL] AVERTISSEMENT: pont audio non demarre (commandes OK).");
        }

        // 7) Mode SA818 : cale le module sur le canal actif au demarrage.
        if (rf_ && rf_->present()) {
#if (RF_MODULE_HL_GPIO >= 0)
            pinMode(RF_MODULE_HL_GPIO, OUTPUT);
            digitalWrite(RF_MODULE_HL_GPIO, LOW);   // puissance haute par defaut
#endif
            rf_->setVolume(RF_MODULE_VOLUME);
            handler_.syncRf();                      // fait aussi tune + filtres + puissance
        }

#if TNC_ENABLE
        // 8) TNC AX.25 / AFSK 1200 pour le canal donnees "APRS".
#if APRS_GPS_ENABLE
        gps_.begin();
#endif
        tncSetup();
        tncReconcile();
#endif

#if DISPLAY_ENABLE
        // 9) Ecran de facade (ILI9225 / MCP23017).
#if DISPLAY_SPECTRUM
        display_.setPcmSource([this](int16_t* out) { audio_.copySpectrumPcm(out); });
#endif
        display_.begin();
#endif
        return true;
    }

    void sendAudioData(const uint8_t* sbcPayload, size_t len) {
        std::vector<uint8_t> f;
        f.reserve(len + 1);
        f.push_back(0x00);                   // Type = AudioData
        f.insert(f.end(), sbcPayload, sbcPayload + len);
        enqueueAudio(std::move(f));
    }

    void sendAudioEnd() { enqueueAudio(std::vector<uint8_t>{ 0x01 }); }  // Type = AudioEnd

#if DISPLAY_ENABLE
    // Instantané pour l'écran de façade (throttlé ; le rendu est ailleurs).
    void feedDisplay() {
        uint32_t now = millis();
        if (now - dispFeedMs_ < 250) return;
        dispFeedMs_ = now;

        RadioState::ActiveRf rf = handler_.activeRf();
        RadioFace f;
        f.rxMHz     = rf.rx_mhz;
        f.channelId = handler_.activeChannelId();
        strlcpy(f.channel, handler_.activeChannelName().c_str(), sizeof(f.channel));
        f.wide      = rf.wide;
        f.highPower = rf.tx_at_max_power;
        f.sMeter    = (uint8_t)((handler_.rssiRaw() * 9 + 7) / 15);   // 0..15 -> 0..9
        f.sqOpen    = handler_.sqOpen();
        f.tx        = audio_.txToRadio() || handler_.inTx();
#if TNC_ENABLE
        f.txAprs    = f.tx && (millis() - beaconQueuedMs_ < 6000);
        // Pendant la balise APRS, l'écran montre la fréquence du canal APRS
        // (celui sur lequel la trame part réellement), pas le canal écouté.
        if (f.txAprs && beaconDispCh_ < CHANNEL_COUNT) {
            RadioState::ActiveRf b = handler_.channelRf(beaconDispCh_);
            if (b.rx_mhz > 1.0) {
                f.rxMHz     = b.rx_mhz;
                f.wide      = b.wide;
                f.channelId = beaconDispCh_;
                strlcpy(f.channel, handler_.channelName(beaconDispCh_).c_str(), sizeof(f.channel));
            }
        }
#endif
        f.bt        = (cmdHandle_ != 0);
#if (TNC_ENABLE && APRS_GPS_ENABLE)
        f.gpsFix    = gps_.fixType();
        f.gpsSats   = gps_.sats();
        {
            uint8_t uh, um, us;
            if (gps_.utc(uh, um, us))
                snprintf(f.utc, sizeof(f.utc), "%02u:%02u:%02u", uh, um, us);
        }
#endif
        display_.set(f);
    }
#endif

    // A appeler regulierement depuis loop() : retune differe du module RF +
    // traces de mise au point de la chaine audio.
    void poll() {
        handler_.pollRf();
        flushAudioCoalesce();
#if TNC_ENABLE
#if APRS_GPS_ENABLE
        gps_.poll();
#endif
        tncReconcile();
        aprsBeaconTick();
        aprsBeaconChannelRestore();
#if APRS_GPS_ENABLE
        handler_.setGpsLocked(gps_.hasFix());   // -> is_gps_locked du HT_STATUS (HTCommander)
#endif
#endif
#if DISPLAY_ENABLE
        feedDisplay();
#endif

        // Filet de securite (reconnexion) : une connexion est en attente depuis
        // longtemps et AUCUN canal n'est encore mappe. Sur reconnexion, le
        // START_EVT du serveur (deja demarre) ne se represente pas toujours ->
        // le mapping par scn reste bloque. HTCommander ouvre TOUJOURS le canal
        // COMMANDE en premier -> on le force.
        if (cmdHandle_ == 0 && audioHandle_ == 0 && pendingConn_ != 0 &&
            millis() - pendingSinceMs_ > 1500) {
            Serial.printf("[SPP-DUAL] mapping COMMANDE force (handle=%lu, reconnexion)\n",
                          (unsigned long)pendingConn_);
            assignRole(pendingConn_, ROLE_CMD);
            pendingConn_ = 0;
            pendingListen_ = 0;
        }

        // Filet de securite : HTCommander a ouvert un 2e canal RFCOMM mais
        // n'y a encore rien envoye -> aucun mapping. Le canal COMMANDE est deja
        // identifie -> ce 2e canal ne peut etre que l'AUDIO.
        if (audioHandle_ == 0 && cmdHandle_ != 0 && pendingConn_ != 0 &&
            pendingConn_ != cmdHandle_ && millis() - pendingSinceMs_ > 700) {
            Serial.printf("[SPP-DUAL] mapping AUDIO force (handle=%lu, aucune donnee recue)\n",
                          (unsigned long)pendingConn_);
            assignRole(pendingConn_, ROLE_AUDIO);
            pendingConn_ = 0;
            pendingListen_ = 0;
        }

#if AUDIO_DEBUG
        uint32_t now = millis();
        if (now - dbgLastMs_ >= 1000) {
            dbgLastMs_ = now;
            uint32_t enc = audio_.encFramesTake();
            uint32_t sps = audio_.adcSamplesTake();
            uint32_t clip = audio_.micClipTake();
            Serial.printf(
                "[DBG] handles cmd=%lu audio=%lu pend=%lu cong=%d | I2S=%s ADC=%luHz clip=%lu | "
                "RX<-HTC: cmd=%u audioData=%uf/%uo types=0x%03X | "
                "mic: enc=%u SBC/s -> spp w=%u drop=%u sbuf=%uo | "
                "PTT=%d SQpin=%d RXgate=%d ADClvl=%lu agc=x%.1f heap=%u%s\n",
                (unsigned long)cmdHandle_, (unsigned long)audioHandle_,
                (unsigned long)pendingConn_, (int)audioCong_,
                audio_.txToRadio() ? "DAC(TX)" : "ADC(RX)", (unsigned long)sps, (unsigned long)clip,
                dbgCmdFrames_, dbgAudioFrames_, dbgAudioBytes_, dbgAudioTypes_,
                enc, dbgAudioSent_, dbgAudioDrop_,
                (unsigned)(audioTxSb_ ? xStreamBufferBytesAvailable(audioTxSb_) : 0),
                (int)audio_.txToRadio(), (int)audio_.squelchRaw(),
                (int)audio_.rxFromRadio(), (unsigned long)audio_.adcLevel(),
                audio_.agcGain(), (unsigned)ESP.getFreeHeap(),
#if TNC_ENABLE
                tncRunning_ ? " | TNC=DATA(AFSK)" : "");
#else
                "");
#endif
            dbgCmdFrames_ = dbgAudioFrames_ = dbgAudioBytes_ = 0;
            dbgAudioSent_ = dbgAudioDrop_ = 0;
            dbgAudioTypes_ = 0;
        }
#endif
    }

private:
    // ~248 trames SBC/s de ~92 o : envoyees une par une ca sature RFCOMM
    // (overhead par paquet). On regroupe donc plusieurs trames par
    // esp_spp_write (HTCommander re-decoupe sur les 0x7E). TOUT est en buffers
    // FIXES : aucune allocation dans le chemin audio -> pas de std::bad_alloc
    // quand le heap est serre (TNC + Bluedroid).
    static const size_t kAudioChunkBytes = 512;
    static const size_t kAudioSbBytes    = 3072;   // file d'octets vers le SPP

    void enqueueAudio(std::vector<uint8_t> payload) {
#if AUDIO_DEBUG
        if (!dbgTxDumped_ && payload.size() > 4 && payload[0] == 0x00) {
            dbgTxDumped_ = true;
            Serial.printf("[AUDIO-TX] 1re trame SBC (%u o) : %02X %02X %02X %02X %02X %02X ...\n",
                          (unsigned)(payload.size() - 1), payload[1], payload[2],
                          payload[3], payload[4], payload[5], payload[6]);
        }
#endif
        if (audioHandle_ == 0) {
            dbgAudioDrop_++;
#if AUDIO_DEBUG
            uint32_t now = millis();
            if (now - dbgNoAudioChanMs_ > 2000) {
                dbgNoAudioChanMs_ = now;
                Serial.println("[AUDIO-TX] !! audio a envoyer mais AUCUN canal audio RFCOMM");
            }
#endif
            return;
        }
        bool isEnd = (!payload.empty() && payload[0] == 0x01);
        lockAudio();
        appendFramedFixed(payload.data(), payload.size());
        audioCoalesceMs_ = millis();
        if (coalLen_ >= kAudioChunkBytes || isEnd) flushCoalesceLocked();
        unlockAudio();
        pumpAudioTx();
    }

    void flushAudioCoalesce() {   // depuis poll() : pas de reliquat qui traine
        lockAudio();
        if (coalLen_ && millis() - audioCoalesceMs_ > 12) flushCoalesceLocked();
        unlockAudio();
        pumpAudioTx();
    }

    void flushCoalesceLocked() {   // audioMtx_ deja pris
        if (!coalLen_ || !audioTxSb_) { coalLen_ = 0; return; }
        size_t w = xStreamBufferSend(audioTxSb_, coalBuf_, coalLen_, 0);   // non bloquant
        if (w < coalLen_) dbgAudioDrop_++;                                 // file pleine -> on jette
        coalLen_ = 0;
    }

    void appendFramedFixed(const uint8_t* payload, size_t len) {   // audioMtx_ deja pris
        auto put = [&](uint8_t b) { if (coalLen_ < sizeof(coalBuf_)) coalBuf_[coalLen_++] = b; };
        put(0x7E);
        for (size_t i = 0; i < len; i++) {
            uint8_t b = payload[i];
            if (b == 0x7E)      { put(0x7D); put(0x5E); }
            else if (b == 0x7D) { put(0x7D); put(0x5D); }
            else                  put(b);
        }
        put(0x7E);
    }

    void lockAudio()   { if (audioMtx_) xSemaphoreTake(audioMtx_, portMAX_DELAY); }
    void unlockAudio() { if (audioMtx_) xSemaphoreGive(audioMtx_); }

    // esp_spp_write en mode CB : une seule ecriture "en vol" (ESP_SPP_WRITE_EVT)
    // + respect de la congestion. Buffer d'ecriture FIXE.
    void pumpAudioTx() {
        lockAudio();
        if (audioWriteInFlight_ || audioCong_ || audioHandle_ == 0 || !audioTxSb_) {
            unlockAudio();
            return;
        }
        size_t n = xStreamBufferReceive(audioTxSb_, txWriteBuf_, sizeof(txWriteBuf_), 0);
        if (n == 0) { unlockAudio(); return; }
        audioWriteInFlight_ = true;
        unlockAudio();

        if (esp_spp_write(audioHandle_, n, txWriteBuf_) != ESP_OK) {
            lockAudio();
            audioWriteInFlight_ = false;
            unlockAudio();
            dbgAudioDrop_++;
        } else {
            dbgAudioSent_++;
        }
    }
    static void setClassOfDevice(uint32_t cod) {
        esp_bt_cod_t c;
        c.minor = (cod >> 2) & 0x3F;
        c.major = (cod >> 8) & 0x1F;
        c.service = (cod >> 13) & 0x7FF;
        esp_bt_gap_set_cod(c, ESP_BT_SET_COD_ALL);
    }

    void sendCommandReply(const BenshiMessage& msg) {
        if (cmdHandle_ == 0) return;
        std::vector<uint8_t> encoded = encodeMessage(msg);
        std::vector<uint8_t> framed = GaiaFrame::encode(encoded, false);
        esp_spp_write(cmdHandle_, framed.size(), framed.data());
    }

    static void sppCallbackTrampoline(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
        if (instance_) instance_->onSppEvent(event, param);
    }

    void onSppEvent(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
        switch (event) {
            case ESP_SPP_INIT_EVT: {
                if (param->init.status != ESP_SPP_SUCCESS) {
                    Serial.printf("[SPP-DUAL] ERREUR: ESP_SPP_INIT_EVT status=%d\n",
                                  param->init.status);
                    return;
                }
                Serial.println("[SPP-DUAL] SPP init OK, demarrage des 2 serveurs RFCOMM");

                // On DEMANDE des numeros de canal RFCOMM fixes (commande=1,
                // audio=2) mais la pile peut en attribuer d'autres : le canal
                // reel est lu dans ESP_SPP_START_EVT (param->start.scn) et
                // c'est LUI qui est publie dans le record SDP. Le canal
                // commande est demarre en 1er -> son record SPP 0x1101 est
                // enregistre en premier (HTCommander prend le 1er 0x1101).
                check("esp_spp_start_srv(CMD)",
                      esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE,
                                        CMD_SCN, "VR-N76-CMD"));
                check("esp_spp_start_srv(AUDIO)",
                      esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE,
                                        AUDIO_SCN, "VR-N76-AUDIO"));
                break;
            }

            case ESP_SPP_START_EVT: {
                if (param->start.status != ESP_SPP_SUCCESS) {
                    Serial.printf("[SPP-DUAL] ERREUR: ESP_SPP_START_EVT status=%d scn=%d\n",
                                  param->start.status, param->start.scn);
                    return;
                }
                // 1er START_EVT = serveur COMMANDE, 2e = serveur AUDIO
                // (ordre des esp_spp_start_srv() ci-dessus, evenements serialises).
                if (cmdScn_ == 0) {
                    cmdScn_ = param->start.scn;
                } else if (audioScn_ == 0) {
                    audioScn_ = param->start.scn;
                }
                Serial.printf("[SPP-DUAL] Serveur RFCOMM %s pret (scn=%d, listen handle=%lu)\n",
                              roleName(roleForScn(param->start.scn)),
                              param->start.scn, (unsigned long)param->start.handle);

                // Une connexion en attente de mapping (voir SRV_OPEN) est
                // resolue ici : le nouveau handle d'ecoute annonce reprend le
                // scn du serveur qui vient d'accepter le client.
                if (pendingConn_ && param->start.handle == pendingListen_) {
                    assignRole(pendingConn_, roleForScn(param->start.scn));
                    pendingConn_ = 0;
                    pendingListen_ = 0;
                }

                // La COD doit etre (re)posee apres le demarrage SPP : la pile
                // ajoute d'office le bit de service "Serial Port" et ecrase la
                // classe "audio-headset" qu'on veut imiter.
                if (++startedServers_ == 2) {
                    // Record SDP vendor "BS AOC" -> canal audio REEL. C'est CE
                    // service (et lui seul) qui rend la radio detectable par
                    // HTCommander. Publie APRES le demarrage SPP (base SDP
                    // initialisee, canal audio connu).
                    vendorSdpHandle_ = VendorSdp::publishBsAoc(audioScn_);
                    if (vendorSdpHandle_ == 0) {
                        Serial.println("[SPP-DUAL] ATTENTION : service vendor non publie "
                                       "-> HTCommander ne verra pas la radio.");
                    }

                    setClassOfDevice(DEVICE_CLASS_OF_DEVICE);

                    // EIR (Extended Inquiry Response) : on met le nom, les
                    // UUIDs (dont le service vendor) et la puissance TX DANS la
                    // reponse d'inquiry elle-meme. L'hote obtient tout en 1
                    // seul scan, sans avoir a enchainer une requete de nom
                    // separee -> "VR-N76" apparait tout de suite et complet.
                    {
                        esp_bt_eir_data_t eir;
                        memset(&eir, 0, sizeof(eir));
                        eir.fec_required    = false;
                        eir.include_txpower = true;
                        eir.include_uuid    = true;
                        eir.flag = ESP_BT_EIR_FLAG_GEN_DISC | ESP_BT_EIR_FLAG_DMT_HOST_SPT;
                        check("esp_bt_gap_config_eir_data",
                              esp_bt_gap_config_eir_data(&eir));
                    }

                    check("esp_bt_gap_set_scan_mode",
                          esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                                   ESP_BT_GENERAL_DISCOVERABLE));
                    Serial.printf("[SPP-DUAL] '%s' visible et connectable (EIR complet)\n",
                                  BT_CLASSIC_NAME);
                }
                break;
            }

            case ESP_SPP_SRV_OPEN_EVT: {
                if (param->srv_open.status != ESP_SPP_SUCCESS) {
                    Serial.printf("[SPP-DUAL] ERREUR: SRV_OPEN status=%d\n",
                                  param->srv_open.status);
                    return;
                }
                const uint8_t* a = param->srv_open.rem_bda;
                Serial.printf("[SPP-DUAL] Connexion entrante de %02X:%02X:%02X:%02X:%02X:%02X"
                              " (handle=%lu, new_listen=%lu, cmd=%lu audio=%lu)\n",
                              a[0], a[1], a[2], a[3], a[4], a[5],
                              (unsigned long)param->srv_open.handle,
                              (unsigned long)param->srv_open.new_listen_handle,
                              (unsigned long)cmdHandle_, (unsigned long)audioHandle_);

                // NE PAS comparer new_listen_handle aux handles memorises au
                // START_EVT : new_listen_handle est un handle NEUF cree pour
                // continuer d'ecouter, jamais celui d'origine (le slot
                // d'ecoute d'origine est recycle en slot de connexion). On
                // differe donc le mapping jusqu'au START_EVT de ce nouveau
                // handle, qui nous donnera le scn du serveur concerne.
                pendingConn_ = param->srv_open.handle;
                pendingListen_ = param->srv_open.new_listen_handle;
                pendingSinceMs_ = millis();
                break;
            }

            case ESP_SPP_CLOSE_EVT:
                if (param->close.handle == cmdHandle_)   { cmdHandle_ = 0;   cmdRxBuf_.clear(); }
                if (param->close.handle == audioHandle_) {
                    audioHandle_ = 0; audioRxFrame_.clear();
                    lockAudio();
                    if (audioTxSb_) xStreamBufferReset(audioTxSb_);
                    coalLen_ = 0;
                    audioWriteInFlight_ = false;
                    audioCong_ = false;
                    unlockAudio();
                    handler_.setAudioConnected(false); audio_.setChannelUp(false);
                    handler_.setAudioRx(false, 0);
                    handler_.setAudioTx(false);
                }
                if (param->close.handle == pendingConn_) { pendingConn_ = 0; pendingListen_ = 0; }
                Serial.println("[SPP-DUAL] Deconnexion d'un canal RFCOMM");
                break;

            case ESP_SPP_DATA_IND_EVT: {
                uint32_t h = param->data_ind.handle;
                const uint8_t* d = param->data_ind.data;
                uint16_t len = param->data_ind.len;

                // Filet de securite : si le mapping par scn n'a pas encore eu
                // lieu, on identifie le canal par le 1er octet recu (0xFF =
                // en-tete GaiaFrame, 0x7E = delimiteur de trame audio).
                if (h != cmdHandle_ && h != audioHandle_ && len > 0) {
                    if (d[0] == 0xFF)      assignRole(h, ROLE_CMD);
                    else if (d[0] == 0x7E) assignRole(h, ROLE_AUDIO);
                    if (h == pendingConn_) { pendingConn_ = 0; pendingListen_ = 0; }
                }

                if (h == cmdHandle_) {
#if AUDIO_DEBUG
                    if (len > 0 && d[0] == 0x7E)
                        Serial.println("[SPP-DUAL] !! trame AUDIO (0x7E) recue sur le canal COMMANDE "
                                       "-> mapping des canaux inverse ?");
#endif
                    handleCmdBytes(d, len);
                } else if (h == audioHandle_) {
                    handleAudioBytes(d, len);
                } else {
                    Serial.printf("[SPP-DUAL] %u octets sur un canal non identifie "
                                  "(handle=%lu, 1er octet 0x%02X)\n",
                                  len, (unsigned long)h, len ? d[0] : 0);
                }
                break;
            }

            case ESP_SPP_WRITE_EVT:
                if (param->write.handle == audioHandle_) {
                    lockAudio();
                    audioWriteInFlight_ = false;
                    audioCong_ = param->write.cong;
                    unlockAudio();
                    pumpAudioTx();
                }
                break;

            case ESP_SPP_CONG_EVT:
                if (param->cong.handle == audioHandle_) {
                    lockAudio();
                    audioCong_ = param->cong.cong;
                    unlockAudio();
                    if (!param->cong.cong) pumpAudioTx();
                }
                break;

            default:
                break;
        }
    }

    enum Role : uint8_t { ROLE_UNKNOWN, ROLE_CMD, ROLE_AUDIO };

    static const char* roleName(Role r) {
        return r == ROLE_CMD ? "COMMANDE" : (r == ROLE_AUDIO ? "AUDIO" : "?");
    }

    Role roleForScn(uint8_t scn) const {
        if (scn != 0 && scn == cmdScn_)   return ROLE_CMD;
        if (scn != 0 && scn == audioScn_) return ROLE_AUDIO;
        return ROLE_UNKNOWN;
    }

    void assignRole(uint32_t handle, Role role) {
        if (role == ROLE_CMD) {
            cmdHandle_ = handle;
            Serial.printf("[SPP-DUAL] -> canal COMMANDE (handle=%lu, scn cmd=%u audio=%u)\n",
                          (unsigned long)handle, cmdScn_, audioScn_);
        } else if (role == ROLE_AUDIO) {
            audioHandle_ = handle;
            lockAudio();
            audioWriteInFlight_ = false;
            audioCong_ = false;
            unlockAudio();
            handler_.setAudioConnected(true); audio_.setChannelUp(true);
            Serial.printf("[SPP-DUAL] -> canal AUDIO (handle=%lu, scn cmd=%u audio=%u)\n",
                          (unsigned long)handle, cmdScn_, audioScn_);
        } else {
            Serial.printf("[SPP-DUAL] -> canal INDETERMINE (handle=%lu) : ni cmd ni audio scn\n",
                          (unsigned long)handle);
        }
    }

    static bool check(const char* what, esp_err_t err) {
        if (err != ESP_OK) {
            Serial.printf("[SPP-DUAL] ERREUR: %s -> %s (0x%X)\n",
                          what, esp_err_to_name(err), err);
            return false;
        }
        return true;
    }

    // ---- GAP : appairage SSP "Just Works" + repli PIN legacy ---------------
    static void gapCallbackTrampoline(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* param) {
        switch (event) {
            case ESP_BT_GAP_AUTH_CMPL_EVT:
                if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                    Serial.printf("[GAP] Appairage OK avec '%s'\n", param->auth_cmpl.device_name);
                } else {
                    Serial.printf("[GAP] Echec d'appairage, stat=%d\n", param->auth_cmpl.stat);
                }
                break;
            case ESP_BT_GAP_CFM_REQ_EVT:
                // En "Just Works" (IO cap NONE), Bluedroid auto-accepte SANS
                // passer par ici : si cet evenement apparait, c'est que la
                // comparaison numerique a ete choisie (donc IO cap != NONE
                // cote ESP32 -> mauvais firmware) ou que le pair l'impose.
                // On accepte quand meme automatiquement.
                Serial.printf("[GAP] !! Comparaison numerique demandee (code %06u) -> auto-acceptee.\n"
                              "        Normalement invisible en 'Just Works' : verifie que le\n"
                              "        firmware flashe est bien celui-ci.\n",
                              (unsigned)param->cfm_req.num_val);
                esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
                break;
            case ESP_BT_GAP_KEY_NOTIF_EVT:
                Serial.printf("[GAP] Passkey a saisir cote pair : %06u\n",
                              (unsigned)param->key_notif.passkey);
                break;
            case ESP_BT_GAP_KEY_REQ_EVT:
                Serial.println("[GAP] Passkey demandee (non gere, IO cap = NONE)");
                break;
            case ESP_BT_GAP_PIN_REQ_EVT: {
                // Appairage legacy (vieux peripheriques) : PIN "0000".
                esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
                Serial.println("[GAP] PIN legacy 0000 fourni");
                break;
            }
            default:
                break;
        }
    }

    void handleCmdBytes(const uint8_t* data, size_t len) {
        cmdRxBuf_.insert(cmdRxBuf_.end(), data, data + len);
        while (true) {
            std::vector<uint8_t> encodedMsg;
            int consumed = GaiaFrame::tryDecode(cmdRxBuf_.data(), cmdRxBuf_.size(), encodedMsg);
            if (consumed == 0) break;
            if (consumed < 0) { cmdRxBuf_.clear(); break; }

            cmdRxBuf_.erase(cmdRxBuf_.begin(), cmdRxBuf_.begin() + consumed);

            BenshiMessage in;
            if (decodeMessage(encodedMsg.data(), encodedMsg.size(), in)) {
#if AUDIO_DEBUG
                dbgCmdFrames_++;
#endif
                BenshiMessage out;
                if (handler_.process(in, out)) {
                    sendCommandReply(out);
                    // Notification(s) de statut declenchee(s) par une ecriture,
                    // emise(s) APRES la reponse (ordre observe sur la vraie radio).
                    handler_.flushPendingNotifications();
                }
            }
        }
    }

    void handleAudioBytes(const uint8_t* data, size_t len) {
        for (size_t i = 0; i < len; i++) {
            uint8_t b = data[i];
            if (b == 0x7E) {
                if (!audioRxFrame_.empty()) dispatchAudioFrame(audioRxFrame_);
                audioRxFrame_.clear();
                audioEscapeNext_ = false;
                continue;
            }
            if (b == 0x7D) { audioEscapeNext_ = true; continue; }
            if (audioEscapeNext_) { b ^= 0x20; audioEscapeNext_ = false; }
            audioRxFrame_.push_back(b);
        }
    }

    void dispatchAudioFrame(const std::vector<uint8_t>& frame) {
        if (frame.empty()) return;
        uint8_t type = frame[0];
        const uint8_t* payload = frame.data() + 1;
        size_t         plen    = frame.size() - 1;
#if AUDIO_DEBUG
        dbgAudioTypes_ |= (type < 16) ? (uint16_t)(1u << type) : 0x8000;
#endif
        switch (type) {
            case 0x00:   // AudioData (numerotation "impaire" cote HTCommander)
            case 0x03:   // AudioData
                audio_.pushRadioSbc(payload, plen);
#if AUDIO_DEBUG
                dbgAudioFrames_++;
                dbgAudioBytes_ += plen;
#endif
                break;
            case 0x01:   // AudioEnd
                audio_.radioAudioEnd();
                break;
            case 0x02:   // AudioAck
                break;
            default:
                Serial.printf("[SPP-DUAL] Trame audio type 0x%02X ignoree (%u octets)\n",
                              type, (unsigned)plen);
                break;
        }
    }

    static DualRfcommServers* instance_;

    // Numeros de canal RFCOMM DEMANDES (la pile peut en donner d'autres ;
    // le canal reel est capture dans cmdScn_ / audioScn_ au START_EVT).
    // Le canal COMMANDE est demarre en 1er : les deux serveurs publient un
    // record SPP 0x1101, et HTCommander prend le PREMIER record 0x1101
    // renvoye (.GetAt(0)) pour le canal de commande.
    static const uint8_t CMD_SCN   = 1;
    static const uint8_t AUDIO_SCN = 2;

    uint32_t cmdHandle_ = 0, audioHandle_ = 0;
    uint32_t pendingConn_ = 0, pendingListen_ = 0, pendingSinceMs_ = 0;
    uint32_t vendorSdpHandle_ = 0;
    uint8_t  cmdScn_ = 0, audioScn_ = 0;
    uint8_t  startedServers_ = 0;
    std::vector<uint8_t> cmdRxBuf_;
    std::vector<uint8_t> audioRxFrame_;
    bool audioEscapeNext_ = false;
    BenshiCommandHandler handler_;
    AudioBridge audio_;
    Sa818* rf_ = nullptr;
#if DISPLAY_ENABLE
    RadioDisplay display_;
    uint32_t     dispFeedMs_ = 0;
#endif

#if TNC_ENABLE
    // --- TNC AX.25 / AFSK 1200 (canal APRS) -------------------------------
    TncModem tnc_;
    bool dataChanActive_ = false;                // radio sur le canal "APRS"
    uint32_t connUpSinceMs_ = 0;                 // date où COMMANDE+AUDIO tous deux mappés
    uint32_t lastBeaconMs_ = 0;                  // dernière balise APRS autonome
    uint32_t beaconQueuedMs_ = 0;                // -> label "TX APRS" sur l'ecran
    uint8_t  beaconRestoreCh_ = 0xFF;            // canal à restaurer après balise (0xFF = rien)
    uint8_t  beaconDispCh_ = 0xFF;               // canal réellement utilisé pour la balise (écran)
    uint32_t beaconTxStartMs_ = 0;
#if APRS_GPS_ENABLE
    GpsNmea gps_;
#endif
    std::vector<uint8_t> tncTxAccum_;            // réassemblage HT_SEND_DATA
    int      tncTxNextFrag_ = 0;
    StreamBufferHandle_t tncTxQ_ = nullptr;      // trames AX.25 completes -> tache TX
    StreamBufferHandle_t tncRxSb_ = nullptr;     // PCM ADC -> tache demod
    volatile bool tncRunning_ = false;
    TaskHandle_t tncTxTask_ = nullptr, tncRxTask_ = nullptr;

    bool tncOn() const { return dataChanActive_; }

    // Ne fait QUE brancher les callbacks (aucune allocation). Le modem, les
    // files et les tâches ne sont créés que quand la radio passe sur le canal
    // "APRS" (tncStart) et sont libérés en la quittant (tncStop) -> un firmware
    // en phonie ne paie rien pour le TNC (heap serré avec Bluedroid).
    void tncSetup() {
        tnc_.onRxFrame([this](const uint8_t* ax25, size_t len) { onTncRxFrame(ax25, len); });
        tnc_.onTxAudio([this](const int16_t* pcm, size_t n)   { audio_.dataTxAudio(pcm, n); });
        tnc_.onTxDone([this]() { audio_.dataTxEnd(); });
        audio_.onDataRxAudio([this](const int16_t* pcm, size_t n) {
            if (tncRunning_ && tncRxSb_)
                xStreamBufferSend(tncRxSb_, pcm, n * sizeof(int16_t), 0);
        });
        handler_.onDataTx([this](const uint8_t* body, size_t len) {
            if (tncRunning_) onHtSendData(body, len);
        });
        Serial.printf("[TNC] callbacks prets ; s'active sur le canal \"%s\"\n", TNC_CHANNEL_NAME);
    }

    void tncStart() {
        if (tncRunning_) return;
        Serial.printf("[TNC] demarrage... (heap libre %u)\n", (unsigned)ESP.getFreeHeap());
        tncTxQ_  = xStreamBufferCreate(512, 1);
        tncRxSb_ = xStreamBufferCreate(2048, 1);
        if (!tncTxQ_ || !tncRxSb_ || !tnc_.begin()) {
            Serial.println("[TNC] ECHEC init (heap ?) -> reste en phonie");
            tncTeardown();
            return;
        }
        tncRunning_ = true;
        xTaskCreatePinnedToCore(&DualRfcommServers::tncTxTrampoline, "tnc_tx",
                                4096, this, 4, &tncTxTask_, 1);
        xTaskCreatePinnedToCore(&DualRfcommServers::tncRxTrampoline, "tnc_rx",
                                4608, this, 5, &tncRxTask_, 1);
        Serial.printf("[TNC] actif (heap libre %u)\n", (unsigned)ESP.getFreeHeap());
    }

    void tncStop() {
        if (!tncRunning_) return;
        tncRunning_ = false;                        // les taches sortent puis s'auto-suppriment
        for (int i = 0; i < 25 && (tncTxTask_ || tncRxTask_); i++) vTaskDelay(pdMS_TO_TICKS(20));
        tnc_.end();
        if (tncTxQ_)  { vStreamBufferDelete(tncTxQ_);  tncTxQ_  = nullptr; }
        if (tncRxSb_) { vStreamBufferDelete(tncRxSb_); tncRxSb_ = nullptr; }
        tncTxAccum_ = std::vector<uint8_t>();
        tncTxNextFrag_ = 0;
        Serial.printf("[TNC] arrete (heap libre %u)\n", (unsigned)ESP.getFreeHeap());
    }

    void tncTeardown() {   // uniquement sur echec de tncStart (taches pas encore creees)
        tnc_.end();
        if (tncTxQ_)  { vStreamBufferDelete(tncTxQ_);  tncTxQ_  = nullptr; }
        if (tncRxSb_) { vStreamBufferDelete(tncRxSb_); tncRxSb_ = nullptr; }
    }

    static void tncRxTrampoline(void* self) { static_cast<DualRfcommServers*>(self)->tncRxLoop(); }
    void tncRxLoop() {
        int16_t pcm[128];
        uint32_t fed = 0, lastLog = millis();
        while (tncRunning_) {
            size_t br = xStreamBufferReceive(tncRxSb_, pcm, sizeof(pcm), pdMS_TO_TICKS(100));
            if (br >= sizeof(int16_t) && tncRunning_) {
                tnc_.feedRxAudio(pcm, br / sizeof(int16_t));
                fed += br / sizeof(int16_t);
            }
#if AUDIO_DEBUG
            if (millis() - lastLog >= 2000) {
                Serial.printf("[TNC] demod : %lu ech/s\n", (unsigned long)(fed / 2));
                fed = 0; lastLog = millis();
            }
#endif
        }
        tncRxTask_ = nullptr;
        vTaskDelete(nullptr);
    }

    // Le TNC ne démarre qu'une fois HTCommander PLEINEMENT connecté (canaux
    // COMMANDE **et** AUDIO mappés, puis TNC_START_DELAY_MS de stabilité : le
    // modem AFSK prend ~15 ko d'un coup et affamerait L2CAP sinon) et sur le
    // canal "APRS".
    void tncReconcile() {
        bool onAprs = handler_.activeChannelName() == String(TNC_CHANNEL_NAME);
        if (onAprs != dataChanActive_) {
            dataChanActive_ = onAprs;
            Serial.printf("[TNC] canal \"%s\" %s\n", TNC_CHANNEL_NAME,
                          onAprs ? "actif" : "quitte");
        }
        bool bothUp = (cmdHandle_ != 0 && audioHandle_ != 0);
        if (bothUp && connUpSinceMs_ == 0)  connUpSinceMs_ = millis();
        if (!bothUp)                        connUpSinceMs_ = 0;
        bool stable = bothUp &&
                      (millis() - connUpSinceMs_ >= (uint32_t)TNC_START_DELAY_MS);
        bool clientish = (cmdHandle_ != 0 || pendingConn_ != 0);

        // Le modem tourne pour : le canal DONNÉES APRS (client), OU la balise
        // autonome activée ("Partager ma position") -> celle-ci peut émettre
        // même si la radio écoute un autre canal (elle bascule sur le canal de
        // balise le temps de la trame, cf auto_share_loc_ch).
#if APRS_BEACON_ENABLE
        bool beaconWanted = handler_.aprsConfig().shouldShareLocation();
#else
        bool beaconWanted = false;
#endif
        bool need = dataChanActive_ || beaconWanted;

        // Décision :
        //  - ni données ni balise           -> non
        //  - déjà lancé                     -> on garde (balise/données), sinon
        //                                       tant qu'un client est/était là
        //  - aucun client en vue            -> oui (heap large, aucun risque L2CAP)
        //  - un client s'établit            -> on attend la stabilité (anti-crash)
        bool want;
        if (!need)                want = false;
        else if (tncRunning_)     want = true;
        else if (!clientish)      want = true;
        else                      want = stable;

        if (want && !tncRunning_) {
            tncStart();
        } else if (!want && tncRunning_) {
            audio_.setDataMode(false);
            vTaskDelay(pdMS_TO_TICKS(20));
            tncStop();
        }
        audio_.setDataMode(tncRunning_);
    }

    // Balise APRS autonome : trame de position générée par l'ESP, poussée dans
    // la file du modem. Émet si "Partager ma position" est coché, quel que soit
    // le canal écouté (bascule sur le canal de balise le temps de la trame).
    void aprsBeaconTick() {
#if APRS_BEACON_ENABLE
        if (!tncRunning_ || !tncTxQ_) return;
        // Connecté : on balise seulement si HTCommander a délégué le balisage à
        // la radio (File > GPS -> il enregistre POSITION_CHANGE). Sinon c'est
        // lui qui balise -> on se tait pour éviter le doublon.
        bool clientHere = (cmdHandle_ != 0 || pendingConn_ != 0);
        bool delegated  = handler_.positionShareWanted();
        if (clientHere && !delegated) return;

        AprsConfig& cfg = handler_.aprsConfig();

        // "Partager ma position" (onglet Beacon de HTCommander) = bit
        // shouldShareLocation du BSS, conservé en NVS. Décoché -> AUCUNE
        // émission APRS, y compris en mode autonome (sans application).
        if (!cfg.shouldShareLocation()) {
            static uint32_t warnMs = 0;
            if (millis() - warnMs > 120000UL) {
                warnMs = millis();
                Serial.println("[APRS] \"Partager ma position\" desactive (BSS) -> pas de balise");
            }
            return;
        }

        uint32_t now = millis();
        uint32_t period = (uint32_t)cfg.intervalSec() * 1000UL;   // réglé dans HTCommander (BSS)
        bool due = (lastBeaconMs_ != 0) && (now - lastBeaconMs_ >= period);
#if APRS_BEACON_AT_BOOT
        if (lastBeaconMs_ == 0 && now >= 30000) due = true;   // 1re balise ~30 s après boot
#else
        if (lastBeaconMs_ == 0) { lastBeaconMs_ = now; }      // démarre le compteur
#endif
        if (!due) return;
        lastBeaconMs_ = now;

        // Position : GPS si fix récent, sinon position fixe (réglée dans HTCommander).
        double lat = cfg.lat(), lon = cfg.lon();
        const char* src = "fixe";
#if APRS_GPS_ENABLE
        if (gps_.fix(lat, lon)) src = "GPS";
#endif
        // Toujours tenir la carte de HTCommander à jour, même sans indicatif.
        handler_.emitPositionChanged(AprsConfig::degToRaw(lat), AprsConfig::degToRaw(lon));

        // Indicatif non configuré -> pas d'émission RF (évite de baliser "NOCALL").
        {
            String c = cfg.callsign();
            if (c.length() == 0 || c == "NOCALL" || c == "N0CALL") {
                static uint32_t warnMs = 0;
                if (millis() - warnMs > 60000UL) {
                    warnMs = millis();
                    Serial.println("[APRS] indicatif non configure -> balise RF desactivee "
                                   "(regle APRS_CALLSIGN ou l'indicatif dans HTCommander)");
                }
                return;
            }
        }

        // Identité + icône + message : lus du BSS (réglés dans HTCommander).
        String call = cfg.callsign();
        String msg  = cfg.beaconMessage();
        String path = cfg.path();
        aprs::BeaconParams bp;
        bp.callsign = call.c_str();
        bp.ssid     = cfg.ssid();
        bp.path     = path.c_str();
        bp.symTable = cfg.symbolTable();
        bp.symCode  = cfg.symbolCode();
        bp.comment  = msg.c_str();

        uint8_t fr[300];
        size_t n = aprs::buildPositionFrame(fr, sizeof(fr), bp, lat, lon);
        if (!n) { Serial.println("[APRS] balise : trame trop longue, ignoree"); return; }

        // Canal de balise (onglet Beacon de HTCommander, "auto_share_loc_ch") :
        // 0 = canal courant ; sinon on cale le module RF sur le canal N-1 juste
        // avant d'émettre, et on restaure ensuite (aprsBeaconChannelRestore).
        uint8_t asc = handler_.aprsConfig().beaconChannel();
        beaconDispCh_ = (asc != 0) ? (uint8_t)(asc - 1) : handler_.activeChannelId();
        Serial.printf("[APRS] canal balise : %u (canal actif=%u)\n",
                      asc, handler_.activeChannelId());
        if (asc != 0 && beaconRestoreCh_ == 0xFF) {
            uint8_t target = (uint8_t)(asc - 1);
            if (target != handler_.activeChannelId() &&
                handler_.syncRfToChannel(target)) {
                beaconRestoreCh_ = handler_.activeChannelId();
                beaconTxStartMs_ = millis();
            }
        }

        uint16_t nn = (uint16_t)n;
        xStreamBufferSend(tncTxQ_, &nn, sizeof(nn), 0);
        xStreamBufferSend(tncTxQ_, fr,  nn,         0);
        beaconQueuedMs_ = millis();   // -> "TX APRS" sur l'ecran
        Serial.printf("[APRS] balise %s : %.5f, %.5f  (%s-%d '%c%c' -> %u o)\n",
                      src, lat, lon, call.c_str(), cfg.ssid(),
                      cfg.symbolTable(), cfg.symbolCode(), (unsigned)n);
#endif
    }

    // Restaure le canal RF après une balise émise sur un autre canal.
    void aprsBeaconChannelRestore() {
#if APRS_BEACON_ENABLE
        if (beaconRestoreCh_ == 0xFF) return;
        uint32_t age = millis() - beaconTxStartMs_;
        bool txDone  = (age > 4000) && !tnc_.transmitting() && !audio_.txToRadio()
                       && !audio_.dataTxActive();
        bool timeout = (age > 12000);
        if (txDone || timeout) {
            Serial.printf("[SA818] Balise : fin d'emission -> restauration du canal actif %u%s\n",
                          handler_.activeChannelId(), timeout ? " (timeout)" : "");
            handler_.syncRf();                 // retune sur le canal actif
            beaconRestoreCh_ = 0xFF;
        }
#endif
    }

    // HT_SEND_DATA : [flags][ax25 fragment][chanId?]. On réassemble puis on
    // pousse la trame complète à la tâche de modulation.
    void onHtSendData(const uint8_t* body, size_t len) {
        if (!len) return;
        uint8_t flags = body[0];
        bool    isFinal = flags & 0x80;
        bool    hasChan = flags & 0x40;
        uint8_t fragId  = flags & 0x3F;
        size_t  dataLen = len - 1 - (hasChan ? 1 : 0);
        const uint8_t* data = body + 1;

        if (fragId == 0) { tncTxAccum_.clear(); tncTxNextFrag_ = 0; }
        if ((int)fragId != tncTxNextFrag_) { tncTxAccum_.clear(); return; }   // désynchro
        tncTxAccum_.insert(tncTxAccum_.end(), data, data + dataLen);
        tncTxNextFrag_++;

        if (isFinal && !tncTxAccum_.empty()) {
            uint16_t n = (uint16_t)tncTxAccum_.size();
            xStreamBufferSend(tncTxQ_, &n, sizeof(n), 0);
            xStreamBufferSend(tncTxQ_, tncTxAccum_.data(), n, 0);
            tncTxAccum_.clear();
            tncTxNextFrag_ = 0;
        }
    }

    static void tncTxTrampoline(void* self) { static_cast<DualRfcommServers*>(self)->tncTxLoop(); }
    void tncTxLoop() {
        uint8_t frame[330];
        while (tncRunning_) {
            uint16_t n = 0;
            if (xStreamBufferReceive(tncTxQ_, &n, sizeof(n), pdMS_TO_TICKS(150)) != sizeof(n)) continue;
            if (n == 0 || n > sizeof(frame)) continue;
            size_t got = 0;
            while (got < n && tncRunning_) {
                got += xStreamBufferReceive(tncTxQ_, frame + got, n - got, pdMS_TO_TICKS(200));
            }
            if (!tncRunning_) break;
            Serial.printf("[TNC] TX trame AX.25 %u octets -> AFSK\n", n);
            handler_.setAudioTx(true);
            tnc_.txAx25(frame, n);           // modulate() bloque = temps réel
            vTaskDelay(pdMS_TO_TICKS(120));  // laisse la traîne DAC/PTT finir
            handler_.setAudioTx(false);
        }
        tncTxTask_ = nullptr;
        vTaskDelete(nullptr);
    }

    // Trame AX.25 reçue (FCS déjà vérifié/retiré) -> RX_DATA vers HTCommander,
    // fragmentée si > MTU. Format fragment : [0x00][flags][data][chanId].
    void onTncRxFrame(const uint8_t* ax25, size_t len) {
        Serial.printf("[TNC] RX trame AX.25 %u octets -> HTCommander\n", (unsigned)len);
        const size_t kMtu = 180;
        uint8_t chId = handler_.activeChannelId();
        size_t off = 0;
        uint8_t fragId = 0;
        while (off < len) {
            size_t chunk = (len - off) < kMtu ? (len - off) : kMtu;
            bool last = (off + chunk) >= len;
            std::vector<uint8_t> b;
            b.reserve(3 + chunk);
            b.push_back(0x00);                                   // octet 4 (ignoré)
            b.push_back((last ? 0x80 : 0x00) | 0x40 | (fragId & 0x3F));  // flags + hasChanId
            b.insert(b.end(), ax25 + off, ax25 + off + chunk);
            b.push_back(chId);
            handler_.emitCommand(BasicCommand::RX_DATA, b.data(), b.size());
            off += chunk;
            fragId++;
        }
    }
#endif  // TNC_ENABLE

    // File d'emission audio (esp_spp_write CB : 1 seule ecriture en vol).
    SemaphoreHandle_t audioMtx_ = nullptr;
    StreamBufferHandle_t audioTxSb_ = nullptr;   // octets framés -> SPP (fixe)
    uint8_t  coalBuf_[700];                      // regroupement (fixe)
    size_t   coalLen_ = 0;
    uint8_t  txWriteBuf_[576];                   // buffer d'écriture SPP (fixe)
    uint32_t audioCoalesceMs_ = 0;
    bool audioWriteInFlight_ = false;
    volatile bool audioCong_ = false;

    // Compteurs de mise au point (remis a zero chaque seconde par poll()).
    uint32_t dbgLastMs_     = 0;
    uint32_t dbgCmdFrames_  = 0;
    uint32_t dbgAudioFrames_ = 0;
    uint32_t dbgAudioBytes_ = 0;
    uint32_t dbgAudioSent_  = 0;
    uint32_t dbgAudioDrop_  = 0;
    uint32_t dbgNoAudioChanMs_ = 0;
    uint16_t dbgAudioTypes_ = 0;
    bool     dbgTxDumped_ = false;
};

inline DualRfcommServers* DualRfcommServers::instance_ = nullptr;
