#pragma once
#include <Arduino.h>
#include <esp_spp_api.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_bt_api.h>
#include <vector>
#include <deque>
#include "config.h"
#include "GaiaFrame.h"
#include "BenshiCommandHandler.h"
#include "VendorSdpRecord.h"
#include "AudioBridge.h"

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
        if (!audioMtx_) audioMtx_ = xSemaphoreCreateMutex();
        audioCoalesce_.reserve(kAudioChunkBytes + 128);

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

    // A appeler regulierement depuis loop() : retune differe du module RF +
    // traces de mise au point de la chaine audio.
    void poll() {
        handler_.pollRf();
        flushAudioCoalesce();

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
                "mic: enc=%u SBC/s -> spp chunks sent=%u drop~=%u qlen=%u | "
                "PTT=%d SQpin=%d RXgate=%d ADClvl=%lu\n",
                (unsigned long)cmdHandle_, (unsigned long)audioHandle_,
                (unsigned long)pendingConn_, (int)audioCong_,
                audio_.txToRadio() ? "DAC(TX)" : "ADC(RX)", (unsigned long)sps, (unsigned long)clip,
                dbgCmdFrames_, dbgAudioFrames_, dbgAudioBytes_, dbgAudioTypes_,
                enc, dbgAudioSent_, dbgAudioDrop_, (unsigned)audioTxQ_.size(),
                (int)audio_.txToRadio(), (int)audio_.squelchRaw(),
                (int)audio_.rxFromRadio(), (unsigned long)audio_.adcLevel());
            dbgCmdFrames_ = dbgAudioFrames_ = dbgAudioBytes_ = 0;
            dbgAudioSent_ = dbgAudioDrop_ = 0;
            dbgAudioTypes_ = 0;
        }
#endif
    }

private:
    // ~248 trames SBC/s de ~92 o : envoyees une par une, ca sature RFCOMM
    // (overhead par paquet). On regroupe donc plusieurs trames par
    // esp_spp_write (HTCommander re-decoupe sur les 0x7E). ~10 chunks en file.
    static const size_t kAudioChunkBytes = 512;
    static const size_t kAudioTxQMax     = 10;

    void enqueueAudio(std::vector<uint8_t> payload) {
#if AUDIO_DEBUG
        if (!dbgTxDumped_ && payload.size() > 4 && payload[0] == 0x00) {
            dbgTxDumped_ = true;
            Serial.printf("[AUDIO-TX] 1re trame SBC (%u o) : %02X %02X %02X %02X %02X %02X ...\n",
                          (unsigned)(payload.size() - 1), payload[1], payload[2],
                          payload[3], payload[4], payload[5], payload[6]);
            Serial.println("           (doit commencer par 9C = syncword SBC)");
        }
#endif
        if (audioHandle_ == 0) {
            dbgAudioDrop_++;
#if AUDIO_DEBUG
            uint32_t now = millis();
            if (now - dbgNoAudioChanMs_ > 2000) {
                dbgNoAudioChanMs_ = now;
                Serial.println("[AUDIO-TX] !! audio a envoyer mais AUCUN canal audio RFCOMM "
                               "(audioHandle_=0) -> HTCommander n'a pas ouvert le 2e canal");
            }
#endif
            return;
        }
        bool isEnd = (!payload.empty() && payload[0] == 0x01);
        lockAudio();
        appendFramed(audioCoalesce_, payload);
        audioCoalesceMs_ = millis();
        if (audioCoalesce_.size() >= kAudioChunkBytes || isEnd) flushCoalesceLocked();
        unlockAudio();
        pumpAudioTx();
    }

    // A appeler regulierement (poll) : evite qu'un petit reliquat traine.
    void flushAudioCoalesce() {
        lockAudio();
        if (!audioCoalesce_.empty() && millis() - audioCoalesceMs_ > 12) flushCoalesceLocked();
        unlockAudio();
        pumpAudioTx();
    }

    void flushCoalesceLocked() {                 // audioMtx_ deja pris
        if (audioCoalesce_.empty()) return;
        if (audioTxQ_.size() >= kAudioTxQMax) { audioTxQ_.pop_front(); dbgAudioDrop_ += 6; }
        audioTxQ_.push_back(std::move(audioCoalesce_));
        audioCoalesce_ = std::vector<uint8_t>();
        audioCoalesce_.reserve(kAudioChunkBytes + 128);
    }

    void lockAudio()   { if (audioMtx_) xSemaphoreTake(audioMtx_, portMAX_DELAY); }
    void unlockAudio() { if (audioMtx_) xSemaphoreGive(audioMtx_); }

    static void appendFramed(std::vector<uint8_t>& out, const std::vector<uint8_t>& payload) {
        out.push_back(0x7E);
        for (uint8_t b : payload) {
            if (b == 0x7E) { out.push_back(0x7D); out.push_back(0x5E); }
            else if (b == 0x7D) { out.push_back(0x7D); out.push_back(0x5D); }
            else out.push_back(b);
        }
        out.push_back(0x7E);
    }

    // esp_spp_write en mode CB : une seule ecriture "en vol", suite au
    // prochain ESP_SPP_WRITE_EVT (+ respect de la congestion).
    void pumpAudioTx() {
        std::vector<uint8_t> f;
        lockAudio();
        if (audioWriteInFlight_ || audioCong_ || audioTxQ_.empty() || audioHandle_ == 0) {
            unlockAudio();
            return;
        }
        f = std::move(audioTxQ_.front());
        audioTxQ_.pop_front();
        audioWriteInFlight_ = true;
        unlockAudio();

        if (esp_spp_write(audioHandle_, f.size(), f.data()) != ESP_OK) {
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
                    audioTxQ_.clear();
                    audioCoalesce_.clear();
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

    // File d'emission audio (esp_spp_write CB : 1 seule ecriture en vol).
    SemaphoreHandle_t audioMtx_ = nullptr;
    std::deque<std::vector<uint8_t>> audioTxQ_;
    std::vector<uint8_t> audioCoalesce_;
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
