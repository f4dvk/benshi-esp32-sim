#pragma once
#include <Arduino.h>

// ============================================================================
// TOUT CE QUI EST "EN DUR" EST ICI.
// C'est le seul fichier à modifier pour changer l'identité de la radio
// simulée, ses canaux, etc.
//
// Basé sur le dump `bluetoothctl` d'une VRAIE VR-N76 :
//   Device 38:D2:00:01:74:71 (public)
//   Name: VR-N76 / Alias: VR-N76
//   Class: 0x00200404 (Audio/Video, Wearable Headset Device, service Audio)
//   Icon: audio-headset
//   UUID: Serial Port             (00001101-0000-1000-8000-00805f9b34fb)
//   UUID: Handsfree Audio Gateway (0000111f-0000-1000-8000-00805f9b34fb)
//   UUID: Unknown                 (000088a1-0000-1000-8000-00805f9b34fb)
//   UUID: Vendor specific         (39144315-32fa-40db-85ed-fbfeba2d86e6)
//
// => Ce modèle n'utilise QUE du Bluetooth Classic (BR/EDR), pas de service
//    BLE. Ce projet est donc entièrement basé sur Bluetooth Classic (SPP).
// ============================================================================

// ----------------------------------------------------------------------
// 1) IDENTITE BLUETOOTH (les "verrous" nom / Class of Device / UUID)
// ----------------------------------------------------------------------
// Nom EXACT observé sur la vraie radio (pas de suffixe). Certaines UI
// filtrent la liste des appareils appairés/découverts par nom exact.
#define BT_CLASSIC_NAME        "VR-N76"

// Class of Device : Audio/Video (major=4), Wearable Headset Device (minor=1),
// service class = Audio (bit 21 global -> 0x100 dans le champ 11 bits).
// Reproduit à l'identique : c'est un vrai critère de détection possible
// côté OS/appli (icône, filtrage par classe), contrairement au nom qui est
// probablement cosmétique.
#define DEVICE_CLASS_OF_DEVICE 0x00200404

// UUID vendor-specific vu en SDP sur la vraie radio. Hypothèse la plus
// probable (protocole "GAIA", historiquement Qualcomm/CSR, dont s'inspire
// le protocole Benshi documenté par benlink) : ce canal vendor-specific
// porte les commandes GaiaFrame, et le SPP standard (1101) porte l'audio
// brut. C'est une HYPOTHESE, pas une certitude absolue (voir README pour
// comment la vérifier avec `sdptool` sur ta VR-N76 réelle).
#define VENDOR_SDP_UUID_128    "39144315-32fa-40db-85ed-fbfeba2d86e6"

// ----------------------------------------------------------------------
// 2) Adresse MAC Bluetooth
// ----------------------------------------------------------------------
// La vraie VR-N76 a la MAC Bluetooth 38:D2:00:01:74:71. On garde le meme
// prefixe constructeur (OUI 38:D2:00 + les 2 octets suivants) et on ne change
// QUE le dernier octet, pour ne pas entrer en conflit radio / cache
// d'appairage si les deux appareils sont allumes en meme temps. L'appairage
// BR/EDR renegocie de toute facon une cle de liaison propre a chaque pairing.
//
// IMPORTANT : mets ici la MAC *Bluetooth* que tu veux voir (celle affichee
// par `bluetoothctl` / Windows). Le firmware calcule tout seul la "base MAC"
// correspondante (BT = base + 2 sur l'ESP32) avant d'initialiser la radio.
//
// ATTENTION : ca ne rend PAS la radio detectable par HTCommander. HTCommander
// identifie une radio compatible UNIQUEMENT par la presence du service SDP
// vendor 39144315-32fa-40db-85ed-fbfeba2d86e6 (verifie dans son code source),
// jamais par la MAC ni par le nom. Voir README section detection.
#define OVERRIDE_BT_MAC        true
// MAC Bluetooth souhaitee (dernier octet different de la vraie radio : 72 vs 71)
static const uint8_t CUSTOM_BT_MAC[6] = {0x38, 0xD2, 0x00, 0x01, 0x74, 0x72};

// ----------------------------------------------------------------------
// 3) Informations "matériel" renvoyées à GET_DEV_INFO
// ----------------------------------------------------------------------
struct DevInfoConfig {
    uint8_t  vendor_id          = 0x02;     // arbitraire, ajuster si besoin
    uint16_t product_id         = 0x1234;   // arbitraire
    uint8_t  hw_ver             = 1;
    uint16_t soft_ver           = 100;      // ex: v1.00
    bool     support_radio      = true;
    bool     support_medium_pw  = true;
    bool     support_dmr        = false;
    uint8_t  channel_count      = 8;        // doit correspondre à CHANNELS[] plus bas
    uint8_t  freq_range_count   = 1;
    uint8_t  region_count       = 1;        // nb de "régions" (bandes réglementaires)
};
static const DevInfoConfig DEV_INFO;

// ----------------------------------------------------------------------
// 4) Canaux "en dur" (RfCh) - fréquences en MHz
// ----------------------------------------------------------------------
enum ModulationType : uint8_t { MOD_FM = 0, MOD_AM = 1, MOD_DMR = 2 };
enum BandwidthType   : uint8_t { BW_NARROW = 0, BW_WIDE = 1 };

struct ChannelConfig {
    uint8_t  channel_id;
    double   tx_freq_mhz;      // ex: 145.500000
    double   rx_freq_mhz;
    ModulationType tx_mod;
    ModulationType rx_mod;
    double   tx_sub_audio;     // tonalité CTCSS (Hz), 0 = aucune
    double   rx_sub_audio;
    BandwidthType bandwidth;
    bool     tx_disable;
    bool     emph_bypass;      // true = pas de pré/dé-emphase ni filtres HP/BP
                               // (audio "plat"). Recommandé sur un canal données
                               // AFSK/APRS ; laisser false pour la phonie.
    const char* name;          // <= 10 caractères (80 bits / 8)
};

// Modifie librement cette table : c'est la "mémoire de canaux" figée
// de la radio simulée.
static const ChannelConfig CHANNELS[] = {
    { 0, 145.500000, 145.500000, MOD_FM, MOD_FM, 0.0,   0.0,   BW_WIDE,   false, false, "SIMPLEX1" },
    { 1, 146.520000, 146.520000, MOD_FM, MOD_FM, 0.0,   0.0,   BW_WIDE,   false, false, "CALL"     },
    { 2, 439.987500, 439.987500, MOD_FM, MOD_FM, 0.0,   0.0,   BW_NARROW, false, false, "PMR1"     },
    { 3, 433.500000, 433.500000, MOD_FM, MOD_FM, 88.5,  88.5,  BW_NARROW, false, false, "ASSOC1"   },
    { 4, 144.800000, 144.800000, MOD_FM, MOD_FM, 0.0,   0.0,   BW_WIDE,   false, false, "APRS"     },
    { 5, 145.500000, 145.500000, MOD_FM, MOD_FM, 0.0,   0.0,   BW_WIDE,   true,  false, "RXONLY"   },
    { 6, 146.000000, 146.000000, MOD_FM, MOD_FM, 0.0,   0.0,   BW_WIDE,   false, false, "CH7"      },
    { 7, 146.100000, 146.100000, MOD_FM, MOD_FM, 0.0,   0.0,   BW_WIDE,   false, false, "CH8"      },
};
static const uint8_t CHANNEL_COUNT = sizeof(CHANNELS) / sizeof(CHANNELS[0]);

// ----------------------------------------------------------------------
// 4ter) Noms des "régions" (bandes réglementaires) renvoyés à
//        READ_REGION_NAME. Doit contenir DEV_INFO.region_count entrées.
// ----------------------------------------------------------------------
static const char* const REGION_NAMES[] = { "GLOBAL" };
static const uint8_t REGION_COUNT = sizeof(REGION_NAMES) / sizeof(REGION_NAMES[0]);

// ----------------------------------------------------------------------
// 4quater) État modifiable + persistance
// ----------------------------------------------------------------------
// HTCommander peut réécrire les canaux (éditeur de canal), changer le VFO /
// canal mémoire actif, le squelch, la région... (WRITE_RF_CH / WRITE_SETTINGS
// / SET_REGION). Ces modifications sont stockées en NVS et survivent au
// redémarrage (voir RadioState.h).
//
// Mets ceci à true pour effacer la NVS au prochain boot et repartir des
// valeurs figées ci-dessus (remets à false ensuite).
#define FACTORY_RESET_ON_BOOT  false

// Canal actif par défaut (VFO A / B) et paramètres généraux, renvoyés par
// READ_SETTINGS / GET_HT_STATUS.
static const uint8_t  DEFAULT_CHANNEL_A   = 0;
static const uint8_t  DEFAULT_CHANNEL_B   = 1;
static const uint8_t  DEFAULT_SQUELCH     = 4;   // 0-9
static const uint8_t  DEFAULT_MIC_GAIN    = 4;
static const uint8_t  DEFAULT_REGION      = 0;

// ----------------------------------------------------------------------
// 4bis) Transport RFCOMM Classic
// ----------------------------------------------------------------------
// true  : ouvre 2 canaux RFCOMM distincts (commande GaiaFrame + audio),
//         via l'API bas niveau esp_spp_api.h (DualRfcommServers.h). C'est
//         le plus fidèle à "separate channels" (doc protocole), mais
//         EXPERIMENTAL / non testé sur matériel réel par mes soins.
// false : un seul canal RFCOMM (BluetoothSerial standard, AudioRfcomm.h),
//         solution de repli plus simple mais qui multiplexe tout sur un
//         seul canal (ne respecte pas la séparation commande/audio).
#define USE_DUAL_RFCOMM_SERVERS true

// ----------------------------------------------------------------------
// 5) Audio - interface entre HTCommander (Bluetooth) et un poste réel
//    (ex. Quansheng UV-K1) via le DAC / l'ADC internes de l'ESP32.
// ----------------------------------------------------------------------
// Le canal RFCOMM audio transporte du SBC. Paramètres confirmés depuis le
// code source de HTCommander (src/lib/radio/audio_engine.dart) :
//   PCM  : 32 kHz, 16 bits, mono
//   SBC  : mono, 8 sous-bandes, 16 blocs, allocation "loudness", bitpool 40
//   1 trame SBC = 128 échantillons PCM (256 octets) -> ~88 octets SBC
// Framing sur le canal : délimiteurs 0x7E, échappement 0x7D (octet XOR 0x20).
// Types de trame (1er octet) : 0x00/0x03 = audio, 0x01 = fin, 0x02 = ACK.
//
// BROCHAGE ALIGNÉ SUR LE PROJET kv4p-ht (ESP32 WROOM-32) :
//   https://github.com/VanceVagell/kv4p-ht
// afin qu'une même carte / un même adaptateur serve aux deux projets.
//   kv4p-ht        GPIO   rôle ici
//   PIN_AUDIO_OUT   25    DAC -> entrée micro du poste (audio TX vers le poste)
//   PIN_AUDIO_IN    34    ADC <- sortie HP du poste  (audio RX depuis le poste)
//   PIN_PTT         18    sortie -> keye l'émission du poste
//   PIN_SQ          32    entrée <- "occupé / squelch ouvert" du poste
//   PIN_PHYS_PTT1    5    bouton PTT local (optionnel)
//   PIN_LED          2    LED d'état (optionnel)
//   PIN_PD          19    (kv4p : power-down du module RF interne ; inutile ici)
// Débit "fil" imposé par le protocole (SBC vers HTCommander).
#define AUDIO_SAMPLE_RATE_HZ   32000
#define AUDIO_SBC_BITPOOL      40

// Débit RÉEL de l'I2S (ADC + DAC).
//   = AUDIO_SAMPLE_RATE_HZ (32 kHz) : ADC/DAC au débit du codec SBC, AUCUN
//     ré-échantillonnage -> chemin le plus court, latence audio minimale.
//     C'est le mode par défaut.
//   = 48000 : échantillonne large façon kv4p-ht (filtres AFSK de la lib calés
//     sur 48 k) puis ré-échantillonne 48<->32 aux frontières SBC. Essais montrés
//     sans gain sur le décodage APRS + latence en plus -> désactivé.
// IMPORTANT : garder -DAFSK_SAMPLE_RATE de platformio.ini égal à cette valeur.
#define AUDIO_I2S_RATE         32000

// Le pont audio (AudioBridge.h) n'est actif qu'avec les 2 serveurs RFCOMM.
// Mets à false pour compiler un firmware "commandes seules" (aucun I2S/DAC/ADC).
#define AUDIO_BRIDGE_ENABLE    true

// --- Sortie audio vers le poste : DAC interne piloté par l'I2S en mode
//     "built-in DAC" (DMA). L'ESP32 pilote les DEUX broches DAC :
//       GPIO25 = DAC1  (= kv4p-ht PIN_AUDIO_OUT)  <- vers l'entrée micro du poste
//       GPIO26 = DAC2
//     Sortie 0..3,3 V sur 8 bits : prévoir un pont diviseur / atténuateur vers
//     le niveau micro du poste (quelques mV à quelques dizaines de mV).
#define AUDIO_DAC_ENABLE       true
#define AUDIO_SPK_VOLUME       0.80f    // 0..1, atténuation numérique avant le DAC
// Filtre passe-bas 1 pôle sur la sortie DAC : adoucit l'escalier 8 bits et le
// souffle de quantification. alpha = 1 - exp(-2*pi*fc/32000). 0.55 ≈ 4 kHz.
// À laisser false si le câblage kv4p-ht filtre déjà assez ; true si "bruits
// numériques" en émission.
#define AUDIO_DAC_LOWPASS      false
#define AUDIO_DAC_LP_ALPHA     0.55f

// --- Entrée audio depuis le poste : ADC1 interne, lu par le même I2S en mode
//     "built-in ADC" (DMA). ADC1 uniquement (ADC2 entre en conflit radio).
//     Canaux ADC1 : 0=GPIO36 1=GPIO37 2=GPIO38 3=GPIO39 4=GPIO32 5=GPIO33
//                   6=GPIO34 7=GPIO35
//     GPIO34 = kv4p-ht PIN_AUDIO_IN = ADC1_CHANNEL_6.
#define AUDIO_ADC_ENABLE       true
#define AUDIO_ADC_CHANNEL      6        // GPIO34 (ADC1_CH6) — comme kv4p-ht
// Gain numérique sur le PCM capté (multiplicateur direct de l'écart à la ligne
// de base ADC). Repère : 16 = "unité" (pleine échelle ADC 12 bits -> pleine
// échelle int16). Utilisé tel quel si AUDIO_AGC_ENABLE = false, sinon c'est
// juste le gain de départ de l'AGC.
#define AUDIO_MIC_GAIN         16.0f

// --- Contrôle automatique de gain (AGC) sur l'entrée ADC -----------------
// Ajuste le gain en continu pour viser AUDIO_AGC_TARGET (crête int16) quel que
// soit le niveau du signal reçu -> plus besoin de régler RF_MODULE_VOLUME /
// AUDIO_MIC_GAIN à la main, et pas d'écrêtage sur signal fort.
// Attaque rapide (baisse le gain vite si ça sature), retour lent (remonte
// doucement) ; le gain est gelé tant que le signal est sous AUDIO_AGC_NOISE
// (pas d'emballement sur le bruit de fond). Le démodulateur AFSK est
// insensible à l'amplitude (démod par différence de phase) -> l'AGC ne gêne
// pas l'APRS.
#define AUDIO_AGC_ENABLE       true
#define AUDIO_AGC_TARGET       8000.0f    // crête int16 visée (~25 % pleine échelle)
#define AUDIO_AGC_MIN_GAIN     1.0f
#define AUDIO_AGC_MAX_GAIN     24.0f
#define AUDIO_AGC_NOISE        400.0f     // sous ce niveau (crête, après gain) le gain est gelé
#define AUDIO_AGC_ATTACK_MS    20.0f      // baisse du gain : assez vite pour éviter l'écrêtage
#define AUDIO_AGC_RELEASE_MS   3000.0f    // remontée : très lente -> pas de pompage dans une trame
// Le gain n'est ajusté que quand le squelch est OUVERT (signal présent) ;
// fermé, il est GELÉ à sa dernière valeur -> pas d'emballement sur le bruit
// entre deux trames.
#define AUDIO_MIC_DC_TRACK     true     // retire la composante continue (biais)
// Polarisation de l'entrée ADC : l'ESP injecte VDD/2 sur GPIO26 (DAC2) pour
// centrer le signal audio de la cible (couplé en alternatif via un condo),
// exactement comme kv4p-ht. Actif uniquement hors émission.
#define AUDIO_ADC_BIAS_ENABLE  true
#define AUDIO_ADC_BIAS_CODE    128      // 0..255 (128 ≈ 1,65 V)

// IMPORTANT : sur l'ESP32, l'ADC interne et le DAC interne partagent l'unique
// I2S0 et NE peuvent PAS fonctionner en même temps. Le pont bascule donc I2S0
// entre "entrée ADC" (réception) et "sortie DAC" (émission) à chaque
// changement de PTT — comme kv4p-ht. Half-duplex, comme une vraie radio.
//
// Horloge I2S : APLL = clock beaucoup plus précise. kv4p-ht l'active pour
// l'ADC ET le DAC intégrés (config.use_apll = true dans rxAudio.h / txAudio.h),
// donc l'APLL fonctionne bien avec l'ADC/DAC interne sur ESP32.
// IMPORTANT pour le TNC : sans APLL, l'I2S se cale sur ~47744 Hz (-0,53 %)
// au lieu de 48000. Or la PLL du slicer AFSK (esp32-afsk) n'a qu'une plage
// de poursuite de ±0,51 % (ppm = 5100e-6) : une horloge à -0,53 % sort de
// cette fenêtre et empêche l'accrochage -> aucune trame décodée.
// -> true par défaut. Repasser à false uniquement si le son DAC est déformé.
#define AUDIO_I2S_APLL         true

// --- PTT : SORTIE qui keye l'émission du poste quand HTCommander émet
//     (dès que des trames AudioData arrivent ; relâché après AUDIO_PTT_TAIL_MS
//     sans trame, ou sur AudioEnd).
//     kv4p-ht : PIN_PTT = 18, ACTIF À L'ÉTAT BAS (LOW = émission).
#define AUDIO_PTT_GPIO         18       // -1 = pas de commande PTT
#define AUDIO_PTT_ACTIVE_LOW   true
#define AUDIO_PTT_TAIL_MS      250      // traîne PTT en PHONIE (flux SBC), pas en TNC
// Traîne PTT en émission DONNÉES (TNC/AFSK) : temps de maintien du PTT après le
// dernier échantillon du modulateur. Évite les coupures si le flux a un trou ;
// une fois la trame finie, c'est du PTT maintenu pour rien -> garder court.
#define AUDIO_DATA_TX_HANG_MS  40

// --- Squelch : ENTRÉE indiquant que le poste reçoit un signal (squelch
//     ouvert). Sert à démarrer la capture ADC -> HTCommander et à renseigner
//     is_sq / is_in_rx (+ RSSI) dans HT_STATUS_CHANGED.
//     kv4p-ht : PIN_SQ = 32 sur les cartes v1 ; = 4 sur les v2.0c / v2.0d !
//     -1 = pas de fil squelch -> détection au niveau du signal ADC (VOX,
//     seuil AUDIO_SQ_VOX_THRESH). Utile pour isoler un souci de squelch.
#define AUDIO_SQ_GPIO          32       // v1 = 32, v2.0c/d = 4, -1 = VOX audio
#define AUDIO_SQ_ACTIVE_LOW    true     // LOW = squelch ouvert (signal présent)
#define AUDIO_SQ_PULLUP        true
#define AUDIO_SQ_VOX_THRESH    600      // |PCM| moyen déclenchant la VOX (si SQ=-1)
// Traîne squelch : le pin SQ du SA818 clignote souvent en cours de réception
// (surtout sur un signal AFSK/APRS). On garde la capture ouverte pendant ce
// délai après la dernière fermeture -> un seul flux audio continu vers
// HTCommander au lieu de fragments hachés. 250-500 ms typique.
#define AUDIO_SQ_HANG_MS       500
#define AUDIO_SQ_ATTACK_MS     4        // anti-rebond à l'ouverture
// Pre-roll : nombre de ms d'audio capté AVANT l'ouverture du squelch que l'on
// pousse quand même à HTCommander. Le pin SQ du SA818 s'ouvre APRÈS le
// préambule d'une trame APRS -> sans pre-roll, HTCommander perd la synchro AX.25
// et ne décode pas. 150-250 ms.
#define AUDIO_RX_PREROLL_MS    220

// APRS / données : le pin SQ du SA818 ne suit PAS le paramètre de squelch
// (AT+DMOSETGROUP squelch=0) -> il reste piloté par la détection de porteuse.
// Mets true pour IGNORER le pin SQ et streamer l'audio EN PERMANENCE vers
// HTCommander (son modem logiciel décode alors n'importe quel paquet).
// Coût : ~23 ko/s de Bluetooth en continu + souffle joué en phonie.
#define AUDIO_RX_ALWAYS        false

// --- Bouton PTT local (optionnel) : force la capture ADC -> HTCommander,
//     comme si le squelch était ouvert. kv4p-ht : 5 ou 33.
#define AUDIO_PHYS_PTT_GPIO    -1       // ex. 5 ; actif à la masse

// --- LED d'état (optionnel) : allumée pendant TX vers le poste, clignote en
//     RX depuis le poste. kv4p-ht : 2 (LED interne).
#define AUDIO_STATUS_LED_GPIO  -1       // ex. 2

// ----------------------------------------------------------------------
// 6) Mode de fonctionnement : module RF SA818 vs poste externe "UV-K1"
// ----------------------------------------------------------------------
// Au démarrage l'ESP tente un handshake AT+DMOCONNECT sur l'UART RF :
//
//   * module SA818 / DRA818 détecté  -> MODE "SA818" : l'ESP pilote un VRAI
//     module radio. Les canaux de la section 4 deviennent des fréquences
//     RÉELLES ; tout changement de canal / VFO côté HTCommander retune le
//     module (AT+DMOSETGROUP). PTT (GPIO18) et squelch (GPIO32) pilotent /
//     lisent le module. C'est l'usage "façon kv4p-ht".
//
//   * aucun module détecté (après RF_MODULE_PROBES essais) -> MODE "UV-K1" :
//     comportement historique = simulation complète (fréquences, canaux,
//     statut) + passerelle audio/PTT/squelch vers un poste externe NON
//     pilotable (Quansheng UV-K1 ou autre).
//
// Dans les deux cas, l'interface vue par HTCommander (Bluetooth Classic,
// protocole Benshi) est IDENTIQUE.
//
// Brochage UART RF aligné sur kv4p-ht (PIN_RF_RXD 16 / PIN_RF_TXD 17 / PIN_PD 19).
#define RF_MODULE_ENABLE      true      // false -> toujours mode UV-K1 (pas de sonde UART)
#define RF_MODULE_UART_RX     16        // ESP RX  <- TX du module
#define RF_MODULE_UART_TX     17        // ESP TX  -> RX du module
#define RF_MODULE_PD_GPIO     19        // PD : HIGH = module alimenté (-1 = non câblé)
#define RF_MODULE_PROBES      3         // nb de handshakes AT+DMOCONNECT avant abandon
#define RF_MODULE_VOLUME      8         // volume HP du module, 1..8 (kv4p-ht : 8)
// Squelch du module SA818, 0..8. Valeur envoyée dans AT+DMOSETGROUP.
//   0 = squelch OUVERT : le module passe l'audio en permanence ET le firmware
//       ignore alors le pin SQ pour la capture (comme kv4p-ht) -> HTCommander
//       reçoit un flux continu, son modem logiciel décode l'APRS de lui-même.
//       Le pin SQ reste utilisé pour l'indicateur RX / le S-mètre.
//   1..8 = squelch fermé au repos, capture déclenchée par le pin SQ (phonie).
// Défaut 1 (phonie). Mettre 0 pour l'APRS / les données.
#define RF_MODULE_SQUELCH     1
#define RF_MODULE_WIDE        true      // true = 25 kHz, false = 12,5 kHz (si le canal ne le fixe pas)
// En MODE SA818 : période d'interrogation du RSSI réel du module (commande
// "RSSI?", comme kv4p-ht qui le fait toutes les 100 ms). La valeur 0..255 du
// module est mise à l'échelle 0..15 pour le S-mètre du HT_STATUS Benshi.
// Chaque lecture bloque l'UART jusqu'à ~60 ms -> ne pas descendre trop bas.
// N'est interrogé que pendant la réception (pin SQ ouvert), jamais en émission.
// 0 = désactive l'interrogation module -> RSSI dérivé du niveau audio (mode UV-K1).
#define RF_MODULE_RSSI_POLL_MS 250
// Broche H/L de puissance du module (kv4p-ht PIN_HL : v2.0c = 23, sinon -1).
// LOW = puissance haute, HIGH = puissance basse. Pilotée par le bit
// tx_at_max_power du canal.
#define RF_MODULE_HL_GPIO     -1

// Ce que le mode SA818 pilote sur le module, à chaque changement de canal :
//   - fréquences TX / RX                     (AT+DMOSETGROUP)   -> depuis le canal
//   - bande passante 12,5 / 25 kHz           (AT+DMOSETGROUP)   -> depuis le canal
//   - CTCSS TX / RX                          (AT+DMOSETGROUP)   -> depuis le canal
//   - squelch matériel 0..8                  (AT+DMOSETGROUP)   -> RF_MODULE_SQUELCH
//   - filtres pré/de-emphase + HP + BP       (AT+SETFILTER)     -> bit pre_de_emph_bypass du canal
//   - volume HP                              (AT+DMOSETVOLUME)  -> RF_MODULE_VOLUME (au boot)
//   - puissance haute / basse                (broche H/L)       -> bit tx_at_max_power (si RF_MODULE_HL_GPIO)

// ----------------------------------------------------------------------
// 6bis) Pilotage d'un poste Quansheng UV-K1 / UV-K5 V3 (port série)
// ----------------------------------------------------------------------
// Alternative au SA818 : si aucun module SA818 n'est détecté, l'ESP tente de
// dialoguer avec un poste Quansheng UV-K1 / UV-K5 V3 sur l'UART RF, en protocole
// série Quansheng (trame AB CD ... DC BA, charge masquée, CRC-16/XMODEM), à
// 38400 bauds. Le poste doit tourner le firmware du dossier firmware/uv-k1-k5v3/
// (mode hôte). L'ESP pilote alors fréquences/VFO/mémoires/mode/CTCSS/PTT depuis
// HTCommander, comme le mode SA818 mais avec beaucoup plus de contrôle.
// Réutilise le brochage RF_MODULE_UART_RX / _TX (16 / 17). L'audio reste
// analogique (HP du poste -> ADC ESP, DAC ESP -> micro du poste).
#define RF_MODULE_UVK5_ENABLE     false     // true -> sonde UV-K1 si pas de SA818
#define RF_MODULE_UVK5_BAUD       38400     // débit série du poste Quansheng
#define RF_MODULE_UVK5_KEEPALIVE_MS 3000    // période du GET_STATUS (watchdog mode hôte)
#define RF_MODULE_UVK5_MODULATION 0         // 0 = FM, 1 = AM, 2 = USB (canal Benshi = FM)
//
// CABLAGE (5 fils) :
//   poste PA9  (TXD)          -> ESP GPIO 16 (RX)      RF_MODULE_UART_RX
//   poste PA10 (RXD)          -> ESP GPIO 17 (TX)      RF_MODULE_UART_TX
//   poste sortie HP / casque  -> ESP GPIO 34 (ADC)     AUDIO_ADC_CHANNEL 6
//   ESP GPIO 25 (DAC)         -> poste entrée micro    (via condensateur, voir README)
//   masse commune
// PTT et squelch/RSSI passent par le PORT SERIE (commandes 0x0633 / 0x0634) :
// AUCUN fil PTT ni SQ. Le pin AUDIO_PTT_GPIO reste piloté (inoffensif, non câblé).
//
// A REGLER pour ce mode :
//   RF_MODULE_SQUELCH -> 0   (capture audio continue ; c'est le POSTE qui fait
//                            le squelch/CTCSS, via state_.squelch() de HTCommander,
//                            0 = données/APRS, 1..9 = phonie).
//   AUDIO_SQ_GPIO peut rester 32 (ignoré pour la capture quand RF_MODULE_SQUELCH=0).
//
// Squelch envoyé au poste : state_.squelch() de HTCommander, borné 0..9. Le
// firmware ferme le HP hors signal ; l'ESP lit le bit "signal reçu" du
// GET_STATUS pour le S-mètre / is_sq (pollUvK5).

// Test de sanité de la liaison série (Phase 1) : ne fait RIEN d'autre que
// sonder le poste et lire le registre BK4819 0x67 (RSSI) toutes les 2 s, en
// boucle. Ne démarre ni Bluetooth ni le reste. Nécessite un firmware compilé
// avec ENABLE_UART_RW_BK_REGS (le .bin actuel de firmware/uv-k1-k5v3/).
#define RF_MODULE_UVK5_SELFTEST   false

// ----------------------------------------------------------------------
// 7bis) TNC AX.25 / AFSK 1200 (canal données APRS)
// ----------------------------------------------------------------------
// Quand la radio est sur le canal nommé TNC_CHANNEL_NAME, l'ESP fait TNC :
//   - HT_SEND_DATA (HTCommander) -> trame AX.25 -> modulation AFSK1200 -> PTT
//     poste -> audio par le DAC.
//   - audio du poste -> démodulation AFSK1200 -> trame AX.25 -> notification
//     dataRxd vers HTCommander.
// Hors de ce canal, le pont audio reste en phonie (SBC).
// Dépend de dkaukov/esp32-afsk (GPL-3). false = pas de TNC, pas de dépendance.
#define TNC_ENABLE            true
#define TNC_CHANNEL_NAME      "APRS"
// true (défaut) : sur le canal APRS, l'audio est TOUJOURS encodé en SBC et
// streamé vers HTCommander, en plus du décodage TNC matériel -> son modem
// logiciel peut décoder en parallèle (FEC FX.25) et tu vois/écoutes la forme
// d'onde. Le premier des deux qui accroche gagne (HTCommander dédoublonne).
// false : sur le canal APRS, pas de flux audio (TNC matériel seul, ~23 ko/s
// de Bluetooth économisés).
#define TNC_ALSO_STREAM_AUDIO true

// Délai (ms) entre "HTCommander pleinement connecté" (canaux COMMANDE **et**
// AUDIO mappés) et le démarrage du modem AFSK. Le TNC consomme ~15 ko de heap
// d'un coup ; le faire pendant l'établissement de la 2e liaison RFCOMM affame
// L2CAP -> HTCommander décroche. On attend donc que la connexion soit stable.
#define TNC_START_DELAY_MS    3000

// --- Émission AFSK : temporisation d'accrochage (≈ "TXDelay") --------------
// Silence de porteuse AVANT le préambule : laisse au module RF le temps de
// vraiment passer en émission, et au récepteur le temps de caler son AGC/PLL.
// Un SA818/Quansheng met ~200-400 ms à monter -> une valeur trop faible fait
// perdre le début de la trame et RIEN n'est décodé.
// kv4p-ht utilise 1100 ms (lead) / 700 ms (tail).
// TNC_TX_TAIL_MS = silence de porteuse APRÈS les fanions de fermeture : juste
// le temps que le dernier bit sorte du module. 80-150 ms suffisent ; au-delà
// c'est du PTT maintenu pour rien. (Le pont ajoute encore AUDIO_DATA_TX_HANG_MS
// avant de relâcher le PTT.)
#define TNC_TX_LEAD_MS        1000.0f
#define TNC_TX_TAIL_MS          60.0f
// Amplitude de l'audio AFSK envoyé au DAC (0..1). 0.8 = comme kv4p-ht
// (TX_AFSK_GAIN). Baisser si sur-déviation (son "cassé" / splatter), monter si
// sous-déviation (trame trop faible). Dépend AUSSI de l'atténuateur matériel
// entre GPIO25 et l'entrée micro du module (voir README).
#define TNC_TX_GAIN           0.8f

// ----------------------------------------------------------------------
// 7ter) Balise APRS autonome (générée par l'ESP)
// ----------------------------------------------------------------------
// Quand la radio est sur le canal TNC_CHANNEL_NAME ("APRS") et qu'AUCUN
// HTCommander n'est connecté, l'ESP construit lui-même une trame de position
// APRS (AX.25 UI) et l'émet via le modem AFSK interne -> PTT -> module RF.
// Dès que HTCommander se connecte, c'est LUI qui gère les balises et l'ESP se
// tait -> pas de doublon.
//
// CONDITION : la balise n'émet QUE si "Partager ma position" (onglet Beacon de
// HTCommander = bit shouldShareLocation du BSS) est coché. Ce réglage est
// conservé en NVS -> en mode autonome, le dernier choix fait foi. Décoché =
// aucune émission APRS.
//
// CANAL : le réglage "Canal" de l'onglet Beacon (auto_share_loc_ch, dans la
// structure Settings) est pris en compte en MODE SA818 : "canal courant" =
// émission sur le canal actif ; un canal mémoire = le module est calé dessus
// juste avant la trame, puis le canal précédent est restauré. En mode UV-K1
// (poste externe), la balise part sur la fréquence où le poste est réglé.
//
// C'est bien l'ESP qui génère la trame (adresses AX.25 + champ position) ; le
// modem n'ajoute que préambule + bit-stuffing + FCS.
//
// LES VALEURS CI-DESSOUS NE SONT QUE LES DÉFAUTS (1re amorce en NVS). Ensuite,
// HTCommander lit et écrit ces réglages via son protocole :
//   READ/WRITE_BSS_SETTINGS  -> indicatif, SSID, icône, intervalle, message,
//                               ID station (ident au relâché de PTT)
//   GET/SET_APRS_PATH        -> chemin de digipeaters
//   GET/SET_POSITION         -> position fixe
// Ce que tu règles dans HTCommander est persisté (NVS namespace "aprs") et
// utilisé par la balise autonome. Voir src/AprsConfig.h.
// GPS ou fixe : la balise prend le fix GPS s'il est récent (APRS_GPS_ENABLE),
// sinon la position fixe réglée dans HTCommander (ou APRS_FIXED_LAT/LON).
#define APRS_BEACON_ENABLE        true
// Indicatif, 1..6 caractères. "NOCALL" = non configuré -> la balise autonome
// NE TRANSMET PAS tant que tu n'as pas mis ton indicatif (ici ou via HTCommander).
#define APRS_CALLSIGN             "NOCALL"
#define APRS_SSID                 7                  // 0..15 (7 = station fixe, 9 = mobile)
#define APRS_DEST                 "APZ001"           // TOCALL (APZxxx = expérimental) — non réglable via HTCommander
#define APRS_PATH                 "WIDE1-1,WIDE2-1"  // digis séparés par des virgules, "" = direct
#define APRS_SYMBOL_TABLE         '/'                // '/' primaire, '\\' alternative
#define APRS_SYMBOL               '-'                // icône : '-' maison, '>' voiture, '[' piéton, 'Y' voilier...
#define APRS_COMMENT              "Benshi ESP32 APRS" // message de balise (18 car. max côté BSS)
#define APRS_BEACON_INTERVAL_MIN  10                 // minutes entre deux balises (défaut si BSS = 0)
#define APRS_BEACON_AT_BOOT       true               // 1 balise ~30 s après le démarrage

// Position FIXE par défaut. Degrés décimaux : négatif = Sud / Ouest.
#define APRS_FIXED_LAT            47.218370
#define APRS_FIXED_LON            -1.552800
#define APRS_FIXED_ALT_M         (-1)                // altitude en m dans le commentaire ; -1 = ne pas inclure

// GPS NMEA -> balise "tracker" (position dynamique) + synchro affichée sur
// l'écran de façade. Repli sur la position fixe tant qu'il n'y a pas de fix
// valide (< APRS_GPS_FIX_MAX_AGE_S). Lu sur Serial1 matériel (RX seul).
#define APRS_GPS_ENABLE           true
#define APRS_GPS_RX_GPIO          4                  // Serial1 RX <- TX du GPS
#define APRS_GPS_BAUD             9600
#define APRS_GPS_FIX_MAX_AGE_S    30                 // au-delà, on considère le fix perdu

// ----------------------------------------------------------------------
// 7bis) Écran de façade "portatif VHF" — raccordé DIRECTEMENT sur le SPI de
//       l'ESP (plus de MCP23017). Deux familles, un seul écran à la fois :
//         - ILI9225 176x220 (passif)
//         - ILI9341 320x240 (+ tactile résistif XPT2046, auto-désactivé absent)
//       Détection au démarrage par lecture de l'ID sur le bus SPI (-> MISO
//       OBLIGATOIRE). Câblage identique pour les deux.
// ----------------------------------------------------------------------
#define DISPLAY_ENABLE            true

#define DISPLAY_DRIVER_ILI9225   0   // force le pilote ILI9225
#define DISPLAY_DRIVER_ILI9341   1   // force le pilote ILI9341 (+ tactile)
#define DISPLAY_DRIVER_AUTO      2   // détection : ID 0x9341 -> ILI9341 ;
                                     //   ID 0x9225 -> ILI9225 ; rien -> aucun
#define DISPLAY_DRIVER           DISPLAY_DRIVER_AUTO
// BEAUCOUP de modules ILI9225 SPI n'ont PAS de broche MISO/SDO -> impossible
// de lire leur ID. Par défaut, en AUTO, tout écran qui n'est pas un ILI9341
// est donc supposé être un ILI9225. Mettre à false pour une détection stricte
// (ID 0x9225 obligatoire) si tu veux vraiment "aucun pilote sans écran"
// -- dans ce cas un ILI9225 sans SDO doit être forcé (DISPLAY_DRIVER_ILI9225).
#define DISPLAY_AUTO_FALLBACK_ILI9225  true

// Pilotes COMPILÉS (en AUTO : les deux).
#define DISPLAY_HAS_ILI9225  (DISPLAY_ENABLE && DISPLAY_DRIVER != DISPLAY_DRIVER_ILI9341)
#define DISPLAY_HAS_ILI9341  (DISPLAY_ENABLE && DISPLAY_DRIVER != DISPLAY_DRIVER_ILI9225)

// --- Bus SPI de l'écran (commun ILI9225 / ILI9341) ---
#define DISPLAY_SPI_SCK          14
#define DISPLAY_SPI_MOSI         13
#define DISPLAY_SPI_MISO         35   // entrée seule ; REQUIS pour la détection auto (+ tactile)
#define DISPLAY_CS_PIN           27
#define DISPLAY_DC_PIN           33   // = RS
#define DISPLAY_RST_PIN          22   // -1 = câblé sur EN / 3,3 V + RC
#define DISPLAY_LED_PIN         (-1)  // -1 = câblé sur 3,3 V ; sinon GPIO (PWM possible)

#define ILI9225_SPI_HZ           20000000   // monter jusqu'à ~33 MHz si le câblage est court/propre
#define ILI9225_ROTATION         1    // 0..3 ; 1 = paysage 220x176
#define ILI9225_INVERT           false

#define ILI9341_SPI_HZ           40000000
#define ILI9341_ROTATION         1    // 0..3 ; 1 = paysage 320x240
#define ILI9341_INVERT           false

// --- Tactile XPT2046 (ILI9341 uniquement, même bus SPI) ---
#define TOUCH_ENABLE             true                // auto-désactivé si la puce ne répond pas
#define TOUCH_CS_PIN             21
#define TOUCH_IRQ_PIN            36                  // entrée seule ; -1 = interrogation sans IRQ
#define TOUCH_SPI_HZ             2000000
#define TOUCH_TAP_MAX_MS         600                 // tap gauche/droite = canal -1 / +1
#define TOUCH_CAL_X0             300                 // étalonnage : brut aux bords (0..4095)
#define TOUCH_CAL_X1             3800
#define TOUCH_CAL_Y0             300
#define TOUCH_CAL_Y1             3800

#define DISPLAY_FULL_CLEAR       true
#define DISPLAY_REFRESH_MS       130                 // cadence de la tâche d'affichage
#define DISPLAY_SPECTRUM         true                // analyseur de spectre de l'audio reçu (FFT 256 pts)
#define DISPLAY_SPECTRUM_MS      130
// L'écran démarre APRÈS le Bluetooth (publication du service SDP) pour ne pas
// ralentir l'appairage.
#define DISPLAY_START_DELAY_MS   6000

// ----------------------------------------------------------------------
// 7) Traces de mise au point
// ----------------------------------------------------------------------
// true -> le firmware imprime périodiquement (1 s) l'état du pont audio :
// canaux RFCOMM, débit audio TX/RX, pertes de congestion, PTT, squelch.
// Indispensable tant que l'audio n'est pas validé sur matériel.
#define AUDIO_DEBUG           true
