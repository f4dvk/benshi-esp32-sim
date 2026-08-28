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
    const char* name;          // <= 10 caractères (80 bits / 8)
};

// Modifie librement cette table : c'est la "mémoire de canaux" figée
// de la radio simulée.
static const ChannelConfig CHANNELS[] = {
    { 0, 145.500000, 145.500000, MOD_FM, MOD_FM, 0.0,   0.0,   BW_WIDE, false, "SIMPLEX1"  },
    { 1, 146.520000, 146.520000, MOD_FM, MOD_FM, 0.0,   0.0,   BW_WIDE, false, "CALL"      },
    { 2, 439.987500, 439.987500, MOD_FM, MOD_FM, 0.0,   0.0,   BW_NARROW, false, "PMR1"    },
    { 3, 433.500000, 433.500000, MOD_FM, MOD_FM, 88.5,  88.5,  BW_NARROW, false, "ASSOC1"  },
    { 4, 144.800000, 144.800000, MOD_FM, MOD_FM, 0.0,   0.0,   BW_WIDE, false, "APRS"      },
    { 5, 145.500000, 145.500000, MOD_FM, MOD_FM, 0.0,   0.0,   BW_WIDE, true,  "RXONLY"    },
    { 6, 146.000000, 146.000000, MOD_FM, MOD_FM, 0.0,   0.0,   BW_WIDE, false, "CH7"       },
    { 7, 146.100000, 146.100000, MOD_FM, MOD_FM, 0.0,   0.0,   BW_WIDE, false, "CH8"       },
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
#define AUDIO_SAMPLE_RATE_HZ   32000
#define AUDIO_SBC_BITPOOL      40

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

// --- Entrée audio depuis le poste : ADC1 interne, lu par le même I2S en mode
//     "built-in ADC" (DMA). ADC1 uniquement (ADC2 entre en conflit radio).
//     Canaux ADC1 : 0=GPIO36 1=GPIO37 2=GPIO38 3=GPIO39 4=GPIO32 5=GPIO33
//                   6=GPIO34 7=GPIO35
//     GPIO34 = kv4p-ht PIN_AUDIO_IN = ADC1_CHANNEL_6.
#define AUDIO_ADC_ENABLE       true
#define AUDIO_ADC_CHANNEL      6        // GPIO34 (ADC1_CH6) — comme kv4p-ht
#define AUDIO_MIC_GAIN         8.0f     // gain numérique appliqué au PCM capté
#define AUDIO_MIC_DC_TRACK     true     // retire la composante continue (biais)

// --- PTT : SORTIE qui keye l'émission du poste quand HTCommander émet
//     (dès que des trames AudioData arrivent ; relâché après AUDIO_PTT_TAIL_MS
//     sans trame, ou sur AudioEnd).
//     kv4p-ht : PIN_PTT = 18, ACTIF À L'ÉTAT BAS (LOW = émission).
#define AUDIO_PTT_GPIO         18       // -1 = pas de commande PTT
#define AUDIO_PTT_ACTIVE_LOW   true
#define AUDIO_PTT_TAIL_MS      250

// --- Squelch : ENTRÉE indiquant que le poste reçoit un signal (squelch
//     ouvert). Sert à démarrer la capture ADC -> HTCommander et à renseigner
//     is_sq / is_in_rx (+ RSSI) dans HT_STATUS_CHANGED.
//     kv4p-ht : PIN_SQ = 32.  -1 = pas de fil squelch -> détection au niveau
//     du signal ADC (VOX) avec le seuil AUDIO_SQ_VOX_THRESH.
#define AUDIO_SQ_GPIO          32       // -1 = VOX audio
#define AUDIO_SQ_ACTIVE_LOW    true     // LOW = squelch ouvert (signal présent)
#define AUDIO_SQ_PULLUP        true
#define AUDIO_SQ_VOX_THRESH    600      // |PCM| moyen déclenchant la VOX (si SQ=-1)

// --- Bouton PTT local (optionnel) : force la capture ADC -> HTCommander,
//     comme si le squelch était ouvert. kv4p-ht : 5 ou 33.
#define AUDIO_PHYS_PTT_GPIO    -1       // ex. 5 ; actif à la masse

// --- LED d'état (optionnel) : allumée pendant TX vers le poste, clignote en
//     RX depuis le poste. kv4p-ht : 2 (LED interne).
#define AUDIO_STATUS_LED_GPIO  -1       // ex. 2
