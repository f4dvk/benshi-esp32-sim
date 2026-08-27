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
// 5) Audio
// ----------------------------------------------------------------------
#define AUDIO_SAMPLE_RATE_HZ   32000   // imposé par le protocole (SBC 32kHz)
// Brochage I2S pour un codec/mic I2S externe (INMP441 en entrée,
// MAX98357A en sortie, par ex.) - à adapter à ton câblage réel.
#define I2S_MIC_BCLK   26
#define I2S_MIC_WS     25
#define I2S_MIC_DATA   33
#define I2S_SPK_BCLK   27
#define I2S_SPK_WS     14
#define I2S_SPK_DATA   12
