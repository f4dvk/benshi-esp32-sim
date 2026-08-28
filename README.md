# Simulateur de radio Benshi (VR-N76) sur ESP32 (pour HTCommander)

> ⚠️ **PROJET EN COURS DE DÉVELOPPEMENT — première version, non finalisée.**
>
> Ce qui fonctionne : détection par HTCommander, appairage, dialogue de
> commande/contrôle (infos radio, réglages, statut), lecture **et édition**
> des canaux mémoire avec persistance en mémoire flash (NVS), sélection du
> VFO / canal actif, région.
>
> Audio : **implémenté mais non validé sur matériel.** Codec SBC 32 kHz
> (encodeur + décodeur de `libbt.a`), framing du canal audio, pont vers le
> **DAC / ADC internes**, **sortie PTT et entrée squelch** pour piloter un
> poste réel (brochage aligné sur [kv4p-ht](https://github.com/VanceVagell/kv4p-ht)),
> et notifications de statut (`HT_STATUS_CHANGED` : TX / squelch / S-mètre).
> Il reste à régler à l'oreille le mode ADC+DAC interne simultané et à câbler
> l'étage analogique (voir « Audio » plus bas). Développé sans matériel de test.
>
> Ce qui **ne fonctionne pas encore** : le mode VFO fréquence libre.
>
> Utilisez-le pour l'interopérabilité Bluetooth / le test de HTCommander,
> pas comme une vraie radio. Retours et contributions bienvenus.

Ce projet PlatformIO fait passer un ESP32 pour une radio Benshi VR-N76, afin
que [HTCommander](https://github.com/Ylianst/HTCommander) puisse s'y
connecter. Les données radio par défaut (fréquences, canaux, réglages) sont
**définies dans `src/config.h`** puis modifiables à chaud depuis HTCommander
(les modifications sont conservées en NVS).

## ⚠️ Changement important par rapport à la doc protocolaire

La doc de référence benlink décrit un transport BLE (GATT) en plus du RFCOMM.
Mais le dump `bluetoothctl` d'une **vraie VR-N76** que tu as fourni montre
qu'elle n'utilise **que du Bluetooth Classic (BR/EDR)** :

```
Device 38:D2:00:01:74:71 (public)
Name: VR-N76 / Alias: VR-N76
Class: 0x00200404 (Audio/Video, Wearable Headset Device, service Audio)
Icon: audio-headset
UUID: Serial Port               (00001101-0000-1000-8000-00805f9b34fb)
UUID: Handsfree Audio Gateway   (0000111f-0000-1000-8000-00805f9b34fb)
UUID: Unknown                    (000088a1-0000-1000-8000-00805f9b34fb)
UUID: Vendor specific            (39144315-32fa-40db-85ed-fbfeba2d86e6)
```

Aucun service GATT BLE n'apparaît. Ce projet est donc **entièrement basé sur
Bluetooth Classic**. C'est aussi une nécessité technique : NimBLE (pile BLE)
et Bluedroid Classic (SPP) ne peuvent pas cohabiter dans un même firmware
ESP32 (deux piles hôte incompatibles sur un même contrôleur radio). Un
ancien fichier `BleServer.h` (BLE/NimBLE) reste dans le repo pour référence
si tu as un autre modèle Benshi qui, lui, utilise vraiment du BLE — il n'est
plus compilé par défaut.

## Ce qui est implémenté

- **Identité Bluetooth Classic** (`DualRfcommServers.h`) : nom `VR-N76`
  exact, Class of Device `0x00200404` reproduite à l'identique (Audio/Video,
  Wearable Headset Device, service Audio — cohérent avec l'icône
  "audio-headset" observée).
- **Deux canaux RFCOMM distincts** via l'API bas niveau `esp_spp_api.h` :
  un pour les commandes (SPP `0x1101`, encapsulées en `GaiaFrame`, voir
  `GaiaFrame.h`), un pour l'audio brut (framing `0x7E` + byte-stuffing +
  types `AudioData`/`AudioEnd`/`AudioAck`, voir `DualRfcommServers.h`).
- **Service SDP vendor** `39144315-32fa-40db-85ed-fbfeba2d86e6`
  (`VendorSdpRecord.h`) pointant sur le canal audio — c'est ce que
  HTCommander cherche pour reconnaître une radio compatible.
- **Appairage « Just Works »** (IO capability `NONE`, auto-confirmation SSP)
  et **MAC Bluetooth** alignée sur le préfixe de la vraie VR-N76
  (`38:D2:00:01:74:72`, dernier octet modifié).
- **Logique de commandes** (`BenshiCommandHandler.h`) : `GET_DEV_INFO`,
  `READ_SETTINGS`, `READ_RF_CH`, `GET_HT_STATUS`, `READ_STATUS`,
  `READ_REGION_NAME`, etc. Dispositions binaires alignées **exactement** sur
  les décodeurs de HTCommander (`RadioDevInfo` / `RadioChannelInfo` /
  `RadioSettings` / `RadioHtStatus` de son dépôt).
- **État modifiable et persistant** (`RadioState.h`) : HTCommander peut
  **éditer un canal mémoire** (`WRITE_RF_CH` : fréquences, CTCSS, nom,
  largeur de bande, TX interdit), **changer le canal VFO A / VFO B actif** et
  le mode double veille, le squelch, le scan (`WRITE_SETTINGS`), **changer la
  région** (`SET_REGION`) et **renommer une région** (`WRITE_REGION_NAME`).
  Les modifications sont stockées en NVS et **survivent au redémarrage**.
  Pour repartir des valeurs de `config.h` : `FACTORY_RESET_ON_BOOT` à `true`
  une fois, ou `pio run -t erase`.
- **Bit-packing MSB-first** générique (`BitStream.h`) pour les structures
  type `RfCh`/`Settings`/`DevInfo`.
- **Notifications non sollicitées** (`BenshiCommandHandler.h`) :
  `EVENT_NOTIFICATION / HT_STATUS_CHANGED` poussé sur le canal commande après
  chaque écriture, et pendant l'audio (`is_in_tx` à l'émission, `is_sq` /
  `is_in_rx` + RSSI en réception). Byte-pour-byte identique à ce qu'émet une
  vraie VR-N76. Respecte `REGISTER_NOTIFICATION`.
- **Interface poste réel** (`AudioBridge.h` + `SbcCodec.h`) : canal RFCOMM audio
  ↔ DAC / ADC internes + **sortie PTT** et **entrée squelch** pour piloter un
  émetteur-récepteur externe (ex. Quansheng UV-K1). Codec SBC réutilisé depuis
  `libbt.a`. Brochage kv4p-ht. Voir [`docs/AudioProtocol.md`](docs/AudioProtocol.md).

## Détection par HTCommander — d'après son code source

Le dépôt HTCommander (`src/windows/runner/bluetooth_classic_plugin.cpp`,
`src/linux/runner/bluetooth_classic_plugin.cc`) tranche les anciennes
incertitudes de ce projet :

### 1. Comment HTCommander reconnaît une radio « compatible »

**Uniquement** par la présence, dans les enregistrements SDP de l'appareil
appairé, du service vendor `39144315-32fa-40db-85ed-fbfeba2d86e6` (« BS AOC »).
Citation du code : *« Identify radios by their unique vendor SDP service UUID
rather than by name, which changes across rebrands / OS stacks »*. La MAC, le
nom Bluetooth et la Class of Device **ne sont pas regardés du tout** pour la
détection. Si ce service n'est pas publié, l'appareil est filtré et
n'apparaît jamais dans la liste (`if (compatible_only && service_uuids.empty()) continue;`).

### 2. Quel UUID porte quel canal

- **Commande / contrôle** : SPP standard `00001101-…` (`DoConnect` →
  `OpenRfcommSocket(btAddr, {kSppUuid})`).
- **Audio SBC** : service vendor `39144315-…`, repli sur Generic Audio
  `0x1203` (`DoConnectAudio` → `{kBsAocUuid, kGenericAudioUuid}`).

C'est **l'inverse** de l'hypothèse d'origine de ce projet — corrigé dans
`DualRfcommServers.h` (le canal commande est démarré en premier, l'audio en
second).

### 3. Comment ce projet publie le service vendor

`esp_spp_start_srv()` n'enregistre que des records SDP « SerialPort »
`0x1101`, et l'API publique `esp_sdp_api.h` (`esp_sdp_create_record`) n'est
compilée dans **aucune** lib Arduino-ESP32 précompilée (2.x comme 3.x :
`CONFIG_BT_SDP` désactivé). En revanche, les primitives bas niveau de la base
SDP de Bluedroid (`SDP_CreateRecord`, `SDP_AddAttribute`,
`SDP_AddProtocolList`, `SDP_AddUuidSequence`) **sont** présentes dans
`libbt.a` — c'est SPP lui-même qui s'en sert pour son `0x1101`.

[`src/VendorSdpRecord.h`](src/VendorSdpRecord.h) les appelle directement pour
créer un second enregistrement SDP portant l'UUID 128 bits
`39144315-32fa-40db-85ed-fbfeba2d86e6` et le numéro de canal RFCOMM audio
réel (celui rapporté par `ESP_SPP_START_EVT`). C'est exactement ce que fait
la fonction `add_raw_sdp()` de l'IDF récent, reproduit sans dépendre du
header interne non livré par le framework Arduino. Aucune migration
nécessaire : le projet reste sur `espressif32@6.11.0` (Arduino 2.0.17).

> ⚠️ **Après le flash, supprime l'appairage existant côté PC/téléphone et
> ré-appaire.** Windows et BlueZ mettent en cache les services SDP au moment
> de l'appairage ; sans ré-appairage, l'ancien cache (sans le service vendor)
> persiste et HTCommander continue de filtrer la radio.

Pour confirmer que le service est bien publié, depuis un Linux :

```bash
sdptool browse <MAC-de-l-ESP32>
# doit lister 39144315-32fa-40db-85ed-fbfeba2d86e6 avec un canal RFCOMM
```

### 4. Pairing / sécurité

`Bonded: yes`, `LegacyPairing: no` sur ta vraie radio indique un pairing SSP
(Secure Simple Pairing) moderne, pas un vieux code PIN. `DualRfcommServers.h`
utilise `ESP_SPP_SEC_NONE` et déclare une IO capability `NONE`
(`esp_bt_gap_set_security_param`), ce qui force le modèle d'association
**« Just Works »** : aucune saisie ni confirmation de code numérique côté
appareil. Le callback GAP auto-accepte `ESP_BT_GAP_CFM_REQ_EVT` et fournit un
PIN `0000` pour un éventuel appareil legacy.

Note : sur un **premier** appairage, l'OS hôte (Windows, GNOME…) peut encore
afficher une boîte « Autoriser l'appairage ? ». La vraie VR-N76 ne montre
rien parce qu'elle est **déjà appairée** (`Bonded: yes`) — une fois l'ESP32
appairé une première fois, il se comporte pareil.

**Découverte fiable au 1er scan** : `DualRfcommServers.h` appelle
`esp_bt_sleep_disable()` (plus de *modem-sleep* : la radio écoute en
permanence) et publie un **EIR complet** (`esp_bt_gap_config_eir_data` : nom
+ UUIDs + puissance TX dans la réponse d'inquiry). Sans ça, l'ESP32 s'endort
entre deux fenêtres de scan et il faut parfois relancer la découverte
plusieurs fois. Pense aussi à **éteindre la vraie VR-N76** pendant les tests
(même nom Bluetooth → confusion possible).

## Audio — interface entre HTCommander et un poste réel (ex. Quansheng UV-K1)

Le firmware peut servir d'**adaptateur Bluetooth** pour un vrai émetteur-récepteur :
HTCommander cause en Bluetooth avec le simulateur, qui relaie l'audio et le
PTT vers un poste câblé sur quelques broches.

- Le canal RFCOMM audio transporte du **SBC 32 kHz / 16 bits / mono** (8
  sous-bandes, 16 blocs, allocation *loudness*, bitpool 40 — confirmé depuis
  le code de HTCommander).
- **Codec SBC** : encodeur (`SBC_Encoder`) et décodeur (`OI_CODEC_SBC_DecodeFrame`)
  **déjà présents dans `libbt.a`** ; seuls les en-têtes sont vendorisés dans
  [`lib/sbc/`](lib/sbc/) — aucune dépendance ajoutée.
- **Audio matériel** : DAC et ADC **internes** de l'ESP32, pilotés par l'unique
  I2S0 en mode « built-in ADC + DAC » full-duplex (`AudioBridge.h`, calqué sur
  l'exemple ESP-IDF `i2s_adc_dac`).

### Brochage — aligné sur [kv4p-ht](https://github.com/VanceVagell/kv4p-ht)

Les broches reprennent celles du firmware kv4p-ht pour ESP32 WROOM-32
(`kv4p_ht_esp32_wroom_32/globals.h`), pour qu'une même carte / un même
adaptateur serve aux deux projets.

| Fonction | GPIO | kv4p-ht | Détail |
|---|---|---|---|
| **Audio → poste** (DAC) | **25** (+26) | `PIN_AUDIO_OUT` | Sortie du DAC interne vers l'**entrée micro** du poste. 0–3,3 V / 8 bits → prévoir un **atténuateur** vers le niveau micro. L'I2S pilote GPIO25 *et* GPIO26 ; utiliser GPIO25. |
| **Audio ← poste** (ADC) | **34** (`ADC1_CH6`) | `PIN_AUDIO_IN` | Entrée ADC1 depuis la **sortie HP** du poste. `AUDIO_ADC_CHANNEL` dans `config.h` (ADC1 uniquement — ADC2 en conflit radio). |
| **PTT → poste** (sortie) | **18** | `PIN_PTT` | Keye l'émission du poste dès que HTCommander envoie de l'audio ; relâché après `AUDIO_PTT_TAIL_MS`. **Actif à l'état bas** (comme kv4p-ht). |
| **Squelch ← poste** (entrée) | **32** | `PIN_SQ` | Signale que le poste reçoit → démarre la capture ADC vers HTCommander et renseigne `is_sq` / `is_in_rx` / RSSI. `-1` ⇒ détection **VOX** sur le signal ADC. Actif à l'état bas par défaut. |
| PTT local (option) | `-1` | `PIN_PHYS_PTT1/2` (5 / 33) | Force la capture ADC → HTCommander comme si le squelch était ouvert. |
| LED d'état (option) | `-1` | `PIN_LED` (2) | Fixe en émission, clignote en réception. |

Tout se règle dans la **section 5 de [`src/config.h`](src/config.h)**
(`AUDIO_PTT_GPIO`, `AUDIO_PTT_ACTIVE_LOW`, `AUDIO_PTT_TAIL_MS`, `AUDIO_SQ_GPIO`,
`AUDIO_SQ_ACTIVE_LOW`, `AUDIO_SQ_VOX_THRESH`, `AUDIO_ADC_CHANNEL`,
`AUDIO_SPK_VOLUME`, `AUDIO_MIC_GAIN`, `AUDIO_SBC_BITPOOL`…). `AUDIO_BRIDGE_ENABLE`
à `false` compile un firmware « commandes seules » (aucun accès I2S / DAC / ADC /
GPIO audio).

> `PIN_PD` (GPIO19) et l'UART RF (16/17) de kv4p-ht pilotent son module radio
> **interne** (SA818) : sans objet ici puisque le poste est externe. Ces broches
> restent libres.

### À valider sur matériel

Le mode ADC + DAC internes **simultané** de l'ESP32 (un seul I2S0) est
notoirement bruyant et sensible au format d'échantillon ; l'étage analogique
(atténuateur DAC → micro, adaptation HP → ADC, prise du signal squelch) est à
faire côté matériel. La chaîne SBC + framing + files d'attente + logique
PTT/squelch est en revanche indépendante du matériel. Détails et pistes de
réglage : [`docs/AudioProtocol.md`](docs/AudioProtocol.md).

## MAC Bluetooth

Ne clone pas la MAC réelle (`38:D2:00:01:74:71`) si les deux appareils
doivent pouvoir être allumés en même temps près du même PC/téléphone : ça
crée un conflit sur le bus radio et dans le cache d'appairage de l'OS. Le
pairing BR/EDR renégocie de toute façon une clé de liaison propre à chaque
essai, donc cloner la MAC n'est pas nécessaire. Supprime plutôt l'ancien
appairage de la vraie radio sur l'appareil qui doit se connecter à l'ESP32
(même nom Bluetooth "VR-N76" ⇒ risque de confusion sinon), puis appaire-toi
à l'ESP32 sous sa propre MAC (affichée au démarrage sur le port série).

## Build

```bash
pio run -t upload -t monitor
```

Nécessite une carte ESP32 **classique** (WROOM-32 / DevKitC / WROVER) : les
variantes S3/C3/C6 n'ont pas de Bluetooth Classic, donc pas de RFCOMM.

## Rappel

Une licence radioamateur est nécessaire pour émettre réellement sur les
fréquences amateur avec HTCommander/une vraie radio. Ce simulateur ne fait
transiter aucun signal RF — c'est un exercice d'interopérabilité Bluetooth
sur un protocole déjà documenté publiquement par les projets `benlink` et
`HTCommander`.
