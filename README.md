# Simulateur de radio Benshi (VR-N76) sur ESP32 (pour HTCommander)

> ⚠️ **PROJET EN COURS DE DÉVELOPPEMENT.**
>
> Ce qui fonctionne : détection par HTCommander, appairage, dialogue de
> commande/contrôle (infos radio, réglages, statut), lecture **et édition**
> des canaux mémoire avec persistance en mémoire flash (NVS), sélection du
> VFO / canal actif, région.
>
> Audio + modes : **validé sur matériel** (carte kv4p-ht v1 + module SA818).
> Réception ET émission audio (codec SBC 32 kHz de `libbt.a`), **sortie PTT /
> entrée squelch**, **détection SA818 au boot** (mode module RF réel vs mode
> passerelle poste externe — brochage
> [kv4p-ht](https://github.com/VanceVagell/kv4p-ht)), pilotage du SA818
> (fréquence, largeur, CTCSS, squelch, filtres, puissance), notifications de
> statut (`HT_STATUS_CHANGED`). L'émission via le DAC 8 bits interne peut
> présenter un peu de bruit de quantification (filtre `AUDIO_DAC_LOWPASS` en
> option).
>
> **TNC AX.25 / APRS** (canal données) : implémenté (`HT_SEND_DATA` / `RX_DATA`
> + modem AFSK 1200 `dkaukov/esp32-afsk`), actif uniquement sur le canal nommé
> « APRS ». **Non validé sur matériel.** ⚠️ ce module rend le firmware **GPL-3**.
>
> Ce qui **ne fonctionne pas encore** : le mode VFO fréquence libre.
>
> Utilisez-le pour l'interopérabilité Bluetooth / le test de HTCommander,
> pas comme une vraie radio. Retours et contributions bienvenus.

## Deux modes de fonctionnement (choisis au démarrage)

Au boot, l'ESP tente un handshake `AT+DMOCONNECT` sur l'UART RF
(GPIO16/17). Selon le résultat :

| | **Mode SA818** | **Mode UV-K1** (historique) |
|---|---|---|
| Condition | un module **SA818 / DRA818** répond (≤ `RF_MODULE_PROBES` essais, 3 par défaut) | aucun module détecté |
| Rôle de l'ESP | **pilote un vrai module RF** : les canaux de `config.h` sont des fréquences réelles ; changer de canal / VFO dans HTCommander **retune** le module (`AT+DMOSETGROUP`) | **simule** entièrement la radio (fréquences, canaux, statut) et sert de **passerelle audio/PTT/squelch** vers un poste externe non pilotable |
| Audio / PTT / squelch | vers le module SA818 | vers le poste externe (ex. Quansheng UV-K1) |
| Interface HTCommander | **identique** (Bluetooth Classic, protocole Benshi) | **identique** |

Le brochage suit celui de [kv4p-ht](https://github.com/VanceVagell/kv4p-ht)
pour que la même carte serve aux deux usages (`RF_MODULE_ENABLE = false`
force le mode UV-K1 sans sonder l'UART).

> Le **mode SA818** est validé sur matériel. Le **mode UV-K1** partage la même
> chaîne audio/PTT/squelch mais n'a pas encore été testé bout à bout avec un
> poste externe.

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
- **Interface radio** (`AudioBridge.h` + `SbcCodec.h`) : canal RFCOMM audio ↔
  DAC / ADC internes + **sortie PTT** et **entrée squelch**. Codec SBC réutilisé
  depuis `libbt.a`. Brochage kv4p-ht. Voir [`docs/AudioProtocol.md`](docs/AudioProtocol.md).
- **Détection de mode au boot** (`Sa818.h`, `main.cpp`) : handshake `AT+DMOCONNECT`
  sur l'UART RF → **mode SA818** (pilotage d'un module RF réel : retune sur
  changement de canal / VFO via `AT+DMOSETGROUP`, volume, filtres) ou **mode
  UV-K1** (simulation + passerelle vers poste externe). Pilote SA818/DRA818
  autonome, sans dépendance ajoutée.

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

« Cible » = le module SA818 en **mode SA818**, ou le poste externe en **mode UV-K1**.

| Fonction | GPIO | kv4p-ht | Détail |
|---|---|---|---|
| **Audio → cible** (DAC) | **25** (+26) | `PIN_AUDIO_OUT` | Sortie du DAC interne vers l'**entrée micro** de la cible. 0–3,3 V / 8 bits → prévoir un **atténuateur**. L'I2S pilote GPIO25 *et* GPIO26 ; utiliser GPIO25. |
| **Audio ← cible** (ADC) | **34** (`ADC1_CH6`) | `PIN_AUDIO_IN` | Entrée ADC1 depuis la **sortie HP** de la cible. `AUDIO_ADC_CHANNEL` dans `config.h` (ADC1 uniquement — ADC2 en conflit radio). |
| **PTT → cible** (sortie) | **18** | `PIN_PTT` | Keye l'émission dès que HTCommander envoie de l'audio ; relâché après `AUDIO_PTT_TAIL_MS`. **Actif à l'état bas** (comme kv4p-ht). |
| **Squelch ← cible** (entrée) | **32** | `PIN_SQ` | Signale une réception → démarre la capture ADC vers HTCommander, renseigne `is_sq` / `is_in_rx` / RSSI. `-1` ⇒ **VOX** sur le signal ADC. Actif à l'état bas par défaut. |
| **UART module RF** | **16 / 17** | `PIN_RF_RXD` / `PIN_RF_TXD` | `Serial2` @ 9600. Sondé au boot ; si un SA818/DRA818 répond → **mode SA818**. Libre en mode UV-K1. |
| **PD module RF** | **19** | `PIN_PD` | Alimentation du module SA818 (HIGH = allumé). `-1` si non câblé. Inutile en mode UV-K1. |
| PTT local (option) | `-1` | `PIN_PHYS_PTT1/2` (5 / 33) | Force la capture ADC → HTCommander comme si le squelch était ouvert. |
| LED d'état (option) | `-1` | `PIN_LED` (2) | Fixe en émission, clignote en réception. |

Réglages : **section 5** de [`src/config.h`](src/config.h) (audio : `AUDIO_PTT_*`,
`AUDIO_SQ_*`, `AUDIO_ADC_CHANNEL`, `AUDIO_SPK_VOLUME`, `AUDIO_MIC_GAIN`,
`AUDIO_SBC_BITPOOL`, `AUDIO_BRIDGE_ENABLE`…) et **section 6** (module RF :
`RF_MODULE_ENABLE`, `RF_MODULE_UART_RX/TX`, `RF_MODULE_PD_GPIO`,
`RF_MODULE_PROBES`, `RF_MODULE_VOLUME`, `RF_MODULE_SQUELCH`).

> Sur ESP32-**WROVER**, GPIO16/17 sont pris par la PSRAM : câbler l'UART RF
> ailleurs (ou utiliser un WROOM-32 / DevKitC).

### Notes matériel

- L'ADC et le DAC internes de l'ESP32 partagent l'unique I2S0 : le pont bascule
  I2S0 entre entrée ADC (réception) et sortie DAC (émission) à chaque PTT —
  **half-duplex**, comme une vraie radio.
- GPIO34 est une **entrée seule** (pas de pull interne) : la polarisation
  externe est obligatoire (GPIO26 → résistance → GPIO34, audio couplé par
  condensateur). Câblage kv4p-ht.
- L'émission utilise le **DAC 8 bits** interne (≈ 48 dB de plancher de bruit) :
  un peu de bruit de quantification est normal. Options : `AUDIO_DAC_LOWPASS`
  (filtre logiciel), ou soigner le filtre RC de la carte.
- En émission réelle, le SA818 tire des pics de courant : **l'alimenter
  séparément** (masse commune), sinon l'ESP peut redémarrer.

Détails du protocole et de l'implémentation :
[`docs/AudioProtocol.md`](docs/AudioProtocol.md).

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
