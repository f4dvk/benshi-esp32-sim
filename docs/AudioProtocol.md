# Protocole audio Benshi (canal RFCOMM audio)

Notes de rétro-ingénierie du flux audio, tirées du **code source de
HTCommander** (`src/lib/radio/audio_engine.dart`, `src/lib/radio/radio_audio.dart`,
`src/lib/sbc/`). Sert de référence pour `src/AudioBridge.h` et `src/SbcCodec.h`.

## Transport

- Canal : le **2ᵉ** canal RFCOMM (service SDP vendor `39144315-…`), séparé du
  canal de commandes GaiaFrame. Voir `DualRfcommServers.h`.
- Chaque message audio est encadré par `0x7E … 0x7E`.
- Octet-stuffing à l'intérieur du cadre :
  - `0x7E` → `0x7D 0x5E`
  - `0x7D` → `0x7D 0x5D`
  - au décodage : après un `0x7D`, l'octet suivant est `XOR 0x20`.
- Premier octet après désescaping = **type de message** :

| Type   | Sens            | Signification                                            |
|--------|-----------------|---------------------------------------------------------|
| `0x00` | app → radio / radio → app | AudioData (trames SBC concaténées)             |
| `0x03` | radio → app     | AudioData (variante ; traitée comme `0x00`)             |
| `0x01` | app → radio / radio → app | AudioEnd (fin d'un train audio / relâché PTT) |
| `0x02` | —               | AudioAck                                                 |
| `0x09` | app → radio     | Écho de l'audio d'émission (métrologie, non rejoué)     |

`_endAudioFrame` observé côté HTCommander :
`7E 01 00 01 00 00 00 00 00 00 7E`.

## Codec

PCM de référence : **32 kHz, 16 bits, mono** (`_sampleRate = 32000`).

Paramètres SBC (`audio_engine.dart`, `_encoderFrame`) :

| Paramètre         | Valeur              |
|-------------------|---------------------|
| Fréquence         | 32 kHz (`freq32K`)  |
| Mode canal        | mono                |
| Blocs             | 16                  |
| Sous-bandes       | 8                   |
| Allocation        | loudness            |
| Bitpool           | 40                  |
| Sync word         | `0x9C` (SBC standard ; `0xAD` = mSBC, non utilisé ici) |

Conséquences :

- 1 trame SBC ⟷ `blocs × sous-bandes` = **128 échantillons PCM** = **256 octets** PCM.
- Longueur d'une trame SBC encodée ≈ `4 + ceil(4·8/8) + ceil(16·40/8)` = **88 octets**.
- HTCommander note que le firmware accepte un bitpool jusqu'à ~40 sans problème
  (« Confirmed working at 40 »).

## Implémentation dans ce projet

- **Décodage / encodage SBC** : `src/SbcCodec.h` (+ `SbcEncoderImpl.cpp`,
  `SbcDecoderImpl.cpp`). On réutilise le codec SBC **déjà présent dans
  `libbt.a`** (`SBC_Encoder`, `OI_CODEC_SBC_DecodeFrame`, …) ; seuls les
  en-têtes sont vendorisés dans `lib/sbc/`. Cf. `lib/sbc/README.md`.
- **Pont audio ↔ poste réel** : `src/AudioBridge.h`. Brochage aligné sur
  [kv4p-ht](https://github.com/VanceVagell/kv4p-ht) (voir le README).
  - **HTCommander → poste** (émission) : trames `AudioData` → `pushRadioSbc()`
    → décodage SBC → file PCM → I2S0 « built-in DAC » (GPIO25) → entrée micro
    du poste. En parallèle, la **sortie PTT (GPIO18, actif bas)** est activée
    tant que des trames arrivent, relâchée après `AUDIO_PTT_TAIL_MS` ou sur
    `AudioEnd`. `onTxState(true/false)` → `is_in_tx`.
  - **poste → HTCommander** (réception) : quand l'**entrée squelch (GPIO32)**
    indique un signal (ou la VOX sur le niveau ADC si `AUDIO_SQ_GPIO = -1`),
    l'ADC1 (GPIO34) est capturé → encodage SBC → `sendAudioData()` ;
    `sendAudioEnd()` à la fermeture du squelch. `onRxLevel(actif, rssi)` →
    `is_sq` / `is_in_rx` / RSSI.
  - **Half-duplex matériel** : sur l'ESP32, l'ADC et le DAC internes partagent
    l'unique I2S0 et ne peuvent pas tourner en même temps. `audio_pump`
    **réinstalle** le pilote I2S0 en entrée ADC (réception) ou en sortie DAC
    (émission) à chaque bascule de PTT — comme kv4p-ht. Une vraie radio est de
    toute façon half-duplex.
  - **Polarisation ADC** : GPIO34 est *entrée seule* (pas de pull interne) →
    l'ESP injecte VDD/2 sur GPIO26 (DAC2) et le montage doit relier GPIO26 →
    (résistance) → GPIO34, avec l'audio de la cible couplé en alternatif
    (condensateur) sur GPIO34. Câblage kv4p-ht.
  - **Contrôle de flux SPP** : `esp_spp_write` en mode callback n'accepte
    qu'une écriture à la fois → les trames audio sortantes sont mises en file
    (`audioTxQ_`, ~100 ms) et écrites au fil des `ESP_SPP_WRITE_EVT` /
    `ESP_SPP_CONG_EVT` ; débordement = on jette les plus vieilles.
  - Tâches FreeRTOS (core 1) : `audio_pump` (I2S temps réel + bascule de sens),
    `audio_rx` (décodage), `audio_tx` (encodage), `audio_ctl` (PTT / squelch /
    statut).
  - `AUDIO_DEBUG` (config.h §7) imprime chaque seconde l'état complet de la
    chaîne : handles RFCOMM, débits TX/RX, pertes, PTT, squelch, niveau ADC.
- Réglages dans `src/config.h`, section 5.

### Deux « cibles » derrière le pont audio

Le pont audio est identique dans les deux modes de démarrage (voir README) ;
seule la « cible » des broches change :

- **Mode SA818** (`src/Sa818.h`) : un module RF SA818/DRA818 a répondu à
  `AT+DMOCONNECT` au boot. `BenshiCommandHandler` retune le module
  (`AT+DMOSETGROUP`) à chaque changement de canal / VFO
  (`RadioState::activeRf()` décode fréquences / CTCSS / bande passante du canal
  actif). Le retune est **différé** : `process()` (contexte Bluetooth) pose un
  drapeau, `DualRfcommServers::poll()` (boucle Arduino) fait l'I/O UART.
- **Mode UV-K1** : aucun module. Fréquences purement simulées ; l'ESP ne peut
  que passer l'audio et piloter PTT / lire squelch d'un poste externe.

## Notifications de statut liées à l'audio

La vraie VR-N76 pousse des `EVENT_NOTIFICATION / HT_STATUS_CHANGED`
(`FF 01 00 05 00 02 00 09 01 XX XX XX XX`) pendant la réception : elle lève
`is_sq` + `is_in_rx` et envoie des mises à jour de RSSI, puis referme le
squelch en fin de trafic (capture d'une VR-N76 réelle, réception APRS).

Le simulateur reproduit ça :

- La tâche `audio_ctl` calcule un **RSSI 0..15** = `log2(|ADC|moyen)` (niveau du
  signal reçu par le poste) et appelle `onRxLevel(actif, rssi)` à chaque
  changement (RSSI limité à une mise à jour / 150 ms).
- `BenshiCommandHandler::setAudioRx()` met à jour `is_sq` / `is_in_rx` / `rssi`,
  `setAudioTx()` met à jour `is_in_tx`, et chacun émet `HT_STATUS_CHANGED`
  **si HTCommander s'est abonné** (`REGISTER_NOTIFICATION`, juste après
  `GET_DEV_INFO`).
- Le bit `is_aoc_connected` suit la connexion du canal RFCOMM audio.
- Les écritures (`WRITE_SETTINGS` / `WRITE_RF_CH` / `SET_REGION`) émettent aussi
  `HT_STATUS_CHANGED`, juste après leur réponse.

Concurrence : ces hooks tournent dans les tâches audio et lisent `RadioState`
pendant que la tâche Bluetooth peut l'écrire (`WRITE_*`). Lecture potentiellement
« déchirée » mais sans gravité — la notification suivante corrige la valeur.

## État (validé sur carte kv4p-ht v1 + SA818)

- Réception (SA818 → HTCommander) et émission (HTCommander → SA818) audio OK.
- ADC réel mesuré ≈ 31,7–31,9 kHz (≈ 32 000, écart négligeable).
- `SBC_Encoder_Init()` de Bluedroid **recalcule le bitpool depuis `u16BitRate`**
  pour un flux mono → il faut fournir le bon bitrate (176 kbps ici pour
  bitpool 40) puis re-forcer `s16BitPool`. Sinon trames SBC vides de ~9 octets.
- `esp_spp_write` en mode callback : **une seule écriture en vol** (attendre
  `ESP_SPP_WRITE_EVT`), et regrouper plusieurs trames SBC par écriture sinon
  RFCOMM sature (`ESP_SPP_CONG_EVT`).
- Bascule ADC↔DAC : le mutex récursif `adc1_i2s_lock` doit être pris **et**
  rendu par la même tâche → tout l'I2S est géré depuis `audio_pump`.
- **Voix** : OK. **APRS / données** : le pin SQ du SA818 s'ouvre *après* le
  préambule AX.25 et clignote → sans traitement, HTCommander perd la synchro.
  Atténuations : `AUDIO_RX_PREROLL_MS` (on pousse l'audio capté *avant*
  l'ouverture du squelch), `AUDIO_SQ_HANG_MS` (traîne), `AUDIO_RX_ALWAYS`
  (ignore le pin SQ, stream continu). Le paramètre squelch de `AT+DMOSETGROUP`
  n'agit **pas** sur le pin SQ (qui suit la détection de porteuse).

### Pistes d'amélioration

- Émission via DAC 8 bits → bruit de quantification. `AUDIO_DAC_LOWPASS` (filtre
  logiciel 1 pôle) ou sortie PDM + filtre RC (comme kv4p-ht) pour faire mieux.
- Le type de trame audio *venant* de la radio est `0x00` (confirmé : HTCommander
  décode `0x00` et `0x03` de la même façon).
## Canal données / TNC AX.25 (APRS fiable)

Quand la radio est sur le canal nommé `TNC_CHANNEL_NAME` (« APRS »), l'ESP fait
**TNC matériel** au lieu de passer l'APRS par l'audio SBC :

- **TX** : `HT_SEND_DATA` (cmd 31, corps `[flags][ax25…][chanId?]`, flags =
  bit7 final · bit6 hasChanId · bits[5:0] fragId) → réassemblage →
  `AfskModulator` → PCM → DAC → PTT poste. On accuse chaque fragment
  (`{is_reply, cmd 31, [0x00]}`) pour que HTCommander envoie le suivant.
- **RX** : ADC → `AfskDemodulator` (bandpass → I/Q → FM demod → PLL → NRZI →
  HDLC → CRC) → trame AX.25 (FCS vérifié + retiré) → **`RX_DATA` (cmd 57)**
  non sollicité, fragmenté à 180 o : `[0x00][flags][ax25][chanId]`.
- Modem : `dkaukov/esp32-afsk` (mêmes libs que kv4p-ht, **GPL-3**), `esp-dsp`
  déjà fourni par arduino-esp32. `-DAFSK_SAMPLE_RATE=32000`.
- Hors canal APRS : `dataChanActive_` = false → pont audio en phonie SBC normal.
- Tâches : `tnc_rx` (démod), `tnc_tx` (modulation, bloquante = temps réel).
- **Allocation différée** : le modem, les files et les tâches ne sont créés
  (`tncStart`) qu'une fois HTCommander connecté ET sur le canal APRS — le heap
  est serré (Bluedroid), il faut que la connexion BT ait sa part d'abord.
  Libérés (`tncStop`) en quittant le canal ou à la déconnexion. Sur échec
  d'allocation : on reste en phonie (dégradé mais connecté). Trace `heap=` dans
  `[DBG]`, `[TNC] actif (heap libre N)`.
- Pas de FEC (FX.25). Comme le chemin TNC matériel de HTCommander lui-même.

Côté HTCommander : **mettre le modem logiciel APRS sur « Off »** (sinon il
module/démodule lui-même via l'audio au lieu d'utiliser notre TNC). Le modem
matériel (`hardwareModemEnabled`) est actif par défaut.

Non validé sur matériel au moment de l'écriture.
