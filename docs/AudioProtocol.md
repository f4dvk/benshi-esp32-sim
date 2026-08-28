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
  - Half-duplex logique : la capture ADC → HTCommander est inhibée pendant
    l'émission vers le poste.
  - Tâches FreeRTOS (core 1) : `audio_pump` (I2S temps réel), `audio_rx`
    (décodage), `audio_tx` (encodage), `audio_ctl` (PTT / squelch / statut).
- Réglages dans `src/config.h`, section 5.

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

## Reste à valider sur matériel

- Le mode **ADC + DAC internes simultané** de l'ESP32 (un seul I2S0) est
  bruyant et sensible au format d'échantillon (`I2S_CHANNEL_FMT_*`,
  décalage 8 bits du DAC, masquage 12 bits de l'ADC). Code calqué sur
  l'exemple ESP-IDF `i2s_adc_dac`, à ajuster à l'oreille.
- Sortie DAC 8 bits → **ampli externe indispensable** (PAM8302 / LM386).
- Entrée ADC → **préampli micro + polarisation ~1,65 V** indispensables.
- Vérifier le type de trame (`0x00` vs `0x03`) réellement attendu par
  HTCommander pour l'audio *venant* de la radio simulée.
