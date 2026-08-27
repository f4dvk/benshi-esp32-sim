# lib/sbc — en-têtes du codec SBC de Bluedroid

Ce dossier ne contient **que des en-têtes**. Aucun fichier `.c` de codec n'est
compilé ici.

## Pourquoi

Le firmware a besoin d'encoder et de décoder du SBC (le codec du canal audio
Benshi). La pile Bluetooth `libbt.a` fournie par arduino-esp32 **contient déjà**
l'encodeur et le décodeur SBC complets (ils servent à l'A2DP et au HFP) :

| Fonction                     | Objet dans `libbt.a`   |
|------------------------------|------------------------|
| `SBC_Encoder_Init`           | `sbc_encoder.c.obj`    |
| `SBC_Encoder`                | `sbc_encoder.c.obj`    |
| `OI_CODEC_SBC_DecoderReset`  | `decoder-sbc.c.obj`    |
| `OI_CODEC_SBC_DecodeFrame`   | `decoder-sbc.c.obj`    |

Seuls les **en-têtes** correspondants ne sont pas livrés dans le SDK Arduino.
On les copie donc ici pour pouvoir appeler ces symboles. C'est la même approche
que `src/VendorSdpRecord.h` (qui appelle les primitives `SDP_*` de `libbt.a`).

## Provenance

Copié depuis `esp-idf/components/bt/host/bluedroid/external/sbc/` :

- `decoder/include/*.h`  → décodeur OI
- `encoder/include/sbc_encoder.h`
- `encoder/include/sbc_types.h` → **modifié** : l'original inclut
  `stack/bt_types.h` (indisponible côté Arduino) ; on déclare directement les
  quelques alias `UINT8/16/32` nécessaires.

Le codec SBC de Bluedroid est un import figé (Broadcom / Android, licence
Apache-2.0). Sa disposition mémoire et ses prototypes sont identiques entre
IDF 4.4 (arduino-esp32 3.2.x) et les versions suivantes ; en particulier
`OI_CODEC_SBC_DecoderReset` prend bien 7 paramètres (`..., enhanced,
msbc_enable`), ce qui a été vérifié sur `libbt.a` (absence du symbole
`OI_CODEC_mSBC_DecoderReset`, signe de la variante fusionnée).
