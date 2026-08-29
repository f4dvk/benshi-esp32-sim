# Interface Nextion — NX4827T043 (480 × 272)

Ce dossier décrit l'interface à construire dans le **Nextion Editor** pour le
pilote `DISPLAY_DRIVER_NEXTION` (voir [`../src/NextionDisplay.h`](../src/NextionDisplay.h)).

> Le format `.HMI` du Nextion Editor est un binaire propriétaire non
> documenté : il ne peut pas être généré hors de l'éditeur. Ce dossier
> fournit donc **le plan exact** (objets, coordonnées, couleurs) pour la
> reconstruire en ~15 min, plus une maquette visuelle
> ([`mockup.svg`](mockup.svg)).

L'ESP **n'envoie que des mises à jour** (`objet.txt=…`, `objet.val=…`,
`objet.pco=…`) et le spectre en mode transparent (`addt`). Il ne crée aucun
objet et ne lit pas le tactile → l'affichage est **passif**.

---

## 1. Nouveau projet

| Réglage | Valeur |
|---|---|
| Modèle | **NX4827T043** (Basic) — ou le T/K correspondant à ton écran |
| Orientation | **Horizontal (90°)** → 480 × 272 |
| Encodage | iso-8859-1 (ou utf-8) |

**Debug / Program.s** — forcer le débit UART (le firmware tente aussi de le
faire au démarrage, mais autant le figer) :

```
bauds=115200
```

## 2. Polices (Tools ▸ Font Generator)

| Nom | Hauteur | Usage |
|---|---|---|
| `font0` | ~22 px | en-tête, mode, S, UTC, SQ/PWR/BT/GPS |
| `font1` | ~34 px | bloc statut (RX / TX …) |
| `font2` | ~72 px | fréquence |

(Les numéros de police sont libres, ils sont fixés sur chaque composant dans
l'éditeur ; le firmware ne s'en occupe pas.)

## 3. Page `page0`

Fond : **noir** (`0`). Ajouter les objets ci-dessous (les **noms doivent être
exacts**, ce sont ceux qu'utilise le firmware). `xcen`/`ycen` = alignement
centré ; `pco` = couleur du texte (RGB565).

### En-tête (y 4 – 30, `font0`)

| objtype | objname | x | y | w | h | pco | align | texte initial |
|---|---|---|---|---|---|---|---|---|
| Text | `tChan` | 6 | 4 | 150 | 26 | 64800 (ambre) | gauche | `M00 ---` |
| Text | `tSq`   | 168 | 4 | 34 | 26 | 2016 (vert) | centre | *(vide)* |
| Text | `tPwr`  | 208 | 4 | 44 | 26 | 64800 | centre | `HI` |
| Text | `tBt`   | 300 | 4 | 40 | 26 | 33808 (gris) | centre | `BT` |
| Text | `tGps`  | 344 | 4 | 130 | 26 | 33808 | droite | `GPS --` |

### Fréquence (y 38 – 126)

| objtype | objname | x | y | w | h | font | pco | align | texte |
|---|---|---|---|---|---|---|---|---|---|
| Text | `tFreq` | 6 | 38 | 410 | 84 | `font2` | 65535 (blanc) | gauche | `000.0000` |
| Text | *(statique)* `pMHz` | 420 | 92 | 54 | 26 | `font0` | 33808 | gauche | `MHz` |

`pMHz` est un simple libellé fixe — nom sans importance.

### Mode + heure UTC (y 132 – 158, `font0`)

| objtype | objname | x | y | w | h | pco | align | texte |
|---|---|---|---|---|---|---|---|---|
| Text | `tMode` | 6 | 132 | 220 | 24 | 41471 (bleu clair) | gauche | `FM  ---` |
| Text | `tUtc`  | 240 | 132 | 234 | 24 | 2016 | droite | `UTC --:--:--` |

### S-mètre (y 164 – 190)

| objtype | objname | x | y | w | h | notes |
|---|---|---|---|---|---|---|
| Text | *(statique)* `pS` | 6 | 164 | 18 | 24 | `font0`, `S` |
| Progress Bar | `jSig` | 28 | 168 | 444 | 16 | `bco` 33808 (fond), `pco` 2016 (barre), `val` 0 |

### Séparateur

| objtype | objname | x | y | w | h | bco |
|---|---|---|---|---|---|---|
| Crop/Line | *(statique)* | 6 | 196 | 468 | 2 | 1023 (cyan foncé) |

### Bloc statut + spectre (y 200 – 268)

| objtype | objname | x | y | w | h | font | align | texte |
|---|---|---|---|---|---|---|---|---|
| Text | `tStat` | 6 | 204 | 180 | 60 | `font1` | centre / centre | `STBY` |
| Waveform | `sSpec` | 200 | 202 | **272** | **64** | — | — | 1 canal (ch0), `gdc`/`gdw` grille au goût |

**Après avoir créé `sSpec`, relève son `id`** dans le panneau d'attributs
(colonne de droite) et reporte-le dans `config.h` :

```c
#define NEXTION_WAVE_ID   <id de sSpec>
#define NEXTION_WAVE_W    272     // = largeur de sSpec
#define NEXTION_WAVE_H    64      // = hauteur de sSpec
```

Si tu changes la taille de `sSpec`, mets `NEXTION_WAVE_W` / `_H` en accord
(le firmware envoie `NEXTION_WAVE_W` points, chacun à l'échelle `0..NEXTION_WAVE_H-1`).

## 4. Compilation / flash

`Compile` puis `File ▸ TFT file output` → copier le `.tft` sur une microSD
(FAT32, fichier seul à la racine), insérer, mettre sous tension : l'écran se
met à jour puis redémarre. (Ou `Upload` via un adaptateur USB-TTL.)

---

## Champs pilotés par le firmware

| Objet | Commande envoyée | Quand |
|---|---|---|
| `tFreq` | `tFreq.txt="145.5000"` + `tFreq.pco=` (blanc / rouge en TX) | au changement |
| `tChan` | `tChan.txt="M04 APRS"` | au changement |
| `tMode` | `tMode.txt="FM  WIDE 25k"` | au changement |
| `tStat` | `tStat.txt="RX"` + `tStat.pco=` | au changement |
| `tSq` | `tSq.txt="SQ"` / `""` | au changement |
| `tPwr` | `tPwr.txt="HI"/"LO"` + `.pco` | au changement |
| `tBt` | `tBt.pco=` (cyan connecté / gris sinon) | au changement |
| `tGps` | `tGps.txt="GPS 3D 08"` + `.pco` | au changement |
| `tUtc` | `tUtc.txt="UTC 12:34:56"` | ~1×/s |
| `jSig` | `jSig.val=NN` (0..100) | au changement |
| `sSpec` | `addt <id>,0,272` + 272 octets | toutes les `NEXTION_SPECTRUM_MS` |

### Couleurs (RGB565)

| Rôle | Hex | Décimal |
|---|---|---|
| rouge (TX) | `0xF800` | 63488 |
| vert (RX / OK) | `0x07E0` | 2016 |
| ambre (puissance) | `0xFD20` | 64800 |
| cyan (BT actif) | `0x07FF` | 2047 |
| gris (inactif) | `0x8410` | 33808 |
| blanc (VFO) | `0xFFFF` | 65535 |
| bleu clair (mode) | `0xA1FF` ≈ | 41471 |
