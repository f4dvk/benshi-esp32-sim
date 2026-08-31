# Firmware UV‑K1 / UV‑K5 V3 — « mode hôte » pour benshi‑esp32‑sim

Firmware **F4HWN Edition Fusion** modifié pour les Quansheng **UV‑K1** et
**UV‑K5 V3**, avec un **mode hôte** : sur commande série, le poste **suspend sa
boucle radio** et laisse un contrôleur externe (l'ESP32 de `benshi-esp32-sim`)
piloter le BK4819 via des commandes haut niveau — fréquences RX/TX, double VFO,
mémoires, modulation, bande, CTCSS/DCS, puissance, PTT, S‑mètre.

But : remplacer le module SA818 par un vrai poste Quansheng, piloté depuis
HTCommander en Bluetooth, avec beaucoup plus de contrôle. L'audio reste
analogique (HP du poste → ADC ESP32, DAC ESP32 → micro du poste).

> Ce n'est **pas** le firmware de l'ESP32. C'est le firmware du poste Quansheng.
> Le pilote côté ESP32 est `src/UvK5Link.h` / `src/UvK5.h`.

L'option `ENABLE_UART_RW_BK_REGS` (lecture/écriture registre BK4819, `0x0601` /
`0x0602`) est **aussi** activée : le même `.bin` sert au test de sanité de la
liaison série côté ESP32 (Phase 1).

---

## ⚠️ Compatibilité — UV‑K1 et UV‑K5 V3 uniquement

| Poste | MCU | Ce firmware |
|---|---|---|
| **UV‑K1** | PY32F071 | ✅ oui |
| **UV‑K5 V3** | PY32F071 | ✅ oui |
| UV‑K5 / UV‑K5(8) / K6 / 5R (v1/v2) | DP32G030 | ❌ **non** — mauvais MCU |

Pour les UV‑K5 « v1/v2 » (DP32G030), le mode hôte n'est pas porté ; voir
[`quansheng-dock-fw`](https://github.com/nicsure/quansheng-dock-fw).

---

## Contenu

```
firmware/uv-k1-k5v3/
├── README.md              ce fichier
├── LICENSE                Apache License 2.0 (amont)
├── NOTICE                 attributions amont + fichiers modifiés
├── build.sh               clone l'amont au tag voulu, applique le patch, compile
├── patch/
│   ├── host.c host.h      le mode hôte (fichiers neufs)
│   └── integration.md     les insertions dans le source amont (revue / rebase)
├── tools/
│   └── uvk5_host.py       test PC du mode hôte (pyserial)
└── bin/
    └── uv-k1-k5v3-f4hwn-fusion-hostmode-v<amont>.H<rev>-<commit>.bin
```

Nom de fichier : `v<version amont>.H<HOST_PROTO_VER>-<commit court>` — p. ex.
`…-hostmode-v5.9.0.H21-435192e.bin`. Le `.H<n>` apparaît aussi **à l'écran de la
radio** et dans la réponse série. Publier le SHA‑256 à côté.

---

## Différence avec le firmware amont

Modification du **source** (voir [`patch/integration.md`](patch/integration.md)) :

- 2 fichiers neufs : `App/app/host.c`, `App/app/host.h`.
- 4 insertions dans `App/app/app.c` (suspension de la boucle radio quand le mode
  hôte est actif + watchdog de sortie).
- 3 dans `App/app/uart.c` (include, `SendReply` non‑static, 7 `case` de dispatch).
- 1 ligne dans `App/CMakeLists.txt`, 1 dans `CMakePresets.json`.

Options de build : `-DENABLE_HOST_MODE=ON -DENABLE_UART_RW_BK_REGS=ON
-DENABLE_FEAT_F4HWN_K5VIEWER=OFF -DENABLE_FEAT_F4HWN_RXTX_LOG_K5VIEWER=OFF
-DVERSION_STRING_2=v<amont>.H<rev>`. K5Viewer est désactivé : son parseur
(`VCP_K5ViewerPing`) tourne sur le même port série et injecte des touches à
partir d'octets arbitraires → conflit avec nos trames. Inutile ici.

Cible validée : **v5.9.0** (`435192e`), preset `Fusion`, `arm-none-eabi-gcc`
13.2/13.3 — compile sans warning (hors warnings amont). **FLASH ≈ 92 %**.

---

## (Re)compiler

```bash
./build.sh                 # défaut : tag v5.9.0, preset Fusion
./build.sh v5.9.0 Custom    # autre preset
```

`build.sh` : clone l'amont, copie `patch/host.*`, applique les insertions
(perl, ancrages sur chaîne stable, échec fatal si un ancrage ne matche pas ;
`HOST_PROTO_VER` de `host.h` -> suffixe de version `.H<rev>`), puis
`./compile-with-docker.sh`. Résultat renommé dans `bin/` + SHA‑256.

Sans Docker : `arm-none-eabi-gcc` + `cmake` + `ninja` suffisent — mêmes
options `-D…` que ci-dessus, `cmake --preset Fusion …` puis
`cmake --build build/Fusion`.

---

## Flasher le poste

**UV‑K1 / UV‑K5 V3** : port **USB‑C natif** (CDC → `/dev/ttyACM0`), pas de
prise série jack.

1. Radio allumée, USB‑C relié au PC.
2. Flasher avec **[UV Studio](https://armel.github.io/uvstudio/)** (l'outil
   d'armel pour ces modèles) ou `k5prog`. Charger le `.bin` de `bin/`.
3. Au reboot, la radio fonctionne **normalement** tant qu'aucune commande
   `HOST_MODE` n'est reçue.

**Vérifier que c'est bien CE firmware** (et pas le F4HWN stock de même version) :

- **à l'écran** : la version affichée est `F4HWN v5.9.0.H21` (suffixe `.H<n>` =
  révision du patch mode hôte = `HOST_PROTO_VER`). Le stock affiche `F4HWN v5.9.0`.
- **par le port série** : `tools/uvk5_host.py <port> check` — seul ce firmware
  répond à `0x0630` ; il renvoie `proto H<n>` + la chaîne de version.

**Récupération** : le bootloader reste intact — refaire l'étape 1 et reflasher.

---

## Tester le mode hôte depuis un PC

```bash
pip install pyserial
tools/uvk5_host.py /dev/ttyACM0 probe                       # liaison + version firmware
tools/uvk5_host.py /dev/ttyACM0 check                       # confirme "firmware mode hôte" (0x0630)
tools/uvk5_host.py /dev/ttyACM0 setvfo --rx 145.500 --tx 145.500 --ctcss 88.5
tools/uvk5_host.py /dev/ttyACM0 status --loop
tools/uvk5_host.py /dev/ttyACM0 ptt on   ;  sleep 2  ;  tools/uvk5_host.py /dev/ttyACM0 ptt off
tools/uvk5_host.py /dev/ttyACM0 exit                        # ou : rien pendant ~2 min (watchdog)
```

`setvfo` / `ptt` / `monitor` / `recall` **entrent en mode hôte automatiquement** :
pas besoin d'un `enter` préalable. `status` reste passif (ne prend pas le
contrôle). `enter` explicite reste utile pour figer la radio sans autre action.

**Audio en mode hôte** — la boucle de squelch du firmware est suspendue, `host.c`
la remplace par un mini‑service (`HOST_Tick10ms`) :

- `setvfo --squelch 0` (défaut) : **AF ouvert en permanence** (données / APRS,
  comme un SA818 squelch=0). `monitor off` coupe, `monitor on` / `setvfo` rouvre.
- `setvfo --squelch 1..9` : **squelch matériel + gating CTCSS/CDCSS RX** — le HP
  ne s'ouvre que sur un signal (et le bon ton/code si configuré). `monitor on`
  force l'ouverture.

Dans les deux cas, `status` renvoie le bit **SIG** (signal reçu présent) →
l'ESP32 le mappe sur `is_sq` du HT_STATUS.

---

## Protocole (résumé)

Trame Quansheng : `AB CD | taille(LE) | charge masquée (XOR) | pad | DC BA`.
Charge interne = `[ID:u16 LE][Size:u16 LE][données]`, CRC‑16/XMODEM sur la charge
interne (commande seulement ; pas de CRC en réponse).

| ID | Requête | Réponse |
|---|---|---|
| `0x0630` HOST_MODE | `{u8 enter}` | `{u8 actif, u8 proto, char ver[16]}` |
| `0x0631` SET_VFO | `{u8 vfo, u32 rxF, u32 txF, u8 mod, u8 bw, u8 pwr, u8 rxCT, u8 rxCode, u8 txCT, u8 txCode, u16 step, u8 squelch}` (fréq en 10 Hz ; squelch 0 = AF permanent, 1‑9 = squelch + gating CTCSS/CDCSS RX) | `{u8 ok}` |
| `0x0632` SET_RADIO | `{u8 txVfo, u8 dualWatch, u8 crossband, u8 txLock}` | `{u8 ok}` |
| `0x0633` PTT | `{u8 on}` | `{u8 ok}` |
| `0x0634` GET_STATUS | — | `{u8 func, u16 rssi, u8 noise, u8 glitch, i16 dBm, u8 ctcType, u16 batt_mV, u8 flags}` — flags b0 hôte, b1 TX, b2 monitor, b3 SIG (signal reçu) |
| `0x0635` MONITOR | `{u8 on}` — 1 = AF ouvert (défaut), 0 = coupe le HP | `{u8 ok}` |
| `0x0636` RECALL_CH | `{u8 vfo, u16 ch}` | `{u8 ok}` |
| `0x0601` / `0x0602` | R/W registre BK4819 | (test Phase 1) |

**Sécurité** : si aucune commande hôte n'arrive pendant ~2 min, le firmware
**quitte** le mode hôte tout seul (`HOST_Tick500ms`) et rend la main à la radio.
`HOST_MODE{0}` le fait immédiatement. Le PTT physique est ignoré tant que le
mode hôte est actif. Une commande d'action reçue après une sortie auto ré‑entre
simplement en mode hôte.

---

## Licence & attributions

Firmware sous **Apache License 2.0**. Voir [`LICENSE`](LICENSE) et
[`NOTICE`](NOTICE) (à conserver dans toute redistribution). Les fichiers
modifiés portent une notice de modification (exigence Apache 2.0 §4).

Travaux amont : Dual Tachyon, OneOfEleven, fagci, Egzumer, muzkr, Armel / F4HWN.
L'attribution n'implique aucun soutien (endorsement) de leur part.

**Aucune garantie.** Flasher un poste radio se fait à vos risques. Respecter la
réglementation radioamateur de votre pays (bandes, puissance, identification).
