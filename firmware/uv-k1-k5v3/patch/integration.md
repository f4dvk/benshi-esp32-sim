# Mode hôte — modifications du firmware amont

`build.sh` applique ceci automatiquement (perl, ancrages sur chaîne stable) sur
un clone neuf de `armel/uv-k1-k5v3-firmware-custom` au tag choisi. Ce document
sert à la **revue** et à l'**application manuelle** / au rebase.

Cible validée : tag **v5.9.0** (commit `435192e`). Compile proprement
(`arm-none-eabi-gcc` 13.2 / 13.3, preset `Fusion`, `-Os`), sans warning sur les
fichiers touchés. FLASH ≈ 94 % avec le preset Fusion — si un rebase fait
déborder, compiler le mode hôte avec un preset plus léger (`Custom`).

## Fichiers ajoutés

- `App/app/host.h`  ← `patch/host.h`
- `App/app/host.c`  ← `patch/host.c`

## `App/app/app.c` (4 insertions, indentation = 4 espaces)

1. Après `#include "app/app.h"` :
   ```c
   #ifdef ENABLE_HOST_MODE
   #include "app/host.h"
   #endif
   ```

2. Dans `void APP_Update(void)`, **juste après** le bloc
   `#ifdef ENABLE_USB … UART_HandleCommand(UART_PORT_VCP) … #endif`
   (si `ENABLE_USB` absent : en tête de fonction) :
   ```c
   #ifdef ENABLE_HOST_MODE
       if (HOST_IsActive())
           return;
   #endif
   ```
   Placé après le service VCP pour qu'en mode hôte les commandes USB
   (dont `HOST_MODE{0}`) restent traitées, la boucle radio étant coupée juste
   après.

3. Dans `APP_TimeSlice10ms()`, juste après le bloc
   `#ifdef ENABLE_UART … UART_HandleCommand(UART_PORT_UART) … #endif`
   (avant `if (gReducedService) return;`) :
   ```c
   #ifdef ENABLE_HOST_MODE
       if (HOST_IsActive()) { HOST_Tick10ms(); return; }
   #endif
   ```
   `HOST_Tick10ms()` sert le squelch matériel + le gating CTCSS/CDCSS RX (la
   boucle du firmware qui le ferait est court‑circuitée).

4. Dans `APP_TimeSlice500ms()`, juste après `gNextTimeslice_500ms = false;` :
   ```c
   #ifdef ENABLE_HOST_MODE
       HOST_Tick500ms();
       if (HOST_IsActive())
           return;
   #endif
   ```

Effet : quand le mode hôte est actif, la boucle radio (RX / dual-watch / scan /
servicing d'interruptions / tâches 500 ms) ne s'exécute plus — l'hôte série a la
main exclusive sur le BK4819. `HOST_Tick500ms()` arme le watchdog de sortie
automatique (~10 s sans commande).

## `App/app/uart.c` (3 insertions)

1. Après `#include "app/uart.h"` :
   ```c
   #ifdef ENABLE_HOST_MODE
   #include "app/host.h"
   #endif
   ```

2. Rendre `SendReply()` non‑`static` :
   `static void SendReply(uint32_t Port, ...)` → `void SendReply(uint32_t Port, ...)`.
   `host.c` la réutilise pour router les réponses vers l'UART matériel **ou**
   le VCP USB selon `Port` (l'UV‑K1 n'a pas de prise série, tout passe par
   l'USB‑C).

3. Dans `UART_HandleCommand()`, dans le `switch (pUART_Command->Header.ID)`,
   juste avant `    } // switch` :
   ```c
   #ifdef ENABLE_HOST_MODE
           case HOST_CMD_MODE:
           case HOST_CMD_SET_VFO:
           case HOST_CMD_SET_RADIO:
           case HOST_CMD_PTT:
           case HOST_CMD_GET_STATUS:
           case HOST_CMD_MONITOR:
           case HOST_CMD_RECALL_CH:
               HOST_HandleCommand(pUART_Command->Header.ID, pUART_Command->Buffer, Port);
               break;
   #endif
   ```

## `App/CMakeLists.txt` (1 insertion)

Après `enable_feature(ENABLE_UART_RW_BK_REGS)` :
```cmake
enable_feature(ENABLE_HOST_MODE
    app/host.c
)
```

## `CMakePresets.json` (1 insertion)

Dans les `cacheVariables` du preset `default`, à côté de
`"ENABLE_UART_RW_BK_REGS": false,` :
```json
"ENABLE_HOST_MODE": false,
```

## Compilation

```bash
cmake --preset Fusion -DENABLE_HOST_MODE=ON -DENABLE_UART_RW_BK_REGS=ON \
      -DVERSION_STRING_2=v5.9.0.H1
cmake --build build/Fusion -j
```

- `ENABLE_UART_RW_BK_REGS` : gardé pour que le même `.bin` serve au test de
  sanité Phase 1 de l'ESP32 (`0x0601`).
- `VERSION_STRING_2=v<amont>.H<HOST_PROTO_VER>` : le suffixe `.H<n>` remonte dans
  `Version[]` / `DisplayVersion[]` (écran de la radio + réponse `0x0514`) et
  distingue ce firmware du F4HWN stock de même version. `build.sh` lit le `<n>`
  depuis `#define HOST_PROTO_VER` de `patch/host.h`.

## Pièges connus

- **K5Viewer** : `K5VIEWER_ParseInput()` → `VCP_K5ViewerPing()` scrute le buffer
  VCP à chaque `APP_Update` et fait `KEYBOARD_InjectKey()` sur un motif
  `AA 55 03/04 <k>` — qui peut apparaître dans nos octets masqués. Le compiler
  avec `-DENABLE_FEAT_F4HWN_K5VIEWER=OFF -DENABLE_FEAT_F4HWN_RXTX_LOG_K5VIEWER=OFF`.
- `HOST_Ptt(0)` : dé-keying = `gCurrentFunction = FUNCTION_FOREGROUND;` +
  `RADIO_SetupRegisters(true)` (qui fait `BK4819_SetupPowerAmplifier(0,0)` +
  `PA_ENABLE=false`). **PAS** `APP_EndTransmission()` (roger + queue CTCSS via
  `SYSTEM_DelayMs`, ~1 s, et dépend du traitement de flags par la boucle
  principale — suspendue en mode hôte).
- `SendReply_VCP()` de l'amont place son footer par pointeur
  (`(Footer_t *)(buf + 4 + Size)`) : si `Size` est **impair**, l'écriture u16
  `pFooter->ID` tombe sur une adresse impaire → **HardFault** (Cortex-M0+, USB-C).
  `HOST_Reply()` bourre donc toute réponse à une taille paire.
- `HOST_SendStatus()` **ne doit pas** appeler `BOARD_ADC_GetBatteryInfo()` :
  busy-wait sur le flag EOS de l'ADC, peut se bloquer si la tâche batterie
  utilise l'ADC en même temps → aucune réponse à `0x0634`. Lire
  `gBatteryVoltageAverage` (déjà maintenu par le firmware) à la place.
- RSSI / bruit / glitch : lectures registre brutes (`BK4819_ReadRegister`
  REG 0x67 / 0x65 / 0x63), comme `CMD_0527`.
- **Audio plat (`SET_VFO` flags b0)** : `HOST_ApplyAudioFilters()` force les bits
  de bypass emphase / HPF / LPF dans `REG_2B` du BK4829
  (b10/9/8 = RX HPF300 / LPF3k / désaccentuation, b2/1/0 = idem TX + préaccent.).
  `RADIO_SetupRegisters()` remet `REG_2B` à 0 (via `BK4819_DisableScramble`) et
  la séquence de keying TX aussi → le helper est rappelé après chacun
  (`HOST_ApplyVfo`, `HOST_OpenAudio`, `HOST_Ptt` on/off). Le compander est aussi
  forcé à 0 (`v->Compander`) en audio plat. Vérifier au rebase que la
  correspondance des bits `REG_2B` du BK4829 n'a pas changé (voir
  `BK4819_EnterBypass` / `BK4819_EnterRaw` dans `driver/bk4829.c`).
- **Double veille (H18, ajusté H19-H21)** : `HOST_Enter()` force
  `gEeprom.DUAL_WATCH = OFF` et le mode hôte suspend `APP_Update()` (donc
  `DualwatchAlternate()`). `HOST_DualWatchTick()` (en tête de `HOST_Tick10ms`)
  réimplémente l'alternance : bascule `gEeprom.RX_VFO` toutes les ~120 ms.
  Verrouillage : **`s_afOpen`** (audio réellement ouvert) fige indéfiniment ;
  une porteuse seule (`s_sqOpen` sans audio) ne donne qu'une **grâce bornée**
  (`HOST_DW_CARRIER_10MS` ~600 ms, le temps du décodage CTCSS) — sinon du bruit
  de bande ou un squelch trop bas figerait le balayage. `s_squelch == 0`
  désactive la DW (AF toujours ouvert). `HOST_SetRadio` arme `s_dualWatch` /
  `s_dwCountdown`. `HOST_Ptt` émet sur le VFO écouté (TDR) si verrouillé, sinon
  sur `TX_VFO`. `GET_STATUS` flags : b4 = VFO RX courant, b5 = DW active.
  `s_cssRequired` recalé sur le VFO courant à chaque tick.
  **H21 — NE PAS appeler `RADIO_SelectVfos()` dans la bascule** : avec
  `DUAL_WATCH != OFF` il refait `gEeprom.RX_VFO = gEeprom.TX_VFO` et annule
  aussitôt la bascule (le poste restait collé au VFO primaire). On règle `gRxVfo`
  directement + `RADIO_SetupRegisters(false)`, comme `DualwatchAlternate()` du
  stock. Vérifier au rebase : `gEeprom.RX_VFO` / `gRxVfo` /
  `RADIO_SelectVfos` (test `DUAL_WATCH`) / `DUAL_WATCH_CHAN_A/B`.
- **Squelch 0 = AF permanent (H20)** : `HOST_ApplyVfo` écrit `gEeprom.SQUELCH_LEVEL`
  **toujours**, y compris 0. Avant (`if (s_squelch) …`) un niveau précédent
  restait actif -> le squelch matériel du BK4819 coupait quand même l'AF en mode
  « données » (squelch 0). `SQUELCH_LEVEL` du menu est sauvé/restauré par
  `HOST_Enter`/`HOST_Exit` (`s_savedSquelch`).
- **`HOST_OpenAudio` léger (H17)** : ne fait QUE lever le mute AF
  (`BK4819_SetAF`), pas un `RADIO_SetModulation` complet. Un
  `RADIO_SetModulation` à chaque ouverture de squelch reprogramme tout le
  BK4819 (~50-100 ms de silence/glitch) → l'audio « arrive en retard » à chaque
  salve en mode hôte, ce que le SA818 (ligne audio continue) n'a pas. La
  modulation/AGC/filtres restent posés par `HOST_ApplyVfo`
  (→ `RADIO_SetupRegisters` → `RADIO_SetModulation`), une fois par retune.
  (H16 avait tenté de figer l'AGC RX `BK4819_SetAGC(false)` — abandonné, l'AGC
  du poste n'était pas en cause.)

## Points à revérifier lors d'un rebase F4HWN

- `App/app/app.c` : les 4 ancrages (noms de fonctions, indentation espaces).
- `App/radio.h` / `App/settings.h` : champs de `VFO_Info_t`, enums
  `ModulationMode_t` / `DCS_CodeType_t` / `OUTPUT_POWER_*` / `DUAL_WATCH_*` /
  `TX_OFFSET_FREQUENCY_DIRECTION_*`, et les prototypes
  `RADIO_ConfigureSquelchAndOutputPower` / `RADIO_ApplyOffset` /
  `RADIO_SetupRegisters` / `RADIO_PrepareTX` / `FREQUENCY_GetBand`.
- `App/app/uart.c` : signature exacte de `SendReply` (pour le `s/static //`) ;
  format de trame (obfuscation, CRC) dans `SendReply` / `UART_IsCommandAvailable`
  — si l'amont change, adapter `host.c` **et** le `UvK5Link` de l'ESP32.
