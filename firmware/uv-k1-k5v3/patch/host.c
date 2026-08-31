/* Copyright 2026 benshi-esp32-sim contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* ---------------------------------------------------------------------------
 * Host mode implementation. See host.h.
 *
 * Frequencies on the wire are in 10 Hz units (same as the firmware's
 * VFO_Info_t::freq_config_*.Frequency and BK4819_SetFrequency), i.e.
 * 145.500 MHz -> 14550000.
 *
 * Everything RF is delegated to the firmware's own tested helpers; this file
 * only marshals serial payloads and calls them.
 *
 * >>> Points to re-check when rebasing on a new F4HWN release: the exact names
 *     RADIO_ConfigureSquelchAndOutputPower / RADIO_ApplyOffset / FREQUENCY_GetBand,
 *     and that VFO_Info_t still has the fields used below.
 * ------------------------------------------------------------------------- */

#ifdef ENABLE_HOST_MODE

#include <string.h>
#include "app/host.h"
#include "app/app.h"
#include "app/chFrScanner.h"
#include "audio.h"
#include "dcs.h"
#include "driver/bk4819.h"
#include "driver/uart.h"
#include "frequencies.h"
#include "functions.h"
#include "helper/battery.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "version.h"

// ---- serial framing --------------------------------------------------------
// Header_t on the wire is {u16 ID; u16 Size} little-endian.
// PY32 (Cortex-M0+) faults on unaligned u16/u32 access -> the command payload
// structs below are __packed__ (byte access) and read from the 4-aligned
// pUART_Command->Buffer.

typedef struct {
    uint16_t ID;
    uint16_t Size;
} HOST_Header_t;   // first member of the packed command payload structs

// uart.c's SendReply, de-static'd by build.sh. It obfuscates + frames + routes
// to the hardware UART OR the USB VCP according to Port -> replies to a command
// received over USB-C go back over USB-C (the UV-K1 has no serial jack).
extern void SendReply(uint32_t Port, void *pReply, uint16_t Size);

// Reply to a host command. `data` = payload AFTER the 4-byte inner header.
static void HOST_Reply(uint32_t Port, uint16_t id, const void *data, uint16_t len)
{
    uint8_t buf[4 + 64 + 1];
    if (len > 64)
        return;
    buf[0] = (uint8_t)(id & 0xFF);
    buf[1] = (uint8_t)(id >> 8);
    buf[2] = (uint8_t)(len & 0xFF);   // en-tête interne : longueur RÉELLE
    buf[3] = (uint8_t)(len >> 8);
    if (data && len)
        memcpy(buf + 4, data, len);

    uint16_t total = (uint16_t)(4 + len);
    // SendReply_VCP() de l'amont place son footer par pointeur : Header_t(u16 ID)
    // à l'offset 4+total -> DOIT être pair sinon HardFault (Cortex-M0+).
    if (total & 1u) {
        buf[total] = 0;
        total++;
    }
    SendReply(Port, buf, total);
}

static void HOST_ReplyOk(uint32_t Port, uint16_t id, uint8_t ok)
{
    HOST_Reply(Port, id, &ok, 1);
}

// ---- command payloads (packed, little-endian = native on PY32) ---------------

typedef struct __attribute__((__packed__)) {
    HOST_Header_t Header;
    uint8_t  vfo;         // 0 = A, 1 = B
    uint32_t rxFreq;      // 10 Hz units
    uint32_t txFreq;      // 10 Hz units
    uint8_t  modulation;  // ModulationMode_t (0 FM, 1 AM, 2 USB)
    uint8_t  bandwidth;   // CHANNEL_BANDWIDTH (0 wide ... narrower)
    uint8_t  power;       // OUTPUT_POWER_*
    uint8_t  rxCodeType;  // DCS_CodeType_t
    uint8_t  rxCode;      // index (CTCSS_Options / DCS table)
    uint8_t  txCodeType;
    uint8_t  txCode;
    uint16_t step;        // StepFrequency (10 Hz units), 0 = keep
    uint8_t  squelch;     // 0 = AF toujours ouvert (données) ; 1..9 = squelch
                          //     matériel + gating CTCSS/CDCSS RX (phonie)
    uint8_t  flags;       // b0 = audio PLAT : bypass pré/dé-emphase + HPF/LPF AF
                          //      (RX et TX) + compander off -> pour l'AFSK/APRS.
                          //      Equivalent du bit pre_de_emph_bypass du SA818.
} HOST_SetVfo_t;

#define HOST_VFO_FLAG_FLAT_AUDIO  0x01

typedef struct __attribute__((__packed__)) {
    HOST_Header_t Header;
    uint8_t txVfo;        // active VFO (0/1)
    uint8_t dualWatch;    // 0 off, 1 A, 2 B  (DUAL_WATCH_*)
    uint8_t crossBand;    // CROSS_BAND_*
    uint8_t txLock;       // 0 = TX allowed
} HOST_SetRadio_t;

typedef struct __attribute__((__packed__)) {
    HOST_Header_t Header;
    uint8_t on;
} HOST_U8Arg_t;

typedef struct __attribute__((__packed__)) {
    HOST_Header_t Header;
    uint8_t  vfo;
    uint16_t ch;
} HOST_RecallCh_t;

typedef struct __attribute__((__packed__)) {
    uint8_t  function;   // gCurrentFunction
    uint16_t rssi;       // BK4819 raw (9 bits)
    uint8_t  noise;
    uint8_t  glitch;
    int16_t  rssi_dBm;
    uint8_t  ctcssType;  // BK4819_GetCTCType()
    uint16_t batterymV;
    uint8_t  flags;      // b0 host active, b1 in TX, b2 monitor, b3 signal reçu,
                         //   b4 VFO RX courant (0/1), b5 double veille active
    uint8_t  sq;         // diagnostic squelch : b0 s_squelch>0, b1 cssRequired,
                         //   b2 sqOpen, b3 cssOk, b4 afOpen
    uint8_t  sqLevel;    // s_squelch (0..9)
} HOST_Status_t;

// ---- state -----------------------------------------------------------------

#define HOST_WATCHDOG_500MS  240  // ~120 s sans commande hôte -> sortie auto
                                  // l ESP32 enverra un GET_STATUS toutes les ~250 ms
#define HOST_TX_MAX_500MS    16   // ~8 s d'émission maxi : filet si la trame
                                  // PTT-OFF se perd (RF du PA sur la liaison série)

static bool     s_active       = false;
static uint16_t s_watchdog     = 0;
static uint8_t  s_savedDualWatch = DUAL_WATCH_OFF;
static uint8_t  s_savedSquelch  = 0;   // SQUELCH_LEVEL du menu, restauré à la sortie

// Double veille en mode hôte : la boucle firmware (DualwatchAlternate) est
// suspendue, on alterne nous-mêmes les VFO dans HOST_Tick10ms.
static uint8_t  s_dualWatch    = DUAL_WATCH_OFF;   // 0 off, 1 VFO A, 2 VFO B
static uint16_t s_dwCountdown  = 0;                // unités 10 ms : temps sur le VFO courant
static uint16_t s_dwCarrier    = 0;                // unités 10 ms : porteuse sans audio (borné)
#define HOST_DW_TOGGLE_10MS   12                   // ~120 ms par VFO en balayage
#define HOST_DW_CARRIER_10MS  60                   // ~600 ms de grâce sur une porteuse (décodage CTCSS)

// Squelch en mode hôte (la boucle du firmware qui le gérerait est suspendue).
static uint8_t  s_squelch      = 0;      // 0 = AF toujours ouvert
static bool     s_cssRequired  = false;  // un ton/code RX est configuré
static bool     s_flatAudio    = false;  // SET_VFO flags b0 : bypass emphase / filtres AF
static bool     s_sqOpen       = false;  // squelch matériel ouvert (sqlLost)
static bool     s_cssOk        = false;  // ton/code RX détecté
static bool     s_afOpen       = false;  // état courant du chemin audio
static bool     s_monitor      = false;  // MONITOR{1} : force l'AF ouvert
static uint8_t  s_txGuard      = 0;      // compte à rebours anti-émission-bloquée

bool HOST_IsActive(void) { return s_active; }

static void HOST_OpenAudio(void);
static void HOST_MuteAudio(void);

static void HOST_Enter(void)
{
    if (s_active) {
        s_watchdog = HOST_WATCHDOG_500MS;
        return;
    }
    s_active   = true;
    s_watchdog = HOST_WATCHDOG_500MS;

    // Stop the firmware from time-slicing VFOs while the host drives.
    s_savedDualWatch    = gEeprom.DUAL_WATCH;
    s_savedSquelch      = gEeprom.SQUELCH_LEVEL;
    gEeprom.DUAL_WATCH  = DUAL_WATCH_OFF;   // RAM only, not persisted
    gScanStateDir       = SCAN_OFF;

    FUNCTION_Select(FUNCTION_FOREGROUND);
    RADIO_SelectVfos();
    RADIO_SetupRegisters(true);
    s_sqOpen = s_cssOk = false;
    HOST_MuteAudio();   // HOST_Tick10ms ouvre juste après (s_squelch initial = 0 -> forcé)
}

void HOST_Exit(void)
{
    if (!s_active)
        return;
    s_active = false;

    if (gCurrentFunction == FUNCTION_TRANSMIT)
        gCurrentFunction = FUNCTION_FOREGROUND;   // coupe le PA via RADIO_SetupRegisters ci-dessous

    s_flatAudio = false;   // rend la chaine AF normale au firmware
    s_dualWatch = DUAL_WATCH_OFF;
    gEeprom.DUAL_WATCH    = s_savedDualWatch;
    gEeprom.SQUELCH_LEVEL = s_savedSquelch;

    // Hand the radio back to the firmware with a clean reload from EEPROM.
    RADIO_ConfigureChannel(0, VFO_CONFIGURE_RELOAD);
    RADIO_ConfigureChannel(1, VFO_CONFIGURE_RELOAD);
    RADIO_SelectVfos();
    RADIO_SetupRegisters(true);
}

static void HOST_Ptt(uint8_t on);   /* fwd */

void HOST_Tick500ms(void)
{
    if (!s_active)
        return;
    // Filet anti-émission-bloquée : si la trame PTT-OFF s'est perdue (RF du PA
    // sur la liaison série), on coupe au bout de HOST_TX_MAX_500MS.
    if (gCurrentFunction == FUNCTION_TRANSMIT && s_txGuard > 0 && --s_txGuard == 0)
        HOST_Ptt(0);
    if (s_watchdog > 0 && --s_watchdog == 0)
        HOST_Exit();
}

// ---- individual command handlers ------------------------------------------

// En mode hôte la boucle de squelch/écoute du firmware est suspendue : rien
// n'ouvrirait le chemin audio. On force donc l'AF ouvert en continu (squelch
// grand ouvert, comme un SA818 avec squelch=0) -> l'audio reçu sort en
// permanence sur le HP / la prise casque (que l'ESP32 capte).
// Filtres AF du BK4829 (REG_2B). En mode "audio plat" (s_flatAudio) on bypasse
// TOUT le conditionnement AF, RX et TX, pour laisser passer l'AFSK 1200 sans
// distorsion (equivalent du pre_de_emph_bypass du SA818) :
//   b10 = desactive HPF 300 Hz RX      b2 = desactive HPF 300 Hz TX
//   b9  = desactive LPF 3 kHz RX       b1 = desactive LPF 1 TX
//   b8  = desactive desaccentuation RX b0 = desactive preaccentuation TX
// Sinon on remet ces bits a 0 (chaine AF FM normale). A rappeler apres tout ce
// qui reecrit REG_2B : RADIO_SetupRegisters (-> BK4819_DisableScramble met 0),
// et la sequence de keying TX.
#define HOST_AF_FLAT_MASK  ((1u<<10)|(1u<<9)|(1u<<8)|(1u<<2)|(1u<<1)|(1u<<0))
static void HOST_ApplyAudioFilters(void)
{
    uint16_t reg2b = BK4819_ReadRegister(BK4819_REG_2B);
    if (s_flatAudio)
        reg2b |= HOST_AF_FLAT_MASK;
    else
        reg2b &= ~HOST_AF_FLAT_MASK;
    BK4819_WriteRegister(BK4819_REG_2B, reg2b);
}

// APP_StartListening() : fonction RX du firmware stock (non déclarée dans app.h).
extern void APP_StartListening(FUNCTION_Type_t function);
// BK4819_RX_TurnOn() : REG_30/REG_37 = DSP RX actif (pas dans bk4819.h).
extern void BK4819_RX_TurnOn(void);

static void HOST_OpenAudio(void)
{
    // Le DSP RX du BK4819 (REG_30) n'est ré-armé par RIEN dans le flux mode hôte
    // (RADIO_SetupRegisters ne le fait pas ; BK4819_TxOn_Beep le passe en config
    // TX à chaque PTT). Sans DSP RX, le discriminateur ne sort rien -> pas de
    // souffle en squelch 0. On le force ici.
    BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, true);
    BK4819_RX_TurnOn();
    // H25 : on appelle DIRECTEMENT la fonction RX du firmware stock. Nos essais
    // (BK4819_SetAF seul H17, RADIO_SetModulation H23) ne routaient pas le
    // discriminateur sans porteuse (squelch 0 / monitor = silence). APP_StartListening
    // fait, en plus, FUNCTION_Select(FUNCTION_RECEIVE/MONITOR) + gEnableSpeaker +
    // AUDIO_AudioPathOn dans le bon ordre. FUNCTION_MONITOR quand le squelch est
    // ouvert en permanence (0 / monitor) : le firmware traite alors le squelch
    // comme désactivé.
    APP_StartListening((s_squelch == 0 || s_monitor) ? FUNCTION_MONITOR : FUNCTION_RECEIVE);
    HOST_ApplyAudioFilters();   // APP_StartListening -> RADIO_SetModulation : on garantit l'audio plat
    s_afOpen = true;
}

static void HOST_MuteAudio(void)
{
    gEnableSpeaker = false;
    AUDIO_AudioPathOff();
    BK4819_SetAF(BK4819_AF_MUTE);
    if (gCurrentFunction != FUNCTION_TRANSMIT)
        gCurrentFunction = FUNCTION_FOREGROUND;   // symétrique d'APP_StartListening
    s_afOpen = false;
}

// Double veille : alterne les deux VFO tant qu'aucun signal n'est capté ; se
// verrouille sur le VFO qui reçoit (squelch + CTCSS OK). Remplace la boucle
// DualwatchAlternate() du firmware, suspendue en mode hôte. Appelé au début de
// HOST_Tick10ms, avant le gating audio.
static void HOST_DualWatchTick(void)
{
    if (s_dualWatch == DUAL_WATCH_OFF || s_squelch == 0 || s_monitor)
        return;   // squelch 0 = AF toujours ouvert -> pas de balayage possible

    // Chaque VFO a son propre ton RX -> on recale le besoin CTCSS sur le VFO
    // courant (en mono-VFO c'est HOST_ApplyVfo qui fixe s_cssRequired).
    s_cssRequired = (gRxVfo->pRX->CodeType != CODE_TYPE_OFF);

    // Audio réellement ouvert (squelch + CTCSS OK, passé par le gating) -> on
    // reste verrouillé tant qu'il y a du son.
    if (s_afOpen) {
        s_dwCountdown = HOST_DW_TOGGLE_10MS;
        s_dwCarrier   = 0;
        return;
    }

    // Porteuse détectée mais pas (encore) d'audio : on patiente, le temps que
    // le décodeur CTCSS se cale ou que le gating ouvre l'AF. BORNÉ : du bruit de
    // bande / un squelch réglé trop bas ne doit pas figer le balayage.
    if (s_sqOpen) {
        if (++s_dwCarrier < HOST_DW_CARRIER_10MS)
            return;
    } else {
        s_dwCarrier = 0;
    }

    if (s_dwCountdown > 0 && --s_dwCountdown > 0)
        return;
    s_dwCountdown = HOST_DW_TOGGLE_10MS;
    s_dwCarrier   = 0;

    // NE PAS appeler RADIO_SelectVfos() ici : avec DUAL_WATCH != OFF il refait
    // gEeprom.RX_VFO = gEeprom.TX_VFO -> notre bascule serait immédiatement
    // annulée (le poste restait collé au VFO primaire, jamais de balayage).
    // On règle gRxVfo directement, comme DualwatchAlternate() du firmware stock.
    gEeprom.RX_VFO = gEeprom.RX_VFO ? 0 : 1;
    gRxVfo         = &gEeprom.VfoInfo[gEeprom.RX_VFO];
    RADIO_SetupRegisters(false);
    s_sqOpen = s_cssOk = false;
}

// Sert le squelch matériel + le gating CTCSS/CDCSS RX quand un niveau de
// squelch a été demandé. Appelé toutes les 10 ms en mode hôte.
void HOST_Tick10ms(void)
{
    if (!s_active || gCurrentFunction == FUNCTION_TRANSMIT)
        return;

    // Draine les interruptions BK4819 (le firmware le ferait — sa boucle est
    // suspendue). On ne suit QUE le squelch matériel par interruption (edge)
    // (bits REG_02 : sqlLost=2 sqlFound=3) ; il faut vider REG_02 sinon REG_0C
    // reste bloqué à 1.
    int guard = 8;
    while ((BK4819_ReadRegister(BK4819_REG_0C) & 1u) && guard--) {
        BK4819_WriteRegister(BK4819_REG_02, 0);
        const uint16_t it = BK4819_ReadRegister(BK4819_REG_02);
        if (it & (1u << 2)) s_sqOpen = true;
        if (it & (1u << 3)) s_sqOpen = false;
    }

    HOST_DualWatchTick();   // peut recaler s_cssRequired sur le VFO courant

    // CTCSS / CDCSS RX : lecture directe du type détecté (niveau fiable ;
    // les interruptions ctcssFound/Lost se sont révélées inversées ici).
    // 1 = CTCSS conforme, 2 = CDCSS conforme, 0 = aucun.
    s_cssOk = s_cssRequired ? (BK4819_GetCTCType() != 0) : true;

    // Squelch 0 / monitor -> AF toujours ouvert ; sinon gate squelch + CTCSS.
    // Tout passe par le MÊME HOST_OpenAudio()/HOST_MuteAudio() depuis le tick
    // (le seul chemin dont on a la preuve qu'il produit de l'audio).
    const bool want = (s_squelch == 0 || s_monitor)
                    || (s_sqOpen && (!s_cssRequired || s_cssOk));
    if (want != s_afOpen) {
        if (want) HOST_OpenAudio();
        else      HOST_MuteAudio();
    }
}

static void HOST_ApplyVfo(const HOST_SetVfo_t *c)
{
    uint8_t vi = (c->vfo & 1);
    VFO_Info_t *v = &gEeprom.VfoInfo[vi];

    v->freq_config_RX.Frequency = c->rxFreq;
    v->freq_config_TX.Frequency = c->txFreq;
    v->freq_config_RX.CodeType  = (DCS_CodeType_t)c->rxCodeType;
    v->freq_config_RX.Code      = c->rxCode;
    v->freq_config_TX.CodeType  = (DCS_CodeType_t)c->txCodeType;
    v->freq_config_TX.Code      = c->txCode;
    v->pRX = &v->freq_config_RX;
    v->pTX = &v->freq_config_TX;

    // Explicit split: no computed offset.
    v->TX_OFFSET_FREQUENCY           = 0;
    v->TX_OFFSET_FREQUENCY_DIRECTION = TX_OFFSET_FREQUENCY_DIRECTION_OFF;

    v->Modulation        = (ModulationMode_t)c->modulation;
    v->CHANNEL_BANDWIDTH = c->bandwidth;
    v->OUTPUT_POWER      = c->power;
    if (c->step)
        v->StepFrequency = c->step;

    s_flatAudio = (c->flags & HOST_VFO_FLAG_FLAT_AUDIO) != 0;
    if (s_flatAudio)
        v->Compander = 0;   // le compander deformerait l'AFSK -> off en audio plat

    v->Band = FREQUENCY_GetBand(v->freq_config_RX.Frequency);

    s_squelch     = (c->squelch <= 9) ? c->squelch : 9;
    s_cssRequired = (v->freq_config_RX.CodeType != CODE_TYPE_OFF);
    // En mode hôte l'ESP est maître du squelch -> on l'écrit TOUJOURS, y compris
    // 0 : SQUELCH_LEVEL 0 = seuils au minimum = squelch matériel grand ouvert.
    // Sinon (ancien code : « if (s_squelch) »), un niveau précédent restait en
    // place et le squelch matériel du BK4819 continuait de couper l'AF même en
    // mode « données / AF permanent » (squelch 0).
    gEeprom.SQUELCH_LEVEL = s_squelch;

    RADIO_ConfigureSquelchAndOutputPower(v);   // -> Squelch*Thresh selon SQUELCH_LEVEL
    RADIO_SelectVfos();
    RADIO_SetupRegisters(true);                // BK4819_SetupSquelch + CTCSS/CDCSS RX
    HOST_ApplyAudioFilters();                  // apres RADIO_SetupRegisters (qui remet REG_2B a 0)

    s_sqOpen = s_cssOk = false;
    // On ferme, et c'est HOST_Tick10ms qui ouvre : squelch 0 / monitor -> tout de
    // suite (want forcé), phonie -> sur signal. Passer par le MÊME chemin que le
    // cas qui marche (tick), après stabilisation du BK4819 post-retune.
    HOST_MuteAudio();
}

static void HOST_SetRadio(const HOST_SetRadio_t *c)
{
    gEeprom.TX_VFO           = (c->txVfo & 1);
    gEeprom.DUAL_WATCH       = c->dualWatch;
    s_savedDualWatch         = c->dualWatch;
    gEeprom.CROSS_BAND_RX_TX = c->crossBand;
    gEeprom.VfoInfo[0].TX_LOCK = c->txLock ? true : false;
    gEeprom.VfoInfo[1].TX_LOCK = c->txLock ? true : false;

    s_dualWatch = c->dualWatch;
    if (s_dualWatch != DUAL_WATCH_OFF) {
        gEeprom.RX_VFO = (s_dualWatch == DUAL_WATCH_CHAN_B) ? 1 : 0;
        s_dwCountdown  = HOST_DW_TOGGLE_10MS;
        s_dwCarrier    = 0;
    } else {
        gEeprom.RX_VFO = gEeprom.TX_VFO;
    }

    RADIO_SelectVfos();
    RADIO_SetupRegisters(true);
    // RADIO_SetupRegisters vient de couper l'AF au niveau matériel : on remet
    // l'état logiciel cohérent (fermé) pour que HOST_Tick10ms rouvre.
    s_sqOpen = s_cssOk = false;
    HOST_MuteAudio();
}

// Keying PA minimal, sans passer par RADIO_PrepareTX / FUNCTION_Transmit :
// on évite AUDIO_AudioPathOff, GUI_DisplayScreen, DTMF_Reply, l'audio-scope,
// les logs RXTX... qui peuvent perturber le CDC USB pendant l'émission.
// Reproduit uniquement la séquence RF de RADIO_SetTxParameters().
static void HOST_Ptt(uint8_t on)
{
    // En double veille, si on écoute activement un VFO on répond dessus (TDR),
    // sinon on émet sur le VFO principal.
    uint8_t txi = (s_dualWatch != DUAL_WATCH_OFF && s_afOpen)
                ? (uint8_t)(gEeprom.RX_VFO & 1)
                : (uint8_t)(gEeprom.TX_VFO & 1);
    VFO_Info_t *v = &gEeprom.VfoInfo[txi];

    if (on) {
        gSerialConfigCountDown_500ms = 0;
        BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, false);
        BK4819_SetFilterBandwidth(v->CHANNEL_BANDWIDTH, false);
        BK4819_SetFrequency(v->freq_config_TX.Frequency);
        BK4819_PrepareTransmit();
        SYSTEM_DelayMs(10);
        BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, true);
        SYSTEM_DelayMs(5);
        BK4819_SetupPowerAmplifier(v->TXP_CalculatedSetting, v->freq_config_TX.Frequency);
        SYSTEM_DelayMs(5);
        if (v->freq_config_TX.CodeType == CODE_TYPE_CONTINUOUS_TONE)
            BK4819_SetCTCSSFrequency(CTCSS_Options[v->freq_config_TX.Code]);
        else if (v->freq_config_TX.CodeType == CODE_TYPE_DIGITAL ||
                 v->freq_config_TX.CodeType == CODE_TYPE_REVERSE_DIGITAL)
            BK4819_SetCDCSSCodeWord(DCS_GetGolayCodeWord(v->freq_config_TX.CodeType,
                                                         v->freq_config_TX.Code));
        else
            BK4819_ExitSubAu();
        HOST_ApplyAudioFilters();   // pré-emphase TX bypassée si audio plat (AFSK)
        BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
        gCurrentFunction = FUNCTION_TRANSMIT;
        s_txGuard = HOST_TX_MAX_500MS;
    } else {
        s_txGuard = 0;
        gCurrentFunction = FUNCTION_FOREGROUND;
        BK4819_SetupPowerAmplifier(0, 0);
        BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);
        BK4819_ExitSubAu();
        BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
        RADIO_SetupRegisters(true);
        HOST_ApplyAudioFilters();
        HOST_MuteAudio();
        s_sqOpen = s_cssOk = false;       // HOST_Tick10ms rouvre (forcé si sq0/monitor, sinon sur signal)
    }
}

static void HOST_Monitor(uint8_t on)
{
    s_monitor = on ? true : false;
    if (!on) s_sqOpen = s_cssOk = false;
    // HOST_Tick10ms applique (want forcé si monitor, sinon gate sur signal).
}

static void HOST_RecallChannel(const HOST_RecallCh_t *c)
{
    uint8_t vi = (c->vfo & 1);
    gEeprom.ScreenChannel[vi] = c->ch;
    gEeprom.MrChannel[vi]     = c->ch;
    RADIO_ConfigureChannel(vi, VFO_CONFIGURE_RELOAD);
    RADIO_SelectVfos();
    RADIO_SetupRegisters(true);
    s_sqOpen = s_cssOk = false;
    HOST_MuteAudio();   // le tick rouvre (cf. HOST_SetRadio)
}

static void HOST_SendStatus(uint32_t Port)
{
    HOST_Status_t st;

    // Lectures registre BK4819 brutes = chemin prouvé sûr (identique à
    // CMD_0527). NE PAS appeler BOARD_ADC_GetBatteryInfo() ici : il fait un
    // busy-wait sur le flag EOS de l'ADC et peut se bloquer si la tâche
    // batterie utilise l'ADC en même temps -> pas de réponse. On lit la
    // moyenne déjà maintenue par le firmware.
    const uint16_t rssi = BK4819_ReadRegister(BK4819_REG_67) & 0x01FF;

    st.function  = (uint8_t)gCurrentFunction;
    st.rssi      = rssi;
    st.noise     = BK4819_ReadRegister(BK4819_REG_65) & 0x007F;
    st.glitch    = (uint8_t)BK4819_ReadRegister(BK4819_REG_63);
    st.rssi_dBm  = (int16_t)((int)(rssi >> 1) - 160);   // approx K5 : rssi/2 - 160
    st.ctcssType = BK4819_GetCTCType();
    st.batterymV = (uint16_t)(gBatteryVoltageAverage * 10);   // 10 mV -> mV
    // b0 mode hôte actif, b1 en TX, b2 monitor forcé, b3 signal reçu présent
    // -> l'ESP32 le mappe sur is_sq et l'utilise pour ouvrir la capture RX.
    // b4 VFO RX courant (0/1), b5 double veille active (-> écran de façade).
    // En squelch 0 / monitor l'AF est forcé ouvert (HOST_OpenAudio) mais aucun
    // front d'interruption squelch ne met s_sqOpen à 1 sans porteuse -> on
    // reporte s_afOpen, sinon l'ESP ne capture jamais (pas de souffle récepteur).
    const bool sig = (s_squelch == 0 || s_monitor)
                   ? s_afOpen
                   : (s_sqOpen && (!s_cssRequired || s_cssOk));
    st.flags     = (uint8_t)((s_active ? 1 : 0)
                 | ((gCurrentFunction == FUNCTION_TRANSMIT) ? 2 : 0)
                 | (s_monitor ? 4 : 0)
                 | (sig ? 8 : 0)
                 | ((gEeprom.RX_VFO & 1) ? 16 : 0)
                 | ((s_dualWatch != DUAL_WATCH_OFF) ? 32 : 0));
    st.sq        = (uint8_t)(((s_squelch != 0) ? 1 : 0)
                 | (s_cssRequired ? 2 : 0)
                 | (s_sqOpen ? 4 : 0)
                 | (s_cssOk ? 8 : 0)
                 | (s_afOpen ? 16 : 0));
    st.sqLevel   = s_squelch;

    HOST_Reply(Port, HOST_CMD_GET_STATUS, &st, sizeof(st));
}

// ---- dispatch ------------------------------------------------------------

void HOST_HandleCommand(uint16_t id, const uint8_t *pBuffer, uint32_t Port)
{
    // Toute commande hôte réarme le watchdog de sortie auto.
    s_watchdog = HOST_WATCHDOG_500MS;

    if (id == HOST_CMD_MODE) {
        const HOST_U8Arg_t *c = (const HOST_U8Arg_t *)pBuffer;
        if (c->on)
            HOST_Enter();
        else
            HOST_Exit();
        // Réponse = preuve que ce firmware EST le firmware mode hôte
        // (le firmware stock ne connaît pas 0x0630 et ne répond pas).
        struct __attribute__((__packed__)) {
            uint8_t active;
            uint8_t proto;
            char    ver[16];
        } r;
        r.active = s_active ? 1 : 0;
        r.proto  = HOST_PROTO_VER;
        memset(r.ver, 0, sizeof(r.ver));
        strncpy(r.ver, Version, sizeof(r.ver) - 1);
        HOST_Reply(Port, HOST_CMD_MODE, &r, sizeof(r));
        return;
    }

    // GET_STATUS : requête passive, n'entre PAS en mode hôte (on peut sonder
    // l'état sans prendre le contrôle).
    if (id == HOST_CMD_GET_STATUS) {
        HOST_SendStatus(Port);
        return;
    }

    // Toute autre commande = prise de contrôle implicite : pas besoin d'un
    // HOST_MODE{1} explicite, et le watchdog / HOST_MODE{0} gèrent la sortie.
    if (!s_active)
        HOST_Enter();

    // Réponse AVANT d'exécuter : RADIO_PrepareTX / APP_EndTransmission / la
    // reprogrammation RF peuvent bloquer > 500 ms (rampe PA, queue CTCSS...).
    // Le `ok` = "reçu & mode hôte actif" ; l'état RF réel se lit via GET_STATUS.
    HOST_ReplyOk(Port, id, s_active ? 1 : 0);

    switch (id) {
    case HOST_CMD_SET_VFO:
        HOST_ApplyVfo((const HOST_SetVfo_t *)pBuffer);
        break;
    case HOST_CMD_SET_RADIO:
        HOST_SetRadio((const HOST_SetRadio_t *)pBuffer);
        break;
    case HOST_CMD_PTT:
        HOST_Ptt(((const HOST_U8Arg_t *)pBuffer)->on);
        break;
    case HOST_CMD_MONITOR:
        HOST_Monitor(((const HOST_U8Arg_t *)pBuffer)->on);
        break;
    case HOST_CMD_RECALL_CH:
        HOST_RecallChannel((const HOST_RecallCh_t *)pBuffer);
        break;
    default:
        break;
    }
}

#endif // ENABLE_HOST_MODE
