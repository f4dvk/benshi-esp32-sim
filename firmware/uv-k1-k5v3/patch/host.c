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
} HOST_SetVfo_t;

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
    uint8_t  flags;      // b0 host active, b1 in TX, b2 monitor, b3 signal reçu
    uint8_t  sq;         // diagnostic squelch : b0 s_squelch>0, b1 cssRequired,
                         //   b2 sqOpen, b3 cssOk, b4 afOpen
    uint8_t  sqLevel;    // s_squelch (0..9)
} HOST_Status_t;

// ---- state -----------------------------------------------------------------

#define HOST_WATCHDOG_500MS  240  // ~120 s sans commande hôte -> sortie auto
                                  // l ESP32 enverra un GET_STATUS toutes les ~3 s

static bool     s_active       = false;
static uint16_t s_watchdog     = 0;
static uint8_t  s_savedDualWatch = DUAL_WATCH_OFF;

// Squelch en mode hôte (la boucle du firmware qui le gérerait est suspendue).
static uint8_t  s_squelch      = 0;      // 0 = AF toujours ouvert
static bool     s_cssRequired  = false;  // un ton/code RX est configuré
static bool     s_sqOpen       = false;  // squelch matériel ouvert (sqlLost)
static bool     s_cssOk        = false;  // ton/code RX détecté
static bool     s_afOpen       = false;  // état courant du chemin audio
static bool     s_monitor      = false;  // MONITOR{1} : force l'AF ouvert

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
    gEeprom.DUAL_WATCH  = DUAL_WATCH_OFF;   // RAM only, not persisted
    gScanStateDir       = SCAN_OFF;

    FUNCTION_Select(FUNCTION_FOREGROUND);
    RADIO_SelectVfos();
    RADIO_SetupRegisters(true);
    HOST_OpenAudio();
}

void HOST_Exit(void)
{
    if (!s_active)
        return;
    s_active = false;

    if (gCurrentFunction == FUNCTION_TRANSMIT)
        gCurrentFunction = FUNCTION_FOREGROUND;   // coupe le PA via RADIO_SetupRegisters ci-dessous

    gEeprom.DUAL_WATCH = s_savedDualWatch;

    // Hand the radio back to the firmware with a clean reload from EEPROM.
    RADIO_ConfigureChannel(0, VFO_CONFIGURE_RELOAD);
    RADIO_ConfigureChannel(1, VFO_CONFIGURE_RELOAD);
    RADIO_SelectVfos();
    RADIO_SetupRegisters(true);
}

void HOST_Tick500ms(void)
{
    if (!s_active)
        return;
    if (s_watchdog > 0 && --s_watchdog == 0)
        HOST_Exit();
}

// ---- individual command handlers ------------------------------------------

// En mode hôte la boucle de squelch/écoute du firmware est suspendue : rien
// n'ouvrirait le chemin audio. On force donc l'AF ouvert en continu (squelch
// grand ouvert, comme un SA818 avec squelch=0) -> l'audio reçu sort en
// permanence sur le HP / la prise casque (que l'ESP32 capte).
static void HOST_OpenAudio(void)
{
    // BK4819_SetAF(BK4819_AF_FM) (via RADIO_SetModulation) active directement le
    // DAC audio / RX DSP -> l'AF sort quel que soit l'état du squelch (le mute
    // sur squelch fermé est une politique du firmware, dont la boucle est
    // suspendue ici). Pas besoin de toucher BK4819_SetupSquelch.
    RADIO_SetModulation(gRxVfo->Modulation);
    BK4819_SetRxAudioGain();
    AUDIO_AudioPathOn();
    gEnableSpeaker = true;
    s_afOpen = true;
}

static void HOST_MuteAudio(void)
{
    gEnableSpeaker = false;
    AUDIO_AudioPathOff();
    BK4819_SetAF(BK4819_AF_MUTE);
    s_afOpen = false;
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

    // CTCSS / CDCSS RX : lecture directe du type détecté (niveau fiable ;
    // les interruptions ctcssFound/Lost se sont révélées inversées ici).
    // 1 = CTCSS conforme, 2 = CDCSS conforme, 0 = aucun.
    s_cssOk = s_cssRequired ? (BK4819_GetCTCType() != 0) : true;

    if (s_squelch == 0 || s_monitor)
        return;   // AF déjà forcé ouvert, pas de gating

    const bool want = s_sqOpen && (!s_cssRequired || s_cssOk);
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

    v->Band = FREQUENCY_GetBand(v->freq_config_RX.Frequency);

    s_squelch     = (c->squelch <= 9) ? c->squelch : 9;
    s_cssRequired = (v->freq_config_RX.CodeType != CODE_TYPE_OFF);
    if (s_squelch)
        gEeprom.SQUELCH_LEVEL = s_squelch;

    RADIO_ConfigureSquelchAndOutputPower(v);   // -> Squelch*Thresh selon SQUELCH_LEVEL
    RADIO_SelectVfos();
    RADIO_SetupRegisters(true);                // BK4819_SetupSquelch + CTCSS/CDCSS RX

    s_sqOpen = s_cssOk = false;
    if (s_squelch == 0)
        HOST_OpenAudio();      // données : AF permanent
    else
        HOST_MuteAudio();      // phonie : fermé, HOST_Tick10ms ouvrira sur signal
}

static void HOST_SetRadio(const HOST_SetRadio_t *c)
{
    gEeprom.TX_VFO           = (c->txVfo & 1);
    gEeprom.DUAL_WATCH       = c->dualWatch;
    s_savedDualWatch         = c->dualWatch;
    gEeprom.CROSS_BAND_RX_TX = c->crossBand;
    gEeprom.VfoInfo[0].TX_LOCK = c->txLock ? true : false;
    gEeprom.VfoInfo[1].TX_LOCK = c->txLock ? true : false;

    RADIO_SelectVfos();
    RADIO_SetupRegisters(true);
}

// Keying PA minimal, sans passer par RADIO_PrepareTX / FUNCTION_Transmit :
// on évite AUDIO_AudioPathOff, GUI_DisplayScreen, DTMF_Reply, l'audio-scope,
// les logs RXTX... qui peuvent perturber le CDC USB pendant l'émission.
// Reproduit uniquement la séquence RF de RADIO_SetTxParameters().
static void HOST_Ptt(uint8_t on)
{
    VFO_Info_t *v = &gEeprom.VfoInfo[gEeprom.TX_VFO & 1];

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
        BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
        gCurrentFunction = FUNCTION_TRANSMIT;
    } else {
        gCurrentFunction = FUNCTION_FOREGROUND;
        BK4819_SetupPowerAmplifier(0, 0);
        BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);
        BK4819_ExitSubAu();
        BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
        RADIO_SetupRegisters(true);
        if (s_squelch == 0 || s_monitor) {
            HOST_OpenAudio();
        } else {
            HOST_MuteAudio();
            s_sqOpen = s_cssOk = false;   // HOST_Tick10ms rouvrira sur signal
        }
    }
}

static void HOST_Monitor(uint8_t on)
{
    s_monitor = on ? true : false;
    if (on) {
        HOST_OpenAudio();               // force l'AF ouvert (bouton "monitor")
    } else {
        HOST_MuteAudio();
        s_sqOpen = s_cssOk = false;     // phonie : HOST_Tick10ms rouvrira sur signal
    }
}

static void HOST_RecallChannel(const HOST_RecallCh_t *c)
{
    uint8_t vi = (c->vfo & 1);
    gEeprom.ScreenChannel[vi] = c->ch;
    gEeprom.MrChannel[vi]     = c->ch;
    RADIO_ConfigureChannel(vi, VFO_CONFIGURE_RELOAD);
    RADIO_SelectVfos();
    RADIO_SetupRegisters(true);
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
    // (squelch ouvert + ton/code RX OK) -> l'ESP32 le mappe sur is_sq.
    const bool sig = s_sqOpen && (!s_cssRequired || s_cssOk);
    st.flags     = (uint8_t)((s_active ? 1 : 0)
                 | ((gCurrentFunction == FUNCTION_TRANSMIT) ? 2 : 0)
                 | (s_monitor ? 4 : 0)
                 | (sig ? 8 : 0));
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
