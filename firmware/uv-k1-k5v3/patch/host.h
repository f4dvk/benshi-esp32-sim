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
 * "Host mode" for benshi-esp32-sim.
 *
 * When active, the normal radio loop (APP_Update / dual-watch / scanner /
 * interrupt servicing) is suspended so that an external controller on the
 * serial port owns the BK4819 through high-level commands (see host.c).
 *
 * Guarded by ENABLE_HOST_MODE (CMake option). Zero cost when disabled.
 * ------------------------------------------------------------------------- */

#ifndef APP_HOST_H
#define APP_HOST_H

#ifdef ENABLE_HOST_MODE

#include <stdbool.h>
#include <stdint.h>

// Révision du protocole / du patch mode hôte. À incrémenter à chaque
// modification de host.c/host.h. Renvoyée dans la réponse 0x0630 ET ajoutée
// à la chaîne de version affichée par la radio (suffixe ".H<n>", via
// -DVERSION_STRING_2=... dans build.sh) -> confirmation visuelle du bon .bin.
#define HOST_PROTO_VER 17

// Serial command IDs handled by host mode (outside the Quansheng 0x05xx range).
#define HOST_CMD_MODE       0x0630   // {u8 enter}              -> {u8 active, u8 proto, char ver[16]}
#define HOST_CMD_SET_VFO    0x0631   // see host.c HOST_SetVfo_t   -> {u8 ok}
#define HOST_CMD_SET_RADIO  0x0632   // {u8 txVfo,dw,xb,txLock}    -> {u8 ok}
#define HOST_CMD_PTT        0x0633   // {u8 on}                    -> {u8 ok}
#define HOST_CMD_GET_STATUS 0x0634   // {}                        -> HOST_Status_t
#define HOST_CMD_MONITOR    0x0635   // {u8 on}                    -> {u8 ok}
#define HOST_CMD_RECALL_CH  0x0636   // {u8 vfo, u16 ch}          -> {u8 ok}

// true while the external controller owns the radio.
bool HOST_IsActive(void);

// Dispatch one serial command (called from UART_HandleCommand).
// pBuffer points at the deobfuscated inner payload (Header_t + data).
void HOST_HandleCommand(uint16_t id, const uint8_t *pBuffer, uint32_t Port);

// Called once per 500 ms from APP_TimeSlice500ms: drives the watchdog that
// auto-leaves host mode if the controller goes silent.
void HOST_Tick500ms(void);

// Called every 10 ms from APP_TimeSlice10ms while host mode is active: services
// the BK4819 squelch / CTCSS-CDCSS interrupts and gates the AF path when a
// squelch level was requested (SET_VFO squelch != 0). No-op in "squelch 0"
// (AF-always-open) mode.
void HOST_Tick10ms(void);

// Leave host mode and hand the radio back to the firmware (also reachable
// from a key combo in CheckKeys as a safety).
void HOST_Exit(void);

#endif // ENABLE_HOST_MODE
#endif // APP_HOST_H
