/******************************************************************************
 *
 *  Copyright (C) 1999-2012 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/******************************************************************************
 *
 *  Data type declarations.
 *
 *  NOTE (benshi-esp32-sim): the upstream file pulls in the whole Bluedroid
 *  "stack/bt_types.h" for the UINTn / SINTn aliases. Those headers are not
 *  shipped in the Arduino-ESP32 SDK, so we declare the handful of aliases the
 *  SBC encoder actually needs here, straight from <stdint.h>. The sizes match
 *  exactly what libbt.a was compiled with (32-bit little-endian target).
 *
 ******************************************************************************/

#ifndef SBC_TYPES_H
#define SBC_TYPES_H

#include <stdint.h>

#ifndef BT_TYPES_H  /* in case a real bt_types.h ever gets on the include path */
typedef uint8_t   UINT8;
typedef uint16_t  UINT16;
typedef uint32_t  UINT32;
typedef int8_t    INT8;
typedef int16_t   INT16;
typedef int32_t   INT32;
#endif

typedef short SINT16;
typedef long  SINT32;

#if (SBC_IPAQ_OPT == TRUE)
typedef int64_t SINT64;
#elif (SBC_IS_64_MULT_IN_WINDOW_ACCU == TRUE) || (SBC_IS_64_MULT_IN_IDCT == TRUE)
typedef int64_t SINT64;
#endif

#define abs32(x) ( (x >= 0) ? x : (-x) )

#endif
