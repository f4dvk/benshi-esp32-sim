#!/usr/bin/env bash
#
# Compile le firmware F4HWN Fusion pour UV-K1 / UV-K5 V3 avec le "mode hôte"
# de benshi-esp32-sim (option ENABLE_HOST_MODE) : suspension de la boucle radio
# + commandes série haut niveau (fréq / VFO / mémoires / mode / CTCSS / PTT).
#
# Prérequis : git, docker, perl.
#
# Usage :
#   ./build.sh [TAG] [PRESET]
#     TAG     tag ou commit amont (défaut : v5.9.0)
#     PRESET  preset CMake        (défaut : Fusion)
#
# Le résultat renommé est déposé dans bin/. Voir patch/integration.md pour le
# détail exact des modifications (revue / application manuelle).
#
set -euo pipefail

UPSTREAM=https://github.com/armel/uv-k1-k5v3-firmware-custom
TAG=${1:-v5.9.0}
PRESET=${2:-Fusion}

HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

# Révision du mode hôte = HOST_PROTO_VER de patch/host.h. Sert de suffixe de
# version (".H<n>") -> confirmation visuelle du bon firmware sur la radio.
HOST_REV=$(grep -oE '#define HOST_PROTO_VER +[0-9]+' "$HERE/patch/host.h" | grep -oE '[0-9]+$')
[ -n "$HOST_REV" ] || { echo "!! HOST_PROTO_VER introuvable dans patch/host.h"; exit 1; }
VERSUFFIX="${TAG#v}.H${HOST_REV}"   # ex. 5.9.0.H1

echo "== clone $UPSTREAM @ $TAG"
git clone --depth 1 --branch "$TAG" "$UPSTREAM" "$WORK/fw"
cd "$WORK/fw"
COMMIT=$(git rev-parse --short HEAD)

echo "== fichiers du mode hôte"
cp "$HERE/patch/host.c" App/app/host.c
cp "$HERE/patch/host.h" App/app/host.h

echo "== points d'ancrage"
# Chaque insertion est faite une seule fois (perl en mode "slurp", pas de /g)
# sur un clone neuf. Le fichier amont est indenté avec des ESPACES (4).
# Un échec d'ancrage est fatal (contrôle plus bas).

# App/app/app.c #0 : include de host.h
perl -0pi -e 's{#include "app/app.h"\n}{$&#ifdef ENABLE_HOST_MODE\n#include "app/host.h"\n#endif\n}' App/app/app.c

# App/app/app.c #1 : dans APP_Update(), APRÈS le service UART_PORT_VCP (USB)
# -> en mode hôte, la boucle radio est coupée mais les commandes USB
#    (dont HOST_MODE{0}) continuent d'être traitées.
perl -0pi -e 's{        UART_HandleCommand\(UART_PORT_VCP\);\n        // SCHEDULER_Enable\(\);\n    \}\n#endif\n}{$&\n#ifdef ENABLE_HOST_MODE\n    if (HOST_IsActive())\n        return;\n#endif\n}' App/app/app.c
# repli si ENABLE_USB est absent : ancrer en tête de fonction
perl -0pi -e 's/\Qvoid APP_Update(void)\E\n\{\n/$&#ifdef ENABLE_HOST_MODE\n    if (HOST_IsActive())\n        return;\n#endif\n/ unless /HOST_IsActive/' App/app/app.c

# App/app/app.c #2 : APP_TimeSlice10ms(), juste après le service UART_PORT_UART
perl -0pi -e 's{        UART_HandleCommand\(UART_PORT_UART\);\n        // SCHEDULER_Enable\(\);\n    \}\n#endif\n}{$&\n#ifdef ENABLE_HOST_MODE\n    if (HOST_IsActive()) { HOST_Tick10ms(); return; }\n#endif\n}' App/app/app.c

# App/app/app.c #3 : APP_TimeSlice500ms(), watchdog + sortie anticipée
perl -0pi -e 's/\Qvoid APP_TimeSlice500ms(void)\E\n\{\n    gNextTimeslice_500ms = false;\n/$&#ifdef ENABLE_HOST_MODE\n    HOST_Tick500ms();\n    if (HOST_IsActive())\n        return;\n#endif\n/' App/app/app.c

# App/app/uart.c : include
perl -0pi -e 's{#include "app/uart.h"\n}{$&#ifdef ENABLE_HOST_MODE\n#include "app/host.h"\n#endif\n}' App/app/uart.c

# App/app/uart.c : rendre SendReply() non-static (host.c le réutilise pour
# router les réponses vers l'UART matériel OU le VCP USB selon Port)
perl -0pi -e 's/static void SendReply\(uint32_t Port, void \*pReply, uint16_t Size\)/void SendReply(uint32_t Port, void *pReply, uint16_t Size)/' App/app/uart.c

# App/app/uart.c : cases du switch (juste avant "    } // switch")
perl -0pi -e 's/\n    \} \/\/ switch/\n#ifdef ENABLE_HOST_MODE\n        case HOST_CMD_MODE:\n        case HOST_CMD_SET_VFO:\n        case HOST_CMD_SET_RADIO:\n        case HOST_CMD_PTT:\n        case HOST_CMD_GET_STATUS:\n        case HOST_CMD_MONITOR:\n        case HOST_CMD_RECALL_CH:\n            HOST_HandleCommand(pUART_Command->Header.ID, pUART_Command->Buffer, Port);\n            break;\n#endif$&/' App/app/uart.c

# App/CMakeLists.txt : option + source
perl -0pi -e 's/enable_feature\(ENABLE_UART_RW_BK_REGS\)\n/$&enable_feature(ENABLE_HOST_MODE\n    app\/host.c\n)\n/' App/CMakeLists.txt

# CMakePresets.json : défaut (off)
perl -0pi -e 's/( *)"ENABLE_UART_RW_BK_REGS": false,\n/$&$1"ENABLE_HOST_MODE": false,\n/' CMakePresets.json

echo "== contrôle"
c=$(grep -c 'HOST_IsActive'     App/app/app.c        || true)
[ "$c" -ge 3 ] || { echo "!! app.c : $c/3 ancrages HOST_IsActive"; exit 1; }
grep -q 'HOST_Tick500ms'        App/app/app.c        || { echo "!! app.c : watchdog 500ms"; exit 1; }
grep -q 'HOST_Tick10ms'         App/app/app.c        || { echo "!! app.c : tick 10ms"; exit 1; }
grep -q 'app/host.h'            App/app/app.c        || { echo "!! app.c : include"; exit 1; }
grep -q 'HOST_HandleCommand'      App/app/uart.c     || { echo "!! uart.c dispatch"; exit 1; }
grep -q 'app/host.h'             App/app/uart.c     || { echo "!! uart.c include"; exit 1; }
grep -qE '^void SendReply\('     App/app/uart.c     || { echo "!! uart.c: SendReply toujours static"; exit 1; }
grep -q 'ENABLE_HOST_MODE'      App/CMakeLists.txt   || { echo "!! CMakeLists.txt"; exit 1; }
grep -q 'ENABLE_HOST_MODE'      CMakePresets.json    || { echo "!! CMakePresets.json"; exit 1; }

echo "== build (preset=$PRESET, ENABLE_HOST_MODE=ON, ENABLE_UART_RW_BK_REGS=ON, version v${VERSUFFIX})"
# ENABLE_UART_RW_BK_REGS : gardé actif -> le même .bin sert aussi au test de
# sanité Phase 1 de l'ESP32 (lecture de registre BK4819 via 0x0601).
# VERSION_STRING_2 : suffixe ".H<rev>" -> visible à l'écran de la radio ET dans
# la réponse 0x0514, pour distinguer ce firmware du F4HWN stock.
# K5VIEWER désactivé : son parseur (VCP_K5ViewerPing) tourne sur le MÊME port
# série et peut injecter des touches à partir de nos octets masqués / gêner nos
# commandes. Inutile ici (la radio est pilotée par notre protocole).
./compile-with-docker.sh "$PRESET" \
    -DENABLE_HOST_MODE=ON -DENABLE_UART_RW_BK_REGS=ON \
    -DENABLE_FEAT_F4HWN_K5VIEWER=OFF \
    -DENABLE_FEAT_F4HWN_RXTX_LOG_K5VIEWER=OFF \
    -DVERSION_STRING_2="v${VERSUFFIX}"

BIN=$(find build -name '*.bin' ! -name '*sa818*' | head -n1)
[ -n "$BIN" ] || { echo "!! binaire introuvable dans build/"; exit 1; }

OUT="$HERE/bin/uv-k1-k5v3-f4hwn-$(echo "$PRESET" | tr '[:upper:]' '[:lower:]')-hostmode-v${VERSUFFIX}-${COMMIT}.bin"
mkdir -p "$HERE/bin"
cp "$BIN" "$OUT"
echo "== OK -> ${OUT#"$HERE"/}"
sha256sum "$OUT"
