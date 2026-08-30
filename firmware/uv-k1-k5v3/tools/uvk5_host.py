#!/usr/bin/env python3
"""
Test du "mode hôte" du firmware UV-K1 / UV-K5 V3 (benshi-esp32-sim) depuis un PC.

Protocole série Quansheng : AB CD | taille(LE) | charge masquée | pad | DC BA.
La charge interne = [ID:u16 LE][Size:u16 LE][données], CRC-16/XMODEM sur la
charge interne (commande uniquement ; les réponses n'ont pas de CRC).

Exemples :
    ./uvk5_host.py /dev/ttyACM0 probe
    ./uvk5_host.py /dev/ttyACM0 check
    ./uvk5_host.py /dev/ttyACM0 setvfo --rx 145.500 --tx 145.500   # entre en mode hote tout seul
    ./uvk5_host.py /dev/ttyACM0 setvfo --vfo 0 --rx 145.500 --tx 145.500 --ctcss 88.5
    ./uvk5_host.py /dev/ttyACM0 ptt on ; sleep 2 ; ./uvk5_host.py /dev/ttyACM0 ptt off
    ./uvk5_host.py /dev/ttyACM0 status --loop
    ./uvk5_host.py /dev/ttyACM0 exit

Dépendance : pyserial  (pip install pyserial)
"""
import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial manquant : pip install pyserial")

OBF = bytes((0x16, 0x6C, 0x14, 0xE6, 0x2E, 0x91, 0x0D, 0x40,
             0x21, 0x35, 0xD5, 0x40, 0x13, 0x03, 0xE9, 0x80))

CTCSS = [67.0, 69.3, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8,
         97.4, 100.0, 103.5, 107.2, 110.9, 114.8, 118.8, 123.0, 127.3, 131.8,
         136.5, 141.3, 146.2, 151.4, 156.7, 159.8, 162.2, 165.5, 167.9, 171.3,
         173.8, 177.3, 179.9, 183.5, 186.2, 189.9, 192.8, 196.6, 199.5, 203.5,
         206.5, 210.7, 218.1, 225.7, 229.1, 233.6, 241.8, 250.3, 254.1]

CMD_MODE, CMD_SET_VFO, CMD_SET_RADIO = 0x0630, 0x0631, 0x0632
CMD_PTT, CMD_GET_STATUS, CMD_MONITOR, CMD_RECALL_CH = 0x0633, 0x0634, 0x0635, 0x0636


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(cmd_id: int, body: bytes) -> bytes:
    inner = struct.pack("<HH", cmd_id, len(body)) + body
    inner += struct.pack("<H", crc16(inner))
    obf = bytes(c ^ OBF[i % 16] for i, c in enumerate(inner))
    return b"\xAB\xCD" + struct.pack("<H", len(inner) - 2) + obf + b"\xDC\xBA"


VERBOSE = False


def send(port, cmd_id, body=b"", timeout=0.6):
    port.reset_input_buffer()
    tx = frame(cmd_id, body)
    if VERBOSE:
        print(f"  TX {cmd_id:#06x}: {tx.hex()}")
    port.write(tx)
    port.flush()
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        buf += port.read(64)
        i = buf.find(b"\xAB\xCD")
        if i < 0 or len(buf) < i + 4:
            continue
        size = struct.unpack("<H", buf[i + 2:i + 4])[0]
        need = i + 4 + size + 2 + 2
        if len(buf) < need:
            continue
        payload = buf[i + 4:i + 4 + size]
        if buf[need - 2:need] != b"\xDC\xBA":
            buf = buf[i + 2:]
            continue
        clear = bytes(c ^ OBF[j % 16] for j, c in enumerate(payload))
        rid, rsize = struct.unpack("<HH", clear[:4])
        if VERBOSE:
            print(f"  RX raw : {buf[i:need].hex()}")
            print(f"  RX     : id={rid:#06x} len={rsize} data={clear[4:4 + rsize].hex()}")
        return rid, clear[4:4 + rsize]
    if VERBOSE:
        print(f"  RX     : (rien en {timeout}s ; brut reçu = {buf.hex() or 'aucun octet'})")
    return None, None


def cstr(b: bytes) -> str:
    return b.split(b"\x00")[0].decode(errors="replace")


def tone_code(hz):
    """(CodeType, Code) pour host.c : 0=off, 1=CTCSS index, DCS non géré ici."""
    if not hz:
        return 0, 0
    idx = min(range(len(CTCSS)), key=lambda i: abs(CTCSS[i] - hz))
    return 1, idx              # CTCSS_Options[] est 0-indexé (host.c / firmware)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=38400)
    ap.add_argument("-v", "--verbose", action="store_true", help="hexdump TX/RX")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("probe")
    sub.add_parser("check")
    sub.add_parser("enter")
    sub.add_parser("exit")
    p = sub.add_parser("ptt"); p.add_argument("state", choices=["on", "off"])
    p = sub.add_parser("monitor"); p.add_argument("state", choices=["on", "off"])
    p = sub.add_parser("status"); p.add_argument("--loop", action="store_true")
    p = sub.add_parser("recall"); p.add_argument("--vfo", type=int, default=0); p.add_argument("--ch", type=int, required=True)
    p = sub.add_parser("radio")
    p.add_argument("--txvfo", type=int, default=0); p.add_argument("--dw", type=int, default=0)
    p.add_argument("--xb", type=int, default=0); p.add_argument("--txlock", type=int, default=0)
    p = sub.add_parser("setvfo")
    p.add_argument("--vfo", type=int, default=0)
    p.add_argument("--rx", type=float, required=True, help="MHz")
    p.add_argument("--tx", type=float, help="MHz (défaut = rx)")
    p.add_argument("--mod", type=int, default=0, help="0 FM, 1 AM, 2 USB")
    p.add_argument("--bw", type=int, default=1, help="0 wide, 1 narrow (selon firmware)")
    p.add_argument("--power", type=int, default=7, help="OUTPUT_POWER_* (7 = high)")
    p.add_argument("--ctcss", type=float, default=0.0, help="Hz TX+RX")
    p.add_argument("--rxctcss", type=float, help="Hz RX seul")
    p.add_argument("--txctcss", type=float, help="Hz TX seul")
    p.add_argument("--step", type=int, default=0)
    p.add_argument("--squelch", "--sq", type=int, default=0,
                   help="0 = AF permanent (données) ; 1..9 = squelch + gating CTCSS/CDCSS RX")
    a = ap.parse_args()

    global VERBOSE
    VERBOSE = a.verbose

    port = serial.Serial(a.port, a.baud, timeout=0.05)
    time.sleep(0.1)

    if a.cmd == "probe":
        rid, d = send(port, 0x0514, struct.pack("<I", int(time.time()) & 0xFFFFFFFF))
        if rid == 0x0515:
            print("liaison OK, firmware :", cstr(d[:16]))
        else:
            sys.exit("pas de réponse 0x0515 (câble / port / baud / flash ?)")

    elif a.cmd == "check":
        # 0x0630 {0} : sans effet (n'entre pas en mode hôte) mais SEUL le
        # firmware mode hôte y répond -> confirme le bon .bin.
        rid, d = send(port, CMD_MODE, b"\x00")
        if rid == CMD_MODE and d and len(d) >= 2:
            print(f"✓ FIRMWARE MODE HÔTE présent  (proto H{d[1]}, "
                  f"{cstr(d[2:18])})")
        else:
            rid2, _ = send(port, 0x0514, struct.pack("<I", 1))
            if rid2 == 0x0515:
                print("✗ liaison OK mais PAS le firmware mode hôte "
                      "(0x0630 sans réponse) -> c'est le F4HWN stock")
            else:
                sys.exit("✗ aucune réponse : câble / port / baud / flash")

    elif a.cmd == "enter":
        rid, d = send(port, CMD_MODE, b"\x01")
        if rid == CMD_MODE and d and len(d) >= 2:
            print(f"mode hôte : {'actif' if d[0] else 'inactif'}  "
                  f"(proto H{d[1]}, {cstr(d[2:18])})")
        else:
            sys.exit("pas de réponse 0x0630 -> firmware stock ? (essayer 'check')")

    elif a.cmd == "exit":
        send(port, CMD_MODE, b"\x00")
        print("sortie mode hôte")

    elif a.cmd == "ptt":
        rid, d = send(port, CMD_PTT, bytes([1 if a.state == "on" else 0]))
        ok = rid == CMD_PTT and d and d[0]
        print("ptt", a.state, "->", "reçu (mode hôte actif)" if ok else "pas de réponse")
        if ok:
            time.sleep(0.7)
            _, s2 = send(port, CMD_GET_STATUS)
            if s2 and len(s2) >= 11:
                f = s2[0]
                print("   -> func =", f, "(1 = TRANSMIT)" if f == 1 else "")

    elif a.cmd == "monitor":
        send(port, CMD_MONITOR, bytes([1 if a.state == "on" else 0]))
        print("monitor", a.state)

    elif a.cmd == "recall":
        rid, d = send(port, CMD_RECALL_CH, struct.pack("<BH", a.vfo, a.ch))
        print("rappel canal", a.ch, "->", "ok" if d and d[0] else "refusé")

    elif a.cmd == "radio":
        rid, d = send(port, CMD_SET_RADIO, bytes([a.txvfo, a.dw, a.xb, a.txlock]))
        print("set_radio ->", "ok" if d and d[0] else "refusé")

    elif a.cmd == "setvfo":
        rxf = round(a.rx * 1e5)
        txf = round((a.tx if a.tx is not None else a.rx) * 1e5)
        rxct, txct = tone_code(a.rxctcss if a.rxctcss is not None else a.ctcss), \
                     tone_code(a.txctcss if a.txctcss is not None else a.ctcss)
        body = struct.pack("<BIIBBBBBBBHB", a.vfo, rxf, txf, a.mod, a.bw, a.power,
                           rxct[0], rxct[1], txct[0], txct[1], a.step, a.squelch)
        rid, d = send(port, CMD_SET_VFO, body)
        print(f"set_vfo {a.rx}/{a.tx or a.rx} MHz ->",
              "reçu (mode hôte actif)" if (rid == CMD_SET_VFO and d and d[0]) else "pas de réponse")

    elif a.cmd == "status":
        while True:
            rid, d = send(port, CMD_GET_STATUS)
            if not d or len(d) < 11:
                print("pas de status"); break
            vals = struct.unpack("<BHBBhBHB", d[:11])
            func, rssi, noise, glitch, dbm, ctc, batt, flags = vals
            sq, sqlvl = (d[11], d[12]) if len(d) >= 13 else (0, 0)
            fl = (("HOST " if flags & 1 else "") + ("TX " if flags & 2 else "")
                  + ("MON " if flags & 4 else "") + ("SIG" if flags & 8 else ""))
            sqd = (("SQ%d " % sqlvl if sq & 1 else "SQ0 ")
                   + ("cssReq " if sq & 2 else "") + ("sqOpen " if sq & 4 else "")
                   + ("cssOk " if sq & 8 else "") + ("afOpen" if sq & 16 else "afMute"))
            print(f"func={func} rssi={rssi} dBm={dbm} noise={noise} glitch={glitch} "
                  f"ctc={ctc} batt={batt}mV  [{fl}]  squelch:[{sqd}]")
            if not a.loop:
                break
            time.sleep(0.5)


if __name__ == "__main__":
    main()
