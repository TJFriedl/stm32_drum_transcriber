#!/usr/bin/env python3
"""
BLE "hello world" receiver for the NUCLEO-WB55 P2P server.

The P-NUCLEO-WB55 USB dongle must be running ST's BLE_TransparentModeVCP
firmware. It then shows up as a virtual COM port that carries raw HCI packets
(UART "H4" framing), and this script plays the role of the BLE host:

  1. reset + init the dongle's BLE stack as a GAP central
  2. scan for a device advertising the name "P2PSRV1"
  3. connect to it
  4. find the P2P "notify" characteristic (UUID 0000fe42-8e22-4541-9d4c-21edae82ed19)
  5. enable notifications on it
  6. check that every notification is {0xA5, seq} with seq going up by 1

Usage:
  pip install pyserial
  python hello_receiver.py                # auto-detect dongle, run until Ctrl-C
  python hello_receiver.py --count 20     # stop after 20 packets
  python hello_receiver.py --port COM7 --verbose
"""

import argparse
import struct
import sys
import time

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
# Constants (values taken from Middlewares/ST/STM32_WPAN/ble/core/auto/*.c)
# ---------------------------------------------------------------------------
TARGET_NAME = b"P2PSRV1"
HELLO_MARKER = 0xA5

# 128-bit UUID of the notify characteristic, little-endian as sent over the air
# (same bytes as COPY_P2P_NOTIFY_UUID in p2p_stm.c, reversed)
NOTIFY_CHAR_UUID = bytes(reversed([0x00, 0x00, 0xFE, 0x42, 0x8E, 0x22, 0x45, 0x41,
                                   0x9D, 0x4C, 0x21, 0xED, 0xAE, 0x82, 0xED, 0x19]))


def aci(ocf):
    """ST vendor-specific opcode: OGF 0x3F."""
    return (0x3F << 10) | ocf


# ST's full BLE stack rejects the standard HCI scan/connect commands (status 0x01,
# "unknown command"), so scanning and connecting use ST's ACI GAP commands instead.
OP_RESET = 0x0C03
OP_ACI_GAP_TERMINATE = aci(0x093)
OP_ACI_GAP_START_GENERAL_DISCOVERY = aci(0x097)
OP_ACI_GAP_CREATE_CONNECTION = aci(0x09C)
OP_ACI_GAP_TERMINATE_GAP_PROC = aci(0x09D)
OP_ACI_GAP_INIT = aci(0x08A)
OP_ACI_GATT_INIT = aci(0x101)
OP_ACI_GATT_DISC_CHAR_BY_UUID = aci(0x116)
OP_ACI_GATT_WRITE_CHAR_VALUE = aci(0x11C)

GAP_CENTRAL_ROLE = 0x04
GAP_GENERAL_DISCOVERY_PROC = 0x02
GAP_DIRECT_CONNECTION_ESTABLISHMENT_PROC = 0x40

# HCI event codes
EVT_DISCONN_COMPLETE = 0x05
EVT_CMD_COMPLETE = 0x0E
EVT_CMD_STATUS = 0x0F
EVT_LE_META = 0x3E
EVT_VENDOR = 0xFF

# LE meta sub-events
LE_CONN_COMPLETE = 0x01
LE_ADV_REPORT = 0x02
LE_ENH_CONN_COMPLETE = 0x0A

# ST vendor event codes (ble_vs_codes.h)
VS_ATT_READ_BY_TYPE_RESP = 0x0C06
VS_GATT_NOTIFICATION = 0x0C0F
VS_GATT_PROC_COMPLETE = 0x0C10
VS_GATT_ERROR_RESP = 0x0C11
VS_GATT_DISC_READ_CHAR_BY_UUID_RESP = 0x0C12


class BleError(Exception):
    pass


# ---------------------------------------------------------------------------
# Minimal HCI transport over the dongle's COM port
# ---------------------------------------------------------------------------
class Hci:
    def __init__(self, port, verbose=False):
        # Baud rate is ignored by USB CDC but pyserial wants one
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.verbose = verbose

    def close(self):
        self.ser.close()

    def _read_exact(self, n, deadline):
        buf = b""
        while len(buf) < n:
            if time.monotonic() > deadline:
                raise TimeoutError
            buf += self.ser.read(n - len(buf))
        return buf

    def send_cmd(self, opcode, params=b""):
        pkt = struct.pack("<BHB", 0x01, opcode, len(params)) + params
        if self.verbose:
            print(f"  TX {pkt.hex(' ')}")
        self.ser.write(pkt)

    def read_event(self, timeout):
        """Return (event_code, params) or None on timeout. Non-event packets are skipped."""
        deadline = time.monotonic() + timeout
        try:
            while True:
                ptype = self._read_exact(1, deadline)[0]
                if ptype == 0x04:                               # HCI event
                    evt, plen = self._read_exact(2, deadline)
                elif ptype == 0x82:                             # ST extended event (16-bit length)
                    evt = self._read_exact(1, deadline)[0]
                    plen = struct.unpack("<H", self._read_exact(2, deadline))[0]
                elif ptype == 0x02:                             # ACL data: not used, skip it
                    _, dlen = struct.unpack("<HH", self._read_exact(4, deadline))
                    self._read_exact(dlen, deadline)
                    continue
                else:
                    continue                                    # out of sync, drop the byte
                params = self._read_exact(plen, deadline)
                if self.verbose:
                    print(f"  RX evt 0x{evt:02X}: {params.hex(' ')}")
                return evt, params
        except TimeoutError:
            return None

    def command(self, opcode, params=b"", timeout=2.0):
        """Send a command and wait for its Command Complete. Returns the return parameters."""
        self.send_cmd(opcode, params)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ev = self.read_event(deadline - time.monotonic())
            if ev is None:
                break
            evt, p = ev
            if evt == EVT_CMD_COMPLETE and struct.unpack_from("<H", p, 1)[0] == opcode:
                ret = p[3:]
                if ret and ret[0] != 0:
                    raise BleError(f"command 0x{opcode:04X} failed, status 0x{ret[0]:02X}")
                return ret
            if evt == EVT_CMD_STATUS and struct.unpack_from("<H", p, 2)[0] == opcode:
                if p[0] != 0:
                    raise BleError(f"command 0x{opcode:04X} failed, status 0x{p[0]:02X}")
                return b""
        raise BleError(f"no response to command 0x{opcode:04X} (is the dongle running BLE_TransparentModeVCP?)")


# ---------------------------------------------------------------------------
# BLE steps
# ---------------------------------------------------------------------------
def init_stack(hci):
    hci.command(OP_RESET)
    hci.command(OP_ACI_GATT_INIT)
    # Role, privacy disabled, device name length
    hci.command(OP_ACI_GAP_INIT, bytes([GAP_CENTRAL_ROLE, 0x00, 0x08]))


def local_name(adv_data):
    """Pull the (complete or shortened) local name out of advertising data."""
    i = 0
    while i < len(adv_data):
        length = adv_data[i]
        if length == 0 or i + 1 + length > len(adv_data):
            break
        ad_type = adv_data[i + 1]
        if ad_type in (0x08, 0x09):
            return adv_data[i + 2:i + 1 + length]
        i += 1 + length
    return None


def scan_for_target(hci, timeout):
    # 60 ms scan interval / 30 ms window, public own address, report duplicates
    hci.command(OP_ACI_GAP_START_GENERAL_DISCOVERY, struct.pack("<HHBB", 0x0060, 0x0030, 0x00, 0x00))
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ev = hci.read_event(deadline - time.monotonic())
            if ev is None:
                break
            evt, p = ev
            # Only single-report events are parsed; that is what the controller sends in practice
            if evt != EVT_LE_META or p[0] != LE_ADV_REPORT or p[1] != 1:
                continue
            addr_type = p[3]
            addr = p[4:10]
            data_len = p[10]
            name = local_name(p[11:11 + data_len])
            if name == TARGET_NAME:
                return addr_type, addr
    finally:
        stop_gap_proc(hci, GAP_GENERAL_DISCOVERY_PROC)
    raise BleError(f"did not find '{TARGET_NAME.decode()}' (the Nucleo stops advertising 60 s after reset: press its RESET button)")


def stop_gap_proc(hci, proc_code):
    """Stop a running GAP procedure. It may already have ended on its own, so errors are ignored."""
    try:
        hci.command(OP_ACI_GAP_TERMINATE_GAP_PROC, bytes([proc_code]))
    except BleError:
        pass


def connect(hci, addr_type, addr, timeout=10.0):
    params = struct.pack("<HHB6sBHHHHHH",
                         0x0060, 0x0030,     # scan interval / window
                         addr_type, addr,
                         0x00,               # own address type: public
                         0x0018, 0x0028,     # connection interval 30..50 ms
                         0x0000,             # peripheral latency
                         0x01F4,             # supervision timeout 5 s
                         0x0000, 0x0000)     # CE length
    hci.command(OP_ACI_GAP_CREATE_CONNECTION, params)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ev = hci.read_event(deadline - time.monotonic())
        if ev is None:
            break
        evt, p = ev
        if evt == EVT_LE_META and p[0] in (LE_CONN_COMPLETE, LE_ENH_CONN_COMPLETE):
            status = p[1]
            if status != 0:
                raise BleError(f"connection failed, status 0x{status:02X}")
            return struct.unpack_from("<H", p, 2)[0]
    stop_gap_proc(hci, GAP_DIRECT_CONNECTION_ESTABLISHMENT_PROC)
    raise BleError("connection attempt timed out")


def wait_gatt_proc(hci, conn, on_vendor_event=None, timeout=5.0):
    """Wait for ACI_GATT_PROC_COMPLETE, passing any other vendor events to on_vendor_event."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ev = hci.read_event(deadline - time.monotonic())
        if ev is None:
            break
        evt, p = ev
        if evt == EVT_DISCONN_COMPLETE:
            raise BleError(f"disconnected, reason 0x{p[3]:02X}")
        if evt != EVT_VENDOR:
            continue
        ecode = struct.unpack_from("<H", p, 0)[0]
        body = p[2:]
        if ecode == VS_GATT_PROC_COMPLETE:
            err = body[2]
            if err != 0:
                raise BleError(f"GATT procedure failed, error 0x{err:02X}")
            return
        if ecode == VS_GATT_ERROR_RESP:
            continue                        # normal at the end of discovery procedures
        if on_vendor_event:
            on_vendor_event(ecode, body)
    raise BleError("GATT procedure timed out")


def find_notify_handle(hci, conn):
    hci.command(OP_ACI_GATT_DISC_CHAR_BY_UUID,
                struct.pack("<HHHB", conn, 0x0001, 0xFFFF, 0x02) + NOTIFY_CHAR_UUID)
    found = []

    def on_event(ecode, body):
        if ecode == VS_GATT_DISC_READ_CHAR_BY_UUID_RESP:
            # Attribute value = characteristic declaration: properties(1), value handle(2), UUID
            value = body[5:5 + body[4]]
            found.append(struct.unpack_from("<H", value, 1)[0])

    wait_gatt_proc(hci, conn, on_event)
    if not found:
        raise BleError("notify characteristic not found on the Nucleo")
    return found[0]


def enable_notifications(hci, conn, value_handle):
    cccd_handle = value_handle + 1          # CCCD comes right after the value (see p2p_stm.c:126)
    hci.command(OP_ACI_GATT_WRITE_CHAR_VALUE,
                struct.pack("<HHB", conn, cccd_handle, 2) + bytes([0x01, 0x00]))
    wait_gatt_proc(hci, conn)


def receive_hellos(hci, value_handle, count):
    ok = bad = 0
    expected_seq = None
    try:
        while count is None or ok + bad < count:
            ev = hci.read_event(5.0)
            if ev is None:
                print("  (no packet for 5 s...)")
                continue
            evt, p = ev
            if evt == EVT_DISCONN_COMPLETE:
                print(f"Disconnected by peer, reason 0x{p[3]:02X}")
                break
            if evt != EVT_VENDOR or struct.unpack_from("<H", p, 0)[0] != VS_GATT_NOTIFICATION:
                continue
            _conn, attr, length = struct.unpack_from("<HHB", p, 2)
            data = p[7:7 + length]
            if attr != value_handle:
                continue

            good = len(data) == 2 and data[0] == HELLO_MARKER and \
                (expected_seq is None or data[1] == expected_seq)
            if good:
                ok += 1
            else:
                bad += 1
            want = f"A5 {expected_seq:02X}" if expected_seq is not None else "A5 ??"
            print(f"  got {data.hex(' ').upper():<6} expected {want}  {'OK' if good else 'MISMATCH'}")
            if len(data) == 2:
                expected_seq = (data[1] + 1) & 0xFF
    except KeyboardInterrupt:
        pass
    return ok, bad


def find_dongle_port():
    # The dongle enumerates as ST's USB virtual COM port (VID 0x0483, PID 0x5740).
    # The Nucleo's ST-LINK also uses VID 0x0483, so match the PID too.
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x0483 and p.pid == 0x5740:
            return p.device
    raise BleError("dongle not found; pass --port COMx (check Device Manager > Ports)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="dongle COM port (default: auto-detect)")
    ap.add_argument("--count", type=int, help="stop after this many packets")
    ap.add_argument("--scan-time", type=float, default=15.0, help="seconds to scan (default 15)")
    ap.add_argument("--verbose", action="store_true", help="hex-dump raw HCI traffic")
    args = ap.parse_args()

    hci = None
    conn = None
    try:
        port = args.port or find_dongle_port()
        print(f"Opening {port}")
        hci = Hci(port, args.verbose)

        init_stack(hci)
        print("Dongle BLE stack initialised")

        print(f"Scanning for '{TARGET_NAME.decode()}'...")
        addr_type, addr = scan_for_target(hci, args.scan_time)
        print(f"Found it at {':'.join(f'{b:02X}' for b in reversed(addr))}")

        conn = connect(hci, addr_type, addr)
        print(f"Connected (handle 0x{conn:04X}) - the Nucleo's blue LED should be on")

        value_handle = find_notify_handle(hci, conn)
        enable_notifications(hci, conn, value_handle)
        print(f"Notifications enabled on handle 0x{value_handle:04X}; waiting for packets (Ctrl-C to stop)")

        ok, bad = receive_hellos(hci, value_handle, args.count)
        print(f"\nSummary: {ok}/{ok + bad} packets OK, {bad} mismatched")
        return 0 if bad == 0 and ok > 0 else 1
    except BleError as e:
        print(f"ERROR: {e}")
        return 2
    finally:
        if hci:
            if conn is not None:
                try:
                    hci.send_cmd(OP_ACI_GAP_TERMINATE, struct.pack("<HB", conn, 0x13))
                    time.sleep(0.2)
                except serial.SerialException:
                    pass
            hci.close()


if __name__ == "__main__":
    sys.exit(main())
