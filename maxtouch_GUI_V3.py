"""
MTCH3380P / MTCH3240P Key + Slider/Wheel Monitor GUI

Protocol reference: DS40002708A
 - T500 Key Array: STATUS (DETECT bit7) + KEYSTATE bitmask
  - T501 Key Controller: per-key status plus touch amplitude
 - T502 Slider Controller: STATUS (TCHDET/POSCHG/DIRCHG) + TCHPOS(16) + TCHAMPL(16)
 - T5 Message Processor: REPORTID + MESSAGE + CHECKSUM
 - T44 Message Count
 - T254 Extended Object Table (16-bit type codes for T500/T501/T502)
"""
import sys
import time
import math
import colorsys
import struct
import wave
import io
import tempfile
import shutil
import platform
import threading
from collections import deque
import logging
import configparser
import os
import re
import tkinter as tk
from tkinter import ttk, messagebox

APP_VERSION = 'V3.3'
APP_AUTHORS = 'David Qin, Zell, Claude'

# Vector-drawn "touch chip" icon (rounded IC body + pins + touch-ripple
# rings), pre-rendered to small PNGs and embedded as base64 so the window
# icon works standalone with no external asset file required.
_ICON_PNG_B64 = {
    16: 'iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAAsUlEQVR42mNgGB5g8sxNDiC8+MbigmWPF+6H4pNA/BSKT8LEQWpg6pEN+A/CCy8s/b/o3rz/M85P/b/wztz/i+/PB2MQGyQGkgOpganHcAnIBpDC1i2t/2dfmfF/6c1FYAxig8RAciA12LwANnHR5aVg20Aazr4/+f/1/2dgDGKDxEByIDUYLgByGkB46f1Fn0BOBtkK0wzDIDGwHFANTD1VDaDMC1QLRLKjkeKENLQBAC7EifjL0l8vAAAAAElFTkSuQmCC',
    32: 'iVBORw0KGgoAAAANSUhEUgAAACAAAAAgCAYAAABzenr0AAABfElEQVR42u1WPUsDQRDNL0kpIpIicEdICg9B6zRCkDQ2qVREDLbpFUTTGE4iCUHEdDZWamHjD0hvIaTxR4x5Q96xHiqbL1ZwBx7czu68+djZ3ctkvHjx8ovcvHeiERozIrJy1mzdP5uIH/rXI2OZB8CV5v8uADFx9XSnxksrgWSzOdmslGX/4lCxXd/Rcb60psA3dJzHGDawBQe40vzWAYCoXKskjpdXQ9UBQbSu4BhzDAQ20E0SQGSiO+jpFjBzEuaCkuyeHUg8uPwC6DBnBgxbcIArzW/TgGigJHNm3Hw9V4enjydyFB8r8A0d5sJxRViJcR80Jm7C9svtGwNAaZEdnW/tVZOyE9AxCKyFDQMA19RNyOxZdtN5obihSAeBtazCzE3IjmbZ6azd78iHDBX4pp7bwZNjHcBPPYBjhn0FKfabmdM5wUpgDdbCBrbWPeA8AOdb8Bea0O0xtLmIwgVfRG6v4mkeI2QbzvExcv4cu/0hcf5L5sXLv5VPJmjtwse2JuwAAAAASUVORK5CYII=',
    48: 'iVBORw0KGgoAAAANSUhEUgAAADAAAAAwCAYAAABXAvmHAAACeUlEQVR42u2Yz0tUURTH/RP8C9xIQRsXgrRwkamIoNAYBBISCkOgqIgYEhUMOLsRpB9EpTUaYiFuFFyqi3Zt3LhqUQvBjYvoLzjN58EZTm/uG3ScB97XPXCYN++d+/1+z7nn3vejpSVYsGDBggX7X2zrdKO14uMVL6TkYLemJR7w3xWXlB2O8YZEvn6/Jy5/U949Bnj5sCSDY/el595QjT96lpfF8lN5vvWirhNDrAsDbDjggjNJz6UT+LC/E1UHgra2jn+8s/uO5JcmZfbVvEyWZiT3eFRu9/ZF520M57hGDLGMsTHqcMAFZ9MSWDvcjkCpkiUbGM1Vhd8dGa4Rk+TEaiJg2GtwwAVnqglQTQQ8fDIh7Tc6LyxenTGMBQOsZiXQ4/LPJ5tlm4BW3hKrd3R1y4OZMVlYXaz2Pcec41o8XguhM6EJwJmkp5EdqKAJ0LdaeSvk5q0uyRenZPXkXV0nhlg7VmcCbE0AzmZuodUEWHz0r20bBBX3ipHA5YNSTbV1VrhGDLE2CbDABPtKCVSm58jln759/QUo2x+VsgvWip9emaupbnyWiHElASbYcMAFZ5KehhcxeziVsqK0bRB20QWsSTDWngcbjtR2IRakXbi0hrZNvcq7ZkLbybYa2HCkmgA3JCWkrxHBb1xk/2BOXn58GznH8euusWBfNYGCy9e/fznSBOzdk+0xXkUV//PPDzmXs8g5jiehsweGvWNrAnAm6Wl4FwLciuA/IuLVpeoqXp1z8TjGujBT20avfQLet1AWFnG2t1EfbmR+P0p4/zDn/eO09y80WXilzOZLvfefVbz5sOX9p8VMf9wNFixYsGDBrqP9BTRbGHOaFvscAAAAAElFTkSuQmCC',
}

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

try:
    import winsound
except ImportError:
    winsound = None

# ---------------------------------------------------------------------
# Embedded HSBB/I2C transport
#
# This is the small subset of the former Python_Gain_Sweep communication
# package required by this GUI. Keeping it here makes this script portable
# as a single file.
# ---------------------------------------------------------------------
class _HsbbConstants:
    standard_header = b'\xde\xad\xbe\xef'
    i2c_sel_0_pin_logical_address = b'\x08'
    i2c_sel_1_pin_logical_address = b'\x09'
    tx_i2c_primary = b'\x04\x02\x00'
    tx_rx_i2c_primary = b'\x06\x02\x00'
    rx_i2c_primary = b'\x05\x02\x00'
    set_i2c_primary_address_value = b'\x00\x02\x00\x03'
    init_i2c_primary = b'\x09\x02\x00'
    disable_bridge_messaging = b'\x03\x00\x0e'
    enable_gpio_pin = b'\x02\x01'
    set_gpio_pin_level = b'\x00\x01'
    reply_time_out_time = 0.2


HsbbConstants = _HsbbConstants()


class HsbbAPI:
    def __init__(self, serial_port_obj):
        self.serial_port_obj = serial_port_obj
        self.last_hsbb_packet_sent = b''
        self.last_hsbb_packet_sent_len = 0

    @staticmethod
    def calculate_checksum(payload):
        return ((~sum(payload)) & 0xFFFF).to_bytes(2, 'big')

    @staticmethod
    def create_hsbb_packet(command):
        body = (HsbbConstants.standard_header + len(command).to_bytes(2, 'big')
                + command)
        return body + HsbbAPI.calculate_checksum(body)

    def get_last_hsbb_packet_sent_len(self):
        return self.last_hsbb_packet_sent_len

    def send_command_to_hsbb(self, command, rx_bytes_len=0,
                             print_data=True, reset_cmd=False):
        packet = self.create_hsbb_packet(command)
        self.last_hsbb_packet_sent = packet
        self.last_hsbb_packet_sent_len = len(packet)
        self.serial_port_obj.flush()
        self.serial_port_obj.write(packet)
        expected = len(packet) + rx_bytes_len
        deadline = time.perf_counter() + HsbbConstants.reply_time_out_time
        while time.perf_counter() < deadline:
            if self.serial_port_obj.in_waiting >= expected:
                break
            time.sleep(0.001)
        if reset_cmd:
            reset_deadline = time.perf_counter() + 1
            while time.perf_counter() < reset_deadline:
                if self.serial_port_obj.in_waiting:
                    break
                time.sleep(0.001)
        received = self.serial_port_obj.read(self.serial_port_obj.in_waiting)
        return self.verify_received_data_from_device(received, expected)

    def verify_received_data_from_device(self, received, expected_length):
        start = received.find(HsbbConstants.standard_header)
        if start < 0 or len(received) - start < expected_length:
            return False, b''
        packet = received[start:start + expected_length]
        if len(packet) >= 9 and packet[6:9] == b'\xba\xdb\xad':
            return False, packet
        if len(packet) < 2:
            return False, packet
        if packet[-2:] != self.calculate_checksum(packet[:-2]):
            return False, packet
        return True, packet


class HsbbGPIOCommands:
    def __init__(self, api):
        self.api = api

    def enable_gpio_pin(self, pin, direction):
        return self.api.send_command_to_hsbb(
            HsbbConstants.enable_gpio_pin + pin + direction)

    def set_gpio_pin_level(self, pin, level):
        return self.api.send_command_to_hsbb(
            HsbbConstants.set_gpio_pin_level + pin + level)


class HsbbGenericCommands:
    def __init__(self, api, comm_protocol):
        self.api = api
        self.comm_protocol = comm_protocol

    def disable_bridge_messaging(self):
        return self.api.send_command_to_hsbb(
            HsbbConstants.disable_bridge_messaging)


class HsbbI2CCommands:
    def __init__(self, api, gpio, slave_address):
        self.api = api
        self.gpio = gpio
        self.slave_address = slave_address

    def set_i2c_primary_address_value(self, slave_address):
        return self.api.send_command_to_hsbb(
            HsbbConstants.set_i2c_primary_address_value + slave_address)

    def initialise_i2c_primary(self):
        return self.api.send_command_to_hsbb(HsbbConstants.init_i2c_primary)

    def tx_primary(self, tx_bytes):
        command = (HsbbConstants.tx_i2c_primary
                   + len(tx_bytes).to_bytes(1, 'big') + tx_bytes)
        return self.api.send_command_to_hsbb(command)

    def tx_rx_primary(self, tx_bytes, rx_bytes_count):
        command = (HsbbConstants.tx_rx_i2c_primary
                   + len(tx_bytes).to_bytes(1, 'big') + tx_bytes
                   + rx_bytes_count.to_bytes(1, 'big'))
        success, packet = self.api.send_command_to_hsbb(
            command, rx_bytes_count)
        sent_length = self.api.get_last_hsbb_packet_sent_len()
        data = packet[sent_length - 2:-2] if packet else b''
        return success, data


class HsbbDebugCommands:
    pass


def _show_missing_pyserial_message():
    """Explain how to install the only external Python package we need."""
    root = tk.Tk()
    root.withdraw()
    messagebox.showerror(
        'Missing dependency: pyserial',
        'This GUI needs the pyserial package to access the USB COM port.\n\n'
        'Open Command Prompt or PowerShell and run:\n\n'
        'python -m pip install pyserial\n\n'
        'If your computer has multiple Python versions, use the same Python '
        'command that starts this GUI.\n\n'
        'After installation, start maxtouch_gui.py again.'
    )
    root.destroy()

logging.basicConfig(level=logging.WARNING)

# -------- Protocol constants --------
FAMILY_ID = 0xA9
VARIANT_3380 = 0x00;  VARIANT_3380_ENC = 0x80
VARIANT_3240 = 0x01;  VARIANT_3240_ENC = 0x81

ID_INFO_ADDR = b'\x00\x00'
ID_INFO_SIZE = 7
OBJ_TABLE_ELEM_SIZE = 6
MAX_RX = 240

I2C_ADDRESSES = [b'\x3a', b'\x4a', b'\x4b', b'\x24', b'\x25']
BAUDRATE = 2_000_000

T501_DETECT_MASK = 0x80   # STATUS bit7 = DETECT, same convention as T500/T502/T100

HSBB_HEADER = b'\xde\xad\xbe\xef'


# =====================================================================
#  HSBB port scan
# =====================================================================

def _make_hsbb_probe_pkt():
    """Pre-build the get_bridge_version probe packet."""
    cmd = b'\x01\x00\x01'
    body = HSBB_HEADER + len(cmd).to_bytes(2, 'big') + cmd
    chk = (~sum(body)) & 0xFFFF
    return body + chk.to_bytes(2, 'big')

_PROBE_PKT = _make_hsbb_probe_pkt()


def find_hsbb_port(retries=3):
    """Scan USB serial ports. Returns (port_name, open_serial_obj) or (None, None).

    The serial object is kept open so there is no close-then-reopen race
    that causes PermissionError on Windows.
    """
    usb_ports = [p.device for p in serial.tools.list_ports.comports()
                 if p.vid is not None]
    for attempt in range(retries):
        for port in usb_ports:
            try:
                s = serial.Serial(port=port, baudrate=BAUDRATE, timeout=0.5)
                s.reset_input_buffer()
                s.write(_PROBE_PKT)
                time.sleep(0.3)
                resp = s.read(s.in_waiting or 64)
                if HSBB_HEADER in resp:
                    return port, s       # keep open — no race
                s.close()
            except serial.SerialException:
                pass                     # port busy, try next / retry
        if attempt < retries - 1:
            time.sleep(1)                # wait before retry
    return None, None


# =====================================================================
#  Connection
# =====================================================================

class SliderInfo:
    """Stores config & live state for one T502 slider/wheel instance."""
    __slots__ = ('inst', 'is_wheel', 'enabled', 'startkey', 'numkeys',
                 'range_bits', 'max_pos', 'position', 'amplitude', 'touched',
                 'active_sensor_id')

    def __init__(self, inst, ctrl, startkey, numkeys, res_byte):
        self.inst = inst
        self.is_wheel = ((ctrl >> 6) & 0x03) == 1   # TYPE = bit7:6, 0=Slider, 1=Wheel
        self.enabled = bool(ctrl & 0x01)
        self.startkey = startkey
        self.numkeys = numkeys
        self.range_bits = (res_byte >> 4) & 0x0F
        self.max_pos = (1 << self.range_bits) - 1 if self.range_bits else 0
        self.position = 0
        self.amplitude = 0
        self.touched = False
        self.active_sensor_id = None

    @property
    def label(self):
        kind = "Wheel" if self.is_wheel else "Slider"
        return f"{kind} {self.inst}"


class HsbbConnection:

    def __init__(self):
        self.serial_obj = None
        self.api = self.gpio = self.generic = self.i2c = None
        self.port_name = ''
        self.id_info = {}
        self.device_name = ''
        self.config_mode = None
        self.is_3380 = True
        self.main_obj_table = []
        self.ext_obj_table = []
        self.object_rid_map = []
        self.t100_report_ids = None
        self.t72_report_ids = None
        self.t5_addr = self.t5_size = None
        self.t44_addr = None
        self.t500_report_ids = None
        self.t501_report_ids = None
        self.t502_report_ids = None
        self.t100_report_ids = None
        self.t72_report_ids = None
        self.last_other_report = None
        self.t500_num_keys = 64
        self.enabled_sensor_ids = []
        self.sliders = []          # list[SliderInfo]

    # ---- connect ----

    def connect(self):
        port, ser = find_hsbb_port()
        if port is None:
            raise RuntimeError("No HSBB bridge found.\n"
                               "Close maXTouch Studio and check USB cable.")
        self.port_name = port
        self.serial_obj = ser              # already open from probe
        self.api = HsbbAPI(self.serial_obj)
        self.gpio = HsbbGPIOCommands(self.api)
        self.generic = HsbbGenericCommands(self.api, 'I2C')
        self.generic.disable_bridge_messaging()

        for addr in I2C_ADDRESSES:
            self.i2c = HsbbI2CCommands(self.api, self.gpio, addr)
            self._pullups_on()
            self.i2c.set_i2c_primary_address_value(addr)
            self.i2c.initialise_i2c_primary()
            ok, data = self.i2c.tx_rx_primary(ID_INFO_ADDR, ID_INFO_SIZE)
            if ok and len(data) >= ID_INFO_SIZE and data[0] == FAMILY_ID:
                self._parse_id(data)
                self._build_tables()
                self._read_config_mode()
                self._read_sensor_configs()
                self._read_slider_configs()
                self._prepare_fast_io()
                return self.device_name

        self.close()
        raise RuntimeError("MTCH3380P/3240P not found on I2C bus.")

    def close(self):
        if self.serial_obj and self.serial_obj.is_open:
            try:
                self.serial_obj.close()
            except Exception:
                pass
        self.serial_obj = None

    @property
    def connected(self):
        return self.serial_obj is not None and self.serial_obj.is_open

    # ---- internals ----

    def _pullups_on(self):
        for pin in (HsbbConstants.i2c_sel_0_pin_logical_address,
                    HsbbConstants.i2c_sel_1_pin_logical_address):
            self.gpio.enable_gpio_pin(pin, b'\x00')
            self.gpio.set_gpio_pin_level(pin, b'\x00')

    def _parse_id(self, data):
        names = ['Family ID', 'Variant ID', 'Version', 'Build',
                 'Matrix X Size', 'Matrix Y Size', 'Number of Object Table Elements']
        self.id_info = {n: data[i] for i, n in enumerate(names)}
        v = self.id_info['Variant ID']
        if v in (VARIANT_3380, VARIANT_3380_ENC):
            self.device_name, self.is_3380, self.t500_num_keys = 'ATMXT3380P', True, 64
        elif v in (VARIANT_3240, VARIANT_3240_ENC):
            self.device_name, self.is_3380, self.t500_num_keys = 'ATMXT3240P', False, 32
        else:
            self.device_name = f'MTCH(0x{v:02X})'

    def _read_bytes(self, addr, count):
        buf = b''
        while count > 0:
            n = min(count, MAX_RX)
            ok, d = self.i2c.tx_rx_primary(addr.to_bytes(2, 'little'), n)
            if not ok:
                raise RuntimeError(f"I2C read @0x{addr:04X}")
            buf += d;  addr += n;  count -= n
        return buf

    def _build_tables(self):
        ne = self.id_info['Number of Object Table Elements']
        raw = self._read_bytes(0x0007, ne * OBJ_TABLE_ELEM_SIZE)
        self.main_obj_table = []
        t254_addr = t254_size = None
        for i in range(ne):
            e = raw[i*6:(i+1)*6]
            ot, sa = e[0], int.from_bytes(e[1:3], 'little')
            sz, inst, rids = e[3]+1, e[4]+1, e[5]
            self.main_obj_table.append((ot, sa, sz, inst, rids))
            if   ot == 5:   self.t5_addr, self.t5_size = sa, sz
            elif ot == 44:  self.t44_addr = sa
            elif ot == 254: t254_addr, t254_size = sa, sz

        self.ext_obj_table = []
        if t254_addr is not None:
            ext = self._read_bytes(t254_addr, t254_size)
            for i in range((t254_size - 3) // 7):
                e = ext[i*7:(i+1)*7]
                ot = int.from_bytes(e[0:2], 'little')
                sa = int.from_bytes(e[2:4], 'little')
                sz, inst, rids = e[4]+1, e[5]+1, e[6]
                if ot > 0:
                    self.ext_obj_table.append((ot, sa, sz, inst, rids))

        self.object_rid_map = []   # every object, incl. those w/o report IDs
        rid = 1
        for ot, sa, sz, inst, rp in self.main_obj_table:
            lo, hi = (rid, rid + inst * rp - 1) if rp > 0 else (None, None)
            self.object_rid_map.append((ot, sa, sz, inst, rp, lo, hi))
            if rp > 0:
                if ot == 72:
                    self.t72_report_ids = (lo, hi)
                rid += inst * rp
        for ot, sa, sz, inst, rp in self.ext_obj_table:
            lo, hi = (rid, rid + inst * rp - 1) if rp > 0 else (None, None)
            self.object_rid_map.append((ot, sa, sz, inst, rp, lo, hi))
            if rp > 0:
                if   ot == 500: self.t500_report_ids = (lo, hi)
                elif ot == 501: self.t501_report_ids = (lo, hi)
                elif ot == 502: self.t502_report_ids = (lo, hi)
                elif ot == 100: self.t100_report_ids = (lo, hi)
                rid = hi + 1

    def _read_config_mode(self):
        """Detect SELF versus MUTUAL from T500 CTRL bits 3:2."""
        self.config_mode = None
        t500 = next((obj for obj in self.main_obj_table + self.ext_obj_table
                     if obj[0] == 500), None)
        if t500 is None:
            return
        start_address = t500[1]
        data = self._read_bytes(start_address, 1)
        if data:
            mode_bits = (data[0] >> 2) & 0x03
            self.config_mode = 'SELF' if mode_bits & 0x01 else 'MUTUAL'

    def _read_sensor_configs(self):
        """Read T501 and retain the physical sensors whose CTRL is enabled."""
        self.enabled_sensor_ids = []
        t501 = next((obj for obj in self.ext_obj_table if obj[0] == 501), None)
        if t501 is None:
            return
        start_address, object_size, instances = t501[1:4]
        raw = self._read_bytes(start_address, object_size * instances)
        for index in range(instances):
            data = raw[index * object_size:(index + 1) * object_size]
            if data and (data[0] & 0x01):
                self.enabled_sensor_ids.append(index)

    def _read_slider_configs(self):
        """Read enabled T502 instances and their sensor key ranges."""
        self.sliders = []
        t502 = None
        for ot, sa, sz, inst, rp in self.ext_obj_table:
            if ot == 502:
                t502 = (sa, sz, inst);  break
        if t502 is None:
            return
        sa, sz, inst = t502
        raw = self._read_bytes(sa, sz * inst)
        for i in range(inst):
            d = raw[i*sz:(i+1)*sz]
            if d and (d[0] & 0x01):
                self.sliders.append(SliderInfo(i, d[0], d[1], d[2], d[3]))

    def write_config(self, configuration_file):
        """Write the object data from an XCFG file to the connected device."""
        if not self.connected:
            raise RuntimeError("Device is not connected.")

        with open(configuration_file, 'r', encoding='utf-8') as config_file:
            config_text = config_file.read()
        comments_end = config_text.find('[VERSION_INFO_HEADER]')
        if comments_end > 0:
            config_text = config_text[comments_end:]

        parser = configparser.ConfigParser()
        parser.read_string(config_text)
        objects = self.main_obj_table + self.ext_obj_table
        object_map = {obj[0]: obj for obj in objects}

        for section in parser.sections():
            match = re.search(r'T(\d+)\s+INSTANCE\s+(\d+)', section)
            if not match:
                continue
            object_type, instance = (int(value) for value in match.groups())
            device_object = object_map.get(object_type)
            if device_object is None:
                continue
            start_address, object_size, instances = device_object[1:4]
            if instance >= instances:
                continue

            items = parser.items(section)
            config_size = int(dict(items).get('object_size', object_size))
            data = bytearray()
            for key, value in items:
                if key in ('object_address', 'object_size'):
                    continue
                field = re.search(r'(\d+)\s+(\d+)', key)
                if not field:
                    continue
                field_size = int(field.group(2))
                data.extend(int(value, 0).to_bytes(field_size, 'little'))

            data = data[:min(object_size, config_size)]
            address = start_address + instance * object_size
            success, _ = self.i2c.tx_primary(
                address.to_bytes(2, 'little') + bytes(data))
            if not success:
                raise RuntimeError(
                    f"Failed to write T{object_type} instance {instance}.")

        # T6 BACKUPNV is byte 1; 0x55 stores the new configuration in NVM.
        t6 = next((obj for obj in objects if obj[0] == 6), None)
        if t6 is None or t6[2] < 2:
            raise RuntimeError("T6 BACKUPNV command is unavailable.")
        t6_data = bytearray(t6[2])
        t6_data[1] = 0x55
        success, _ = self.i2c.tx_primary(
            t6[1].to_bytes(2, 'little') + bytes(t6_data))
        if not success:
            raise RuntimeError("Failed to back up configuration.")

    # ---- fast I/O ----

    def _build_hsbb_pkt(self, cmd):
        body = HSBB_HEADER + len(cmd).to_bytes(2, 'big') + cmd
        return body + ((~sum(body)) & 0xFFFF).to_bytes(2, 'big')

    def _build_txrx_pkt(self, addr_bytes, rx_count):
        tx_len = len(addr_bytes).to_bytes(1, 'big')
        rx_len = rx_count.to_bytes(1, 'big')
        cmd = b'\x06\x02\x00' + tx_len + addr_bytes + rx_len
        pkt = self._build_hsbb_pkt(cmd)
        echo = len(pkt)
        return pkt, echo

    def _prepare_fast_io(self):
        self._t44_pkt, self._t44_echo = self._build_txrx_pkt(
            self.t44_addr.to_bytes(2, 'little'), 1)
        self._t44_expect = self._t44_echo + 1
        self._t5_pkt, self._t5_echo = self._build_txrx_pkt(
            self.t5_addr.to_bytes(2, 'little'), self.t5_size)
        self._t5_expect = self._t5_echo + self.t5_size

    def _fast_xfer(self, pkt, expect, timeout_ms=20):
        ser = self.serial_obj
        ser.write(pkt)
        deadline = time.perf_counter() + timeout_ms / 1000.0
        while time.perf_counter() < deadline:
            if ser.in_waiting >= expect:
                break
            time.sleep(0.0005)
        return ser.read(ser.in_waiting) if ser.in_waiting else b''

    def read_msg_fast(self):
        if not self.connected:
            return 0xFF, None
        try:
            resp = self._fast_xfer(self._t5_pkt, self._t5_expect)
            idx = resp.find(HSBB_HEADER)
            if idx == -1:
                return 0xFF, None
            if len(resp) >= idx + 9 and resp[idx+6:idx+9] == b'\xba\xdb\xad':
                return 0xFF, None
            ds = idx + self._t5_echo - 2
            de = ds + self.t5_size
            if len(resp) >= de + 2:
                d = resp[ds:de]
                return d[0], d[1:self.t5_size - 1]
        except Exception:
            pass
        return 0xFF, None

    # ---- message helpers ----

    def is_t500(self, rid):
        r = self.t500_report_ids
        return r and r[0] <= rid <= r[1]

    def is_t501(self, rid):
        r = self.t501_report_ids
        return r and r[0] <= rid <= r[1]

    def is_t502(self, rid):
        r = self.t502_report_ids
        return r and r[0] <= rid <= r[1]

    def is_t100(self, rid):
        r = self.t100_report_ids
        return r and r[0] <= rid <= r[1]

    def is_t72(self, rid):
        r = self.t72_report_ids
        return r and r[0] <= rid <= r[1]

    def t501_key_id(self, rid):
        return rid - self.t501_report_ids[0] if self.t501_report_ids else -1

    def t502_instance(self, rid):
        return rid - self.t502_report_ids[0] if self.t502_report_ids else -1

    def t502_sensor_id(self, slider, position):
        """Map T502's continuous position to the closest configured sensor."""
        if slider.numkeys <= 0:
            return None
        if slider.numkeys == 1 or slider.max_pos <= 0:
            return slider.startkey
        position = max(0, min(position, slider.max_pos))
        offset = round(position * (slider.numkeys - 1) / slider.max_pos)
        return slider.startkey + offset

    @staticmethod
    def parse_t500_keys(payload):
        pressed = set()
        if len(payload) < 2 or not (payload[0] & 0x80):
            return pressed
        for bi in range(1, len(payload)):
            v = payload[bi]
            for bit in range(8):
                if v & (1 << bit):
                    pressed.add((bi - 1) * 8 + bit)
        return pressed

    @staticmethod
    def parse_t502(payload):
        """Return (touched, position, amplitude)."""
        if len(payload) < 5:
            return False, 0, 0
        status = payload[0]
        # T502 STATUS: TCHDET is bit 7; the lower bits are change/status flags.
        touched = bool(status & 0x80)
        pos = payload[1] | (payload[2] << 8)
        amp = payload[3] | (payload[4] << 8)
        return touched, pos, amp

    def slider_for_instance(self, inst_idx):
        for si in self.sliders:
            if si.inst == inst_idx:
                return si
        return None


# =====================================================================
#  GUI
# =====================================================================

DARK_BG  = '#1e1e2e';  PANEL_BG = '#181825';  BAR_BG = '#313244'
GREEN    = '#a6e3a1';   RED     = '#f38ba8';   YELLOW = '#f9e2af'
BLUE     = '#89b4fa';   MAUVE   = '#cba6f7';   PEACH  = '#fab387'
TEXT_FG  = '#cdd6f4';   DIM_FG  = '#6c7086'
KEY_OFF_BG = '#313244'; KEY_ON_BG = '#a6e3a1'
KEY_OFF_FG = '#6c7086'; KEY_ON_FG = '#1e1e2e'


def _hue_to_hex(hue, sat=0.85, val=1.0):
    """Map a hue in [0,1) to a rainbow RGB hex color."""
    r, g, b = colorsys.hsv_to_rgb(hue % 1.0, sat, val)
    return f'#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}'


def _make_tone_wav(freq, seconds=1.0, sample_rate=44100, volume=0.3):
    """Synthesize a mono 16-bit PCM sine tone, trimmed to a whole number of
    cycles so it loops back-to-back with no click (SND_LOOP-friendly)."""
    cycles = max(1, round(freq * seconds))
    duration = cycles / freq
    n_samples = int(sample_rate * duration)
    amp = int(32767 * volume)
    frames = bytearray()
    for i in range(n_samples):
        t = i / sample_rate
        sample = int(amp * math.sin(2 * math.pi * freq * t))
        frames += struct.pack('<h', sample)
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(bytes(frames))
    return buf.getvalue()


def _blend_hex(fg, bg, t):
    """Blend fg toward bg by t in [0,1] -- fakes alpha on a canvas with no alpha channel."""
    fr, fgc, fb = int(fg[1:3], 16), int(fg[3:5], 16), int(fg[5:7], 16)
    br, bgc, bb = int(bg[1:3], 16), int(bg[3:5], 16), int(bg[5:7], 16)
    r = round(fr + (br - fr) * t)
    g = round(fgc + (bgc - fgc) * t)
    b = round(fb + (bb - fb) * t)
    return f'#{r:02x}{g:02x}{b:02x}'


class SliderWidget:
    """Canvas widget showing a slider bar or wheel arc."""

    WHEEL_SEGMENTS = 48
    ANIM_MS = 40

    def __init__(self, parent, slider_info: SliderInfo, width=220):
        self.si = slider_info
        self.w = width
        self.h = 200 if slider_info.is_wheel else 100
        self.canvas = tk.Canvas(parent, width=self.w, height=self.h,
                                bg=PANEL_BG, highlightthickness=0)
        self.canvas.pack(padx=4, pady=4)
        self._position = 0
        self._amplitude = 0
        self._touched = False
        self._anim_phase = 0.0
        self._render()
        self.canvas.after(self.ANIM_MS, self._animate)

    def _animate(self):
        if not self.canvas.winfo_exists():
            return
        if self._touched:
            self._anim_phase = (self._anim_phase + 0.18) % (2 * math.pi)
            self._render()
        self.canvas.after(self.ANIM_MS, self._animate)

    def update(self, position, amplitude, touched):
        self._position = position
        self._amplitude = amplitude
        self._touched = touched
        self._render()

    def _render(self):
        self.canvas.delete('all')
        lbl = self.si.label
        color = GREEN if self._touched else DIM_FG
        self.canvas.create_text(self.w // 2, 12, text=lbl,
                                font=("Consolas", 10, "bold"),
                                fill=color, anchor='center')
        if self.si.is_wheel:
            self._draw_wheel(self._position, self._touched)
        else:
            self._draw_bar(self._position, self._touched)

        if self._touched:
            info = f"pos: {self._position}    amp: {self._amplitude}"
        else:
            info = "pos: --    amp: --"
        self.canvas.create_text(self.w // 2, self.h - 8,
                                text=info,
                                font=("Consolas", 9), fill=TEXT_FG,
                                anchor='center')

    def _draw_bar(self, position, touched):
        """Draw a horizontal slider bar; the filled portion sweeps a rainbow gradient."""
        margin = 20
        bar_y = self.h // 2 + 2
        bar_h = 14
        x0, x1 = margin, self.w - margin
        track_w = x1 - x0
        # track
        self.canvas.create_rectangle(x0, bar_y - bar_h//2,
                                     x1, bar_y + bar_h//2,
                                     fill=BAR_BG, outline=DIM_FG, width=1)
        if not (touched and self.si.max_pos > 0):
            return

        frac = max(0.0, min(1.0, position / self.si.max_pos))
        knob_x = x0 + frac * track_w

        # filled portion: rainbow gradient, hue tied to absolute position on the track
        step = 2
        x = x0
        while x < knob_x:
            sx1 = min(knob_x, x + step)
            hue = ((x - x0) / track_w) * 0.8
            self.canvas.create_rectangle(x, bar_y - bar_h//2 + 2,
                                         sx1, bar_y + bar_h//2 - 2,
                                         fill=_hue_to_hex(hue), outline='')
            x += step

        # glowing knob matching the local gradient color, with a pulsing halo
        pulse = 0.5 + 0.5 * math.sin(self._anim_phase)
        head_color = _hue_to_hex(frac * 0.8)
        for radius, t in ((17, 0.82), (12, 0.55)):
            glow_r = radius * (0.85 + 0.3 * pulse)
            blended = _blend_hex(head_color, PANEL_BG, t)
            self.canvas.create_oval(knob_x - glow_r, bar_y - glow_r,
                                    knob_x + glow_r, bar_y + glow_r,
                                    fill=blended, outline='')
        r = 8 + 1.5 * pulse
        self.canvas.create_oval(knob_x - r, bar_y - r,
                                knob_x + r, bar_y + r,
                                fill='#ffffff', outline=head_color, width=2)

    def _draw_wheel(self, position, touched):
        """Draw a circular wheel indicator as a ring that lights up, LED by LED,
        in rainbow color as the touch position sweeps around it."""
        cx, cy = self.w // 2, self.h // 2
        r_out, r_in = 74, 46
        r_mid = (r_out + r_in) / 2
        ring_w = r_out - r_in

        # unlit track (always visible, subtle)
        self.canvas.create_oval(cx - r_out, cy - r_out, cx + r_out, cy + r_out,
                                outline='#2a2a3d', width=ring_w)
        self.canvas.create_oval(cx - r_in, cy - r_in, cx + r_in, cy + r_in,
                                outline=DIM_FG, width=1)

        if not (touched and self.si.max_pos > 0):
            return

        frac = position / (self.si.max_pos + 1)
        n_lit = max(1, int(round(frac * self.WHEEL_SEGMENTS)))
        seg_deg = 360.0 / self.WHEEL_SEGMENTS

        # pass 1: soft halo behind every lit segment (wide, dim -> bloom)
        for i in range(n_lit):
            hue = i / self.WHEEL_SEGMENTS
            halo = _blend_hex(_hue_to_hex(hue), PANEL_BG, 0.55)
            start_deg = 90 - i * seg_deg
            self.canvas.create_arc(cx - r_out - 5, cy - r_out - 5,
                                   cx + r_out + 5, cy + r_out + 5,
                                   start=start_deg, extent=-seg_deg * 0.88,
                                   style='arc', outline=halo, width=ring_w + 12)

        # pass 2: bright rainbow core on top of the halo
        for i in range(n_lit):
            hue = i / self.WHEEL_SEGMENTS
            start_deg = 90 - i * seg_deg
            self.canvas.create_arc(cx - r_out, cy - r_out, cx + r_out, cy + r_out,
                                   start=start_deg, extent=-seg_deg * 0.88,
                                   style='arc', outline=_hue_to_hex(hue), width=ring_w - 6)

        # pulsing glow head (comet tip) at the current touch position
        angle = 2 * math.pi * frac
        hx = cx + r_mid * math.sin(angle)
        hy = cy - r_mid * math.cos(angle)
        pulse = 0.5 + 0.5 * math.sin(self._anim_phase)
        head_color = _hue_to_hex(frac, sat=0.35, val=1.0)
        for radius, t in ((24, 0.85), (17, 0.6), (11, 0.3)):
            glow_r = radius * (0.85 + 0.35 * pulse)
            blended = _blend_hex(head_color, PANEL_BG, t)
            self.canvas.create_oval(hx - glow_r, hy - glow_r,
                                    hx + glow_r, hy + glow_r,
                                    fill=blended, outline='')
        core_r = 9 + 2 * pulse
        self.canvas.create_oval(hx - core_r, hy - core_r,
                                hx + core_r, hy + core_r,
                                fill='#ffffff', outline=head_color, width=2)


class MaxTouchGUI:
    def __init__(self):
        self.conn = HsbbConnection()
        self.polling = False
        self.poll_thread = None
        self.pressed_keys = set()
        self.key_amplitudes = {}
        self.t502_active_sensor_ids = set()
        self.key_labels = []
        self.sensor_labels = {}
        self.slider_widgets = {}   # inst_idx → SliderWidget
        self.config_buttons = []
        self.current_config_mode = None
        self.amp_window = None
        self.amp_list_frame = None
        self.amp_debug_text = None
        self.t501_raw = {}          # kid -> (status_byte, payload_hex)
        self.t501_log = deque(maxlen=15)

        self.keypattern_window = None
        self.keypattern_canvas = None
        self.keypattern_shapes = {}
        self.keypattern_prev = set()
        self.note_stack = []       # held pad keys, most-recent last (mono voice)
        self.note_wavs = {}        # key_id -> path of synthesized loop WAV file
        self.note_tmp_dir = None

        self.about_window = None

        self._build_ui()

    # ---- UI ----

    def _set_window_icon(self):
        """Vector-drawn touch-chip icon: a proper multi-size .ico next to
        the script for the Windows taskbar/title bar, with the embedded
        base64 PNGs as a self-contained fallback (also covers non-Windows
        Tk builds, where iconbitmap(.ico) isn't supported)."""
        ico_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'app_icon.ico')
        try:
            if os.path.exists(ico_path):
                self.root.iconbitmap(default=ico_path)
        except Exception:
            pass
        try:
            self._icon_imgs = [tk.PhotoImage(data=b64)
                               for b64 in _ICON_PNG_B64.values()]
            self.root.iconphoto(True, *self._icon_imgs)
        except Exception:
            pass

    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title("Microchip MTCH3380P / MTCH3240P Monitor")
        self.root.geometry("960x620")
        self.root.configure(bg=DARK_BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._set_window_icon()

        # top bar
        bar = tk.Frame(self.root, bg=BAR_BG, height=50)
        bar.pack(fill=tk.X); bar.pack_propagate(False)
        self.status_lbl = tk.Label(bar, text="Not Connected",
                                   font=("Consolas", 16, "bold"),
                                   fg=RED, bg=BAR_BG, anchor='w', padx=16)
        self.status_lbl.pack(fill=tk.BOTH, expand=True)

        # main
        main = tk.Frame(self.root, bg=DARK_BG)
        main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left column: sensors
        left = tk.LabelFrame(main, text=" Sensors ",
                             font=("Consolas", 11, "bold"),
                             fg=TEXT_FG, bg=DARK_BG, labelanchor='n',
                             bd=2, relief='groove')
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        self.sensor_grid = tk.Frame(left, bg=DARK_BG)
        self.sensor_grid.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)

        # Middle column: sliders / wheels
        self.slider_frame = tk.LabelFrame(main, text=" Sliders / Wheels ",
                                          font=("Consolas", 11, "bold"),
                                          fg=TEXT_FG, bg=DARK_BG, labelanchor='n',
                                          bd=2, relief='groove', width=260)
        self.slider_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=5)
        self.slider_frame.pack_propagate(False)

        self.slider_inner = tk.Frame(self.slider_frame, bg=DARK_BG)
        self.slider_inner.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        self.slider_placeholder = tk.Label(self.slider_inner, text="--",
                                           font=("Consolas", 12),
                                           fg=DIM_FG, bg=DARK_BG)
        self.slider_placeholder.pack(expand=True)

        # Keys column: independent key sensors (T500/T501).
        key_panel = tk.LabelFrame(main, text=" Keys ",
                                  font=("Consolas", 11, "bold"),
                                  fg=TEXT_FG, bg=DARK_BG, labelanchor='n',
                                  bd=2, relief='groove', width=260)
        key_panel.pack(side=tk.LEFT, fill=tk.BOTH, padx=5)
        key_panel.pack_propagate(False)
        self.keys_text = tk.Label(key_panel, text="--",
                                  font=("Consolas", 20, "bold"),
                                  fg=GREEN, bg=DARK_BG,
                                  wraplength=240, justify='center')
        self.keys_text.pack(pady=(6, 2))
        self.grid_frame = tk.Frame(key_panel, bg=DARK_BG)
        self.grid_frame.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)

        # Right column: info
        right = tk.LabelFrame(main, text=" Info ",
                              font=("Consolas", 11, "bold"),
                              fg=TEXT_FG, bg=DARK_BG, labelanchor='n',
                              bd=2, relief='groove', width=220)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(5, 0))
        right.pack_propagate(False)
        info_scroll = tk.Scrollbar(right)
        info_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.info_txt = tk.Text(right, font=("Consolas", 9),
                                fg='#bac2de', bg=PANEL_BG,
                                bd=0, highlightthickness=0,
                                wrap='word', state='disabled',
                                yscrollcommand=info_scroll.set)
        self.info_txt.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        info_scroll.config(command=self.info_txt.yview)

        # bottom
        bot = tk.Frame(self.root, bg=DARK_BG)
        bot.pack(fill=tk.X, padx=10, pady=(0, 10))
        self.btn = tk.Button(bot, text="Connect ATMXT3380P/3240P",
                             font=("Consolas", 13, "bold"),
                             fg='#1e1e2e', bg=BLUE,
                             activebackground='#74c7ec',
                             activeforeground='#1e1e2e',
                             relief='flat', padx=20, pady=8,
                             command=self._on_btn)
        self.btn.pack(side=tk.LEFT)
        self.amp_btn = tk.Button(bot, text="Key Amplitudes",
                                 font=("Consolas", 11, "bold"),
                                 fg='#1e1e2e', bg=PEACH,
                                 activebackground='#fab387',
                                 activeforeground='#1e1e2e',
                                 relief='flat', padx=14, pady=8,
                                 command=self._open_amplitude_window)
        self.amp_btn.pack(side=tk.LEFT, padx=(10, 0))
        self.keypad_btn = tk.Button(bot, text="Touch Key Pad",
                                    font=("Consolas", 11, "bold"),
                                    fg='#1e1e2e', bg=MAUVE,
                                    activebackground='#f5c2e7',
                                    activeforeground='#1e1e2e',
                                    relief='flat', padx=14, pady=8,
                                    command=self._open_keypattern_window)
        self.keypad_btn.pack(side=tk.LEFT, padx=(10, 0))
        self.about_btn = tk.Button(bot, text="About",
                                   font=("Consolas", 11, "bold"),
                                   fg='#1e1e2e', bg=BLUE,
                                   activebackground='#74c7ec',
                                   activeforeground='#1e1e2e',
                                   relief='flat', padx=14, pady=8,
                                   command=self._open_about_window)
        self.about_btn.pack(side=tk.LEFT, padx=(10, 0))
        self.log_lbl = tk.Label(bot, text="", font=("Consolas", 9),
                                fg=DIM_FG, bg=DARK_BG, anchor='w')
        self.log_lbl.pack(side=tk.LEFT, padx=16, fill=tk.X, expand=True)

        self.config_frame = tk.Frame(bot, bg=DARK_BG)
        self.config_frame.pack(side=tk.RIGHT)
        self.config_status_lbl = tk.Label(
            self.config_frame, text="Config: --", font=("Consolas", 10, "bold"),
            fg=DIM_FG, bg=DARK_BG)
        self.config_status_lbl.pack(side=tk.LEFT, padx=(6, 0))
        self._set_config_buttons_visible(False)

    def run(self):
        self.root.mainloop()

    # ---- key grid ----

    def _node_group(self, index):
        """Return the logical group for one enabled T501 sensor."""
        for slider in self.conn.sliders:
            if slider.startkey <= index < slider.startkey + slider.numkeys:
                return 'W' if slider.is_wheel else 'S'
        return 'K'

    def _node_label(self, index):
        return f'{self._node_group(index)}{index}'

    def _independent_key_ids(self):
        return [index for index in self.conn.enabled_sensor_ids
                if self._node_group(index) == 'K']

    def _build_sensor_grid(self):
        """Show each physical node exactly once with its logical group."""
        for widget in self.sensor_grid.winfo_children():
            widget.destroy()
        self.sensor_labels = {}
        for index in self.conn.enabled_sensor_ids:
            node = self._node_label(index)
            label = tk.Label(self.sensor_grid, text=node,
                             font=("Consolas", 8, "bold"), width=4,
                             fg=KEY_OFF_FG, bg=KEY_OFF_BG)
            label.grid(row=index // 8, column=index % 8,
                       padx=2, pady=2, sticky='nsew')
            self.sensor_labels[node] = label
        for column in range(8):
            self.sensor_grid.columnconfigure(column, weight=1)

    def _set_sensor_node_state(self, node, active):
        label = self.sensor_labels.get(node)
        if label:
            label.configure(bg=KEY_ON_BG if active else KEY_OFF_BG,
                            fg=KEY_ON_FG if active else KEY_OFF_FG)

    def _build_key_grid(self, key_ids):
        for w in self.grid_frame.winfo_children():
            w.destroy()
        self.key_labels = []
        cols = 8
        for r in range((len(key_ids) + cols - 1) // cols):
            for c in range(cols):
                position = r * cols + c
                if position >= len(key_ids):
                    break
                lbl = tk.Label(self.grid_frame, text=str(key_ids[position]),
                               font=("Consolas", 9, "bold"), width=4, height=1,
                               fg=KEY_OFF_FG, bg=KEY_OFF_BG, relief='flat', bd=1)
                lbl.grid(row=r, column=c, padx=2, pady=2, sticky='nsew')
                self.key_labels.append(lbl)
        for c in range(cols):
            self.grid_frame.columnconfigure(c, weight=1)

    def _update_key_grid(self, pressed):
        for i, lbl in enumerate(self.key_labels):
            if i in pressed:
                lbl.configure(bg=KEY_ON_BG, fg=KEY_ON_FG)
            else:
                lbl.configure(bg=KEY_OFF_BG, fg=KEY_OFF_FG)

    # ---- slider widgets ----

    def _build_slider_widgets(self):
        for w in self.slider_inner.winfo_children():
            w.destroy()
        self.slider_widgets = {}

        sliders = self.conn.sliders
        if not sliders:
            tk.Label(self.slider_inner, text="No slider/wheel\nconfigured",
                     font=("Consolas", 10), fg=DIM_FG, bg=DARK_BG
                     ).pack(expand=True)
            return

        for si in sliders:
            sw = SliderWidget(self.slider_inner, si, width=240)
            self.slider_widgets[si.inst] = sw

    def _update_slider(self, inst_idx, position, amplitude, touched):
        sw = self.slider_widgets.get(inst_idx)
        if sw:
            sw.update(position, amplitude, touched)

    # ---- helpers ----

    def _set_status(self, txt, color=RED):
        self.status_lbl.configure(text=txt, fg=color)

    def _show_keys(self, keys):
        independent_keys = set(self._independent_key_ids())
        visible_keys = keys & independent_keys
        if visible_keys:
            self.keys_text.configure(
                text='  '.join(
                    f'Key {k}' for k in sorted(visible_keys)))
        else:
            self.keys_text.configure(text='--')
        self._update_key_grid(visible_keys)
        self._render_sensor_states()
        self._update_amplitude_window(keys)
        self._update_keypattern_window(keys)

    # ---- key amplitude window ----

    def _open_amplitude_window(self):
        if self.amp_window and self.amp_window.winfo_exists():
            self.amp_window.lift()
            self.amp_window.focus_force()
            return
        win = tk.Toplevel(self.root)
        win.title("Key Touch Amplitude")
        win.geometry("360x560")
        win.configure(bg=DARK_BG)
        win.protocol("WM_DELETE_WINDOW", win.destroy)

        header = tk.Label(win, text="Pressed Key Amplitude",
                          font=("Consolas", 13, "bold"),
                          fg=TEXT_FG, bg=BAR_BG, pady=8)
        header.pack(fill=tk.X)

        self.amp_list_frame = tk.Frame(win, bg=DARK_BG)
        self.amp_list_frame.pack(fill=tk.BOTH, expand=False, padx=4, pady=4)

        debug_header = tk.Label(win, text="Raw T501 messages (debug)",
                                font=("Consolas", 10, "bold"),
                                fg=DIM_FG, bg=BAR_BG, pady=4)
        debug_header.pack(fill=tk.X, pady=(8, 0))
        self.amp_debug_text = tk.Text(win, font=("Consolas", 9),
                                      fg='#bac2de', bg=PANEL_BG,
                                      bd=0, highlightthickness=0,
                                      wrap='none', state='disabled')
        self.amp_debug_text.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        self.amp_window = win
        self._update_amplitude_window(self.pressed_keys)
        self._refresh_t501_debug()

    def _update_amplitude_window(self, keys):
        """Rebuild the amplitude window's rows for the current pressed-key set."""
        if not (self.amp_window and self.amp_window.winfo_exists()):
            return
        for widget in self.amp_list_frame.winfo_children():
            widget.destroy()

        if not keys:
            tk.Label(self.amp_list_frame, text="(no key pressed)",
                    font=("Consolas", 11), fg=DIM_FG, bg=DARK_BG
                    ).pack(pady=24)
            return

        bar_w = 260
        for kid in sorted(keys):
            amp = self.key_amplitudes.get(kid)
            row = tk.Frame(self.amp_list_frame, bg=PANEL_BG)
            row.pack(fill=tk.X, padx=8, pady=4)

            top = tk.Frame(row, bg=PANEL_BG)
            top.pack(fill=tk.X, padx=8, pady=(6, 2))
            tk.Label(top, text=self._node_label(kid),
                    font=("Consolas", 12, "bold"),
                    fg=GREEN, bg=PANEL_BG, anchor='w').pack(side=tk.LEFT)
            amp_text = str(amp) if amp is not None else 'N/A'
            tk.Label(top, text=amp_text, font=("Consolas", 12, "bold"),
                    fg=PEACH, bg=PANEL_BG, anchor='e').pack(side=tk.RIGHT)

            bar = tk.Canvas(row, width=bar_w, height=10, bg=BAR_BG,
                            highlightthickness=0)
            bar.pack(padx=8, pady=(0, 8))
            if amp is not None:
                frac = max(0.0, min(1.0, amp / 65535))
                bar.create_rectangle(0, 0, frac * bar_w, 10,
                                     fill=PEACH, outline='')

            raw = self.t501_raw.get(kid)
            raw_text = (f"T501 status=0x{raw[0]:02X}  raw={raw[1]}"
                       if raw else "T501: never received for this node")
            tk.Label(row, text=raw_text, font=("Consolas", 8),
                    fg=DIM_FG, bg=PANEL_BG, anchor='w'
                    ).pack(fill=tk.X, padx=8, pady=(0, 6))

    def _refresh_t501_debug(self):
        """Mirror the recent raw T501 messages into the debug text box."""
        if not (self.amp_window and self.amp_window.winfo_exists()):
            return
        self.amp_debug_text.configure(state='normal')
        self.amp_debug_text.delete('1.0', tk.END)
        self.amp_debug_text.insert('1.0', '\n'.join(self.t501_log))
        self.amp_debug_text.configure(state='disabled')

    # ---- touch key pad (9-key map + note playback) ----

    KEY_PAD_LAYOUT = {
        # key_id: (shape, x0, y0, x1, y1)
        15: ('rect',   133,  33, 222,  91),
        14: ('circle',  48, 131, 122, 206),
        13: ('rect',   183, 138, 265, 196),
        11: ('circle',  48, 249, 122, 324),
        12: ('rect',   191, 268, 263, 308),
        7:  ('rect',    22, 334,  92, 382),
        8:  ('rect',   112, 334, 182, 382),
        9:  ('rect',   202, 334, 272, 382),
        10: ('rect',   292, 334, 362, 382),
    }

    # MIDI-ish note per pad key (ascending with key id, like piano keys
    # laid out bottom-to-top: 7..10, 11, 12, 13, 14, 15 -> C4..D5).
    KEY_PAD_NOTES = {
        7: 261.63, 8: 293.66, 9: 329.63, 10: 349.23,
        11: 392.00, 12: 440.00, 13: 493.88, 14: 523.25, 15: 587.33,
    }

    def _ensure_note_wavs(self):
        if self.note_wavs or winsound is None:
            return
        # winsound can't play SND_ASYNC + SND_MEMORY together (CPython
        # raises RuntimeError), so each note is synthesized once to a
        # small temp .wav file and looped from disk instead.
        self.note_tmp_dir = tempfile.mkdtemp(prefix='mtch_notes_')
        for kid, freq in self.KEY_PAD_NOTES.items():
            path = os.path.join(self.note_tmp_dir, f'note_{kid}.wav')
            with open(path, 'wb') as f:
                f.write(_make_tone_wav(freq))
            self.note_wavs[kid] = path

    def _sound_current_top(self):
        """Play the most-recently-pressed held key's note (mono, last-note
        priority, like a simple synth's note-on/note-off voice); silence
        when nothing is held. The tone loops continuously until replaced
        or stopped, so its length tracks how long the key stays pressed."""
        if winsound is None:
            return
        if self.note_stack:
            path = self.note_wavs.get(self.note_stack[-1])
            if path:
                winsound.PlaySound(path, winsound.SND_FILENAME | winsound.SND_ASYNC
                                   | winsound.SND_LOOP)
        else:
            winsound.PlaySound(None, winsound.SND_PURGE)

    def _note_on(self, key_id):
        if key_id not in self.KEY_PAD_NOTES:
            return
        if key_id in self.note_stack:
            self.note_stack.remove(key_id)
        self.note_stack.append(key_id)
        self._sound_current_top()

    def _note_off(self, key_id):
        if key_id in self.note_stack:
            self.note_stack.remove(key_id)
        self._sound_current_top()

    def _open_keypattern_window(self):
        if self.keypattern_window and self.keypattern_window.winfo_exists():
            self.keypattern_window.lift()
            self.keypattern_window.focus_force()
            return
        self._ensure_note_wavs()

        def _on_close():
            self.note_stack = []
            if winsound is not None:
                winsound.PlaySound(None, winsound.SND_PURGE)
            win.destroy()

        win = tk.Toplevel(self.root)
        win.title("Touch Key Pad")
        win.geometry("420x460")
        win.configure(bg=DARK_BG)
        win.protocol("WM_DELETE_WINDOW", _on_close)

        canvas = tk.Canvas(win, width=400, height=430, bg=DARK_BG,
                           highlightthickness=0)
        canvas.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)
        self.keypattern_canvas = canvas
        self.keypattern_window = win
        self._draw_keypattern_layout()
        self.keypattern_prev = set()
        self._update_keypattern_window(self.pressed_keys)

    def _draw_keypattern_layout(self):
        c = self.keypattern_canvas
        c.delete('all')
        self.keypattern_shapes = {}
        for kid, (shape, x0, y0, x1, y1) in self.KEY_PAD_LAYOUT.items():
            if shape == 'rect':
                item = c.create_rectangle(x0, y0, x1, y1,
                                          outline=GREEN, width=4, fill=PANEL_BG)
            else:
                item = c.create_oval(x0, y0, x1, y1,
                                     outline=GREEN, width=4, fill=PANEL_BG)
            label = 'Key7' if kid == 7 else str(kid)
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            c.create_text(cx, cy, text=label, fill=TEXT_FG,
                         font=("Consolas", 10))
            self.keypattern_shapes[kid] = item

    def _update_keypattern_window(self, keys):
        """Highlight pressed pad keys; sustain each key's note for exactly
        as long as it stays pressed, and silence it the instant it's
        released (note-on / note-off, like a MIDI keyboard)."""
        if not (self.keypattern_window and self.keypattern_window.winfo_exists()):
            return
        active = keys & self.keypattern_shapes.keys()
        newly_pressed = active - self.keypattern_prev
        newly_released = self.keypattern_prev - active
        for kid in newly_pressed:
            self._note_on(kid)
        for kid in newly_released:
            self._note_off(kid)
        self.keypattern_prev = active

        c = self.keypattern_canvas
        for kid, item in self.keypattern_shapes.items():
            c.itemconfig(item, fill=KEY_ON_BG if kid in active else PANEL_BG)

    # ---- about ----

    def _open_about_window(self):
        if self.about_window and self.about_window.winfo_exists():
            self.about_window.lift()
            self.about_window.focus_force()
            return
        win = tk.Toplevel(self.root)
        win.title("About")
        win.geometry("420x420")
        win.configure(bg=DARK_BG)
        win.resizable(False, False)
        win.protocol("WM_DELETE_WINDOW", win.destroy)

        tk.Label(win, text="MTCH3380P / MTCH3240P Touch Monitor",
                font=("Consolas", 13, "bold"), fg=TEXT_FG, bg=BAR_BG,
                pady=10).pack(fill=tk.X)
        tk.Label(win, text=f"Version {APP_VERSION}",
                font=("Consolas", 11, "bold"), fg=GREEN, bg=DARK_BG,
                pady=6).pack(fill=tk.X)
        tk.Label(win, text=f"Authors: {APP_AUTHORS}",
                font=("Consolas", 10), fg=TEXT_FG, bg=DARK_BG,
                pady=2).pack(fill=tk.X)

        pyserial_ver = getattr(serial, '__version__', 'unknown') if serial else None
        dep_lines = [
            "Dependencies:",
            f"  Python        {platform.python_version()} required: 3.8+",
            f"  pyserial      {'installed, v' + pyserial_ver if serial else 'MISSING (pip install pyserial)'}",
            "  tkinter       standard library (bundled with Python)",
            f"  winsound      {'available' if winsound else 'unavailable (non-Windows)'} "
            "-- Touch Key Pad audio",
        ]
        sys_lines = [
            "",
            "System info:",
            f"  OS            {platform.platform()}",
            f"  Machine       {platform.machine()}",
            f"  Python exe    {sys.executable}",
            f"  Interpreter   {platform.python_implementation()} "
            f"{platform.python_version()}",
        ]

        body = tk.Text(win, font=("Consolas", 9), fg='#bac2de', bg=PANEL_BG,
                       bd=0, highlightthickness=0, wrap='none', state='normal')
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=(4, 10))
        body.insert('1.0', '\n'.join(dep_lines + sys_lines))
        body.configure(state='disabled')

        self.about_window = win

    def _render_sensor_states(self):
        """Render the union of per-sensor T501 and T502 touch states."""
        active = self.pressed_keys | self.t502_active_sensor_ids
        for index in self.conn.enabled_sensor_ids:
            self._set_sensor_node_state(self._node_label(index), index in active)

    def _show_touch(self, touched, x=0, y=0):
        # Sensor nodes are shown in the grid; do not add a second summary area.
        return

    def _set_info(self, txt):
        self.info_txt.configure(state='normal')
        self.info_txt.delete('1.0', tk.END)
        self.info_txt.insert('1.0', txt)
        self.info_txt.configure(state='disabled')

    def _log(self, msg):
        self.log_lbl.configure(text=msg)

    def _set_config_buttons_visible(self, visible):
        for button in self.config_buttons:
            button.destroy()
        self.config_buttons = []
        if not visible:
            return
        for label, config_name in self._config_choices():
            button = tk.Button(
                self.config_frame, text=f"Burn {label}",
                font=("Consolas", 10, "bold"), fg='#1e1e2e', bg=MAUVE,
                activebackground='#f5c2e7', activeforeground='#1e1e2e',
                relief='flat', padx=10, pady=6,
                command=lambda path=config_name: self._on_config_button(path))
            button.pack(side=tk.LEFT, padx=(6, 0))
            self.config_buttons.append(button)

    def _set_config_mode(self, mode):
        self.current_config_mode = mode
        self.config_status_lbl.configure(text=f"Config: {mode or 'UNKNOWN'}",
                                         fg=GREEN if mode else DIM_FG)

    def _detect_config_mode(self):
        """Return the mode decoded from the live T500 configuration."""
        return self.conn.config_mode or self.current_config_mode

    def _config_choices(self):
        prefix = 'MTCH3380P' if self.conn.is_3380 else 'MTCH3240P'
        if self.conn.is_3380:
            self_config = 'EVK_MTCH3380P_1.0.AA_P2PSC.xcfg'
        else:
            self_config = 'EVK_MTCH3240P_1.0.AA_P2PSC.xcfg'
        return (
            ('MUTUAL', f'EVK_{prefix}_1.0.AA_Mutual.xcfg'),
            ('SELF', self_config),
        )

    def _on_config_button(self, config_name):
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   config_name)
        if not os.path.exists(config_path):
            self._log(f"Config not found: {config_name}")
            return
        self._set_config_buttons_visible(False)
        self.btn.configure(state='disabled')
        self._log(f"Burning {config_name}...")
        self._stop_poll()
        threading.Thread(target=self._do_write_config,
                         args=(config_path, config_name), daemon=True).start()

    def _do_write_config(self, config_path, config_name):
        try:
            self.conn.write_config(config_path)
            self.root.after(0, self._config_write_ok, config_name)
        except Exception as error:
            self.root.after(0, self._config_write_fail, str(error))

    def _config_write_ok(self, config_name):
        self.btn.configure(state='disabled')
        self._log(f"Burned {config_name}; refreshing device...")
        threading.Thread(target=self._refresh_after_config,
                         args=(config_name,), daemon=True).start()

    def _config_write_fail(self, error):
        self.btn.configure(state='normal')
        self._set_config_buttons_visible(True)
        self._log(f"Burn failed: {error}")
        self._start_poll()

    def _refresh_after_config(self, config_name):
        """Reconnect so the UI reflects the newly written configuration."""
        try:
            self.conn.close()
            time.sleep(0.5)
            name = self.conn.connect()
            self.root.after(0, self._config_refresh_ok, name, config_name)
        except Exception as error:
            self.root.after(0, self._config_refresh_fail, str(error))

    def _config_refresh_ok(self, name, config_name):
        mode = 'MUTUAL' if 'Mutual' in config_name else 'SELF'
        self._set_config_mode(mode)
        self._set_status(f"{name} Connected", GREEN)
        self._set_info("")
        self.key_amplitudes.clear()
        self.t502_active_sensor_ids.clear()
        self._show_keys(set())
        self._show_touch(False)
        self._build_key_grid(self._independent_key_ids())
        self._build_sensor_grid()
        self._build_slider_widgets()
        self.btn.configure(text="Disconnect", bg=RED, state='normal')
        self._set_config_buttons_visible(True)
        self._log(f"{config_name} applied; interface refreshed")
        self._start_poll()

    def _config_refresh_fail(self, error):
        self._set_status("Refresh Failed", RED)
        self.btn.configure(text="Connect ATMXT3380P/3240P", bg=BLUE,
                           state='normal')
        self._set_config_buttons_visible(False)
        self._log(f"Config applied, but refresh failed: {error}")

    # ---- connect ----

    def _on_btn(self):
        if self.conn.connected:
            self._stop_poll()
            self.conn.close()
            self._set_status("Not Connected", RED)
            self.key_amplitudes.clear()
            self.t502_active_sensor_ids.clear()
            self._show_keys(set())
            self._show_touch(False)
            self._set_info("")
            for w in self.slider_inner.winfo_children():
                w.destroy()
            self.slider_widgets = {}
            for widget in self.sensor_grid.winfo_children():
                widget.destroy()
            self.sensor_labels = {}
            self._set_config_buttons_visible(False)
            self._set_config_mode(None)
            self.btn.configure(text="Connect ATMXT3380P/3240P", bg=BLUE)
            self._log("Disconnected")
            return
        self.btn.configure(state='disabled')
        self._set_status("Searching...", YELLOW)
        self._log("Scanning USB ports for HSBB bridge...")
        threading.Thread(target=self._do_connect, daemon=True).start()

    def _do_connect(self):
        try:
            name = self.conn.connect()
            self.root.after(0, self._connected_ok, name)
        except Exception as e:
            self.root.after(0, self._connected_fail, str(e))

    def _connected_ok(self, name):
        self._set_config_mode(self._detect_config_mode())
        self._set_status(f"{name} Connected", GREEN)
        self.btn.configure(text="Disconnect", bg=RED, state='normal')

        info = self.conn.id_info
        v = info.get('Version', 0)
        lines = [
            f"Device:  {name}",
            f"Family:  0x{info.get('Family ID',0):02X}",
            f"Variant: 0x{info.get('Variant ID',0):02X}",
            f"Version: {v>>4}.{v&0xF}",
            f"Build:   0x{info.get('Build',0):02X}",
            f"Matrix:  {info.get('Matrix X Size',0)}"
            f" x {info.get('Matrix Y Size',0)}",
            "",
        ]
        if self.conn.t500_report_ids:
            lo, hi = self.conn.t500_report_ids
            lines.append(f"T500 RID: {lo}-{hi}")
        if self.conn.t501_report_ids:
            lo, hi = self.conn.t501_report_ids
            lines.append(f"T501 RID: {lo}-{hi}")
            t501_obj = next((o for o in self.conn.ext_obj_table if o[0] == 501), None)
            if t501_obj:
                lines.append(f"T501 size: {t501_obj[2]} bytes/instance")
        if self.conn.t502_report_ids:
            lo, hi = self.conn.t502_report_ids
            lines.append(f"T502 RID: {lo}-{hi}")
        if self.conn.t100_report_ids:
            lo, hi = self.conn.t100_report_ids
            lines.append(f"T100 RID: {lo}-{hi}")
        if self.conn.t72_report_ids:
            lo, hi = self.conn.t72_report_ids
            lines.append(f"T72 RID: {lo}-{hi}")
        occupied_sensor_ids = {
            index
            for slider in self.conn.sliders
            for index in range(slider.startkey,
                               slider.startkey + slider.numkeys)
            if index in self.conn.enabled_sensor_ids
        }
        lines.extend([
            "",
            f"Sensors: {len(self.conn.enabled_sensor_ids)}",
            f"Slider/Wheel sensors: {len(occupied_sensor_ids)}",
            f"Keys: {len(self._independent_key_ids())}",
        ])
        if self.conn.sliders:
            lines.append("")
            for si in self.conn.sliders:
                lines.append(f"{si.label}: keys {si.startkey}-"
                             f"{si.startkey+si.numkeys-1}"
                             f" range=0-{si.max_pos}")

        lines.append("")
        lines.append("All objects (addr size x inst, rp=report-ids/inst):")
        for ot, sa, sz, inst, rp, lo, hi in self.conn.object_rid_map:
            rid_txt = f"RID {lo}-{hi}" if lo is not None else "no RID"
            lines.append(f"  T{ot}: addr=0x{sa:04X} size={sz} "
                        f"inst={inst} rp={rp} {rid_txt}")
        self._set_info('\n'.join(lines))

        self._build_key_grid(self._independent_key_ids())
        self._build_sensor_grid()
        self._build_slider_widgets()
        self._set_config_buttons_visible(True)
        self._log(f"{name} on {self.conn.port_name}")
        self._start_poll()

    def _connected_fail(self, err):
        self._set_status("Connection Failed", RED)
        self.btn.configure(state='normal')
        self._log(err)

    # ---- polling ----

    def _start_poll(self):
        self.polling = True
        self.poll_thread = threading.Thread(target=self._poll, daemon=True)
        self.poll_thread.start()

    def _stop_poll(self):
        self.polling = False
        if self.poll_thread:
            self.poll_thread.join(timeout=2)
            self.poll_thread = None

    def _poll(self):
        while self.polling and self.conn.connected:
            try:
                for _ in range(30):
                    rid, payload = self.conn.read_msg_fast()
                    if rid == 0xFF or payload is None:
                        break
                    self._handle_msg(rid, payload)
            except Exception as e:
                if self.polling:
                    self.root.after(0, self._log, f"Error: {e}")
                    time.sleep(0.5)
            time.sleep(0.002)

    def _handle_msg(self, rid, payload):
        # ---- T100 Touchscreen ----
        if self.conn.is_t100(rid):
            if len(payload) >= 5:
                status = payload[0]
                touched = bool(status & 0x80)
                x = payload[1] | ((payload[2] & 0x0F) << 8)
                y = payload[3] | ((payload[4] & 0x0F) << 8)
                self.root.after(0, self._show_touch, touched, x, y)
                if touched:
                    self.root.after(0, self._log,
                                    f"Touch X={x} Y={y} status=0x{status:02X}")

        # ---- T500 Key Array ----
        elif self.conn.is_t500(rid):
            new_keys = self.conn.parse_t500_keys(payload)
            if new_keys != self.pressed_keys:
                self.pressed_keys = new_keys
                self.key_amplitudes = {
                    key: value for key, value in self.key_amplitudes.items()
                    if key in new_keys
                }
                ks = new_keys.copy()
                self.root.after(0, self._show_keys, ks)
                if ks:
                    s = ', '.join(str(k) for k in sorted(ks))
                    self.root.after(0, self._log, f"Key {s}")
                else:
                    self.root.after(0, self._log, "Released")

        # ---- T501 Key Controller ----
        elif self.conn.is_t501(rid):
            kid = self.conn.t501_key_id(rid)
            if len(payload) >= 1:
                st = payload[0]
                # Record every raw T501 message regardless of how we interpret
                # it -- lets us see the real STATUS/field layout on real HW.
                self.t501_raw[kid] = (st, payload.hex())
                self.t501_log.append(f"k{kid} st=0x{st:02X} raw={payload.hex()}")
                self.root.after(0, self._refresh_t501_debug)

                touched = bool(st & T501_DETECT_MASK)
                if touched:
                    self.pressed_keys.add(kid)
                    if len(payload) >= 3:
                        self.key_amplitudes[kid] = int.from_bytes(
                            payload[1:3], 'little')
                else:
                    self.pressed_keys.discard(kid)
                    self.key_amplitudes.pop(kid, None)
                ks = self.pressed_keys.copy()
                self.root.after(0, self._show_keys, ks)

        # ---- T502 Slider/Wheel ----
        elif self.conn.is_t502(rid):
            inst = self.conn.t502_instance(rid)
            touched, pos, amp = self.conn.parse_t502(payload)
            si = self.conn.slider_for_instance(inst)
            if si:
                si.touched = touched
                si.position = pos
                si.amplitude = amp
                self.root.after(0, self._update_slider, inst, pos, amp, touched)
                previous_sensor_id = si.active_sensor_id
                current_sensor_id = (
                    self.conn.t502_sensor_id(si, pos) if touched else None
                )
                if previous_sensor_id is not None:
                    self.t502_active_sensor_ids.discard(previous_sensor_id)
                if current_sensor_id is not None:
                    self.t502_active_sensor_ids.add(current_sensor_id)
                si.active_sensor_id = current_sensor_id
                self.root.after(0, self._render_sensor_states)
                if touched:
                    self.root.after(0, self._log,
                                   f"{si.label} pos={pos} amp={amp}")

        # Keep unsupported sensor reports visible instead of silently dropping
        # them. This is useful for configurations that report through T72.
        else:
            report = (rid, bytes(payload))
            if report != self.conn.last_other_report:
                self.conn.last_other_report = report
                self.root.after(0, self._log,
                                f"Report {rid}: {payload.hex()}")

    # ---- cleanup ----

    def _on_close(self):
        self._stop_poll()
        self.conn.close()
        if winsound is not None:
            winsound.PlaySound(None, winsound.SND_PURGE)
        if self.note_tmp_dir:
            shutil.rmtree(self.note_tmp_dir, ignore_errors=True)
        self.root.destroy()


if __name__ == '__main__':
    if serial is None:
        _show_missing_pyserial_message()
    else:
        MaxTouchGUI().run()
