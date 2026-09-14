#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (C) EdgeTX
#
# Based on code named
#   opentx - https://github.com/opentx/opentx
#   th9x - http://code.google.com/p/th9x
#   er9x - http://code.google.com/p/er9x
#   gruvin9x - http://code.google.com/p/gruvin9x
#
# License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

"""Display an EdgeTX framebuffer, RTT log, and live state through J-Link."""

import argparse
import codecs
from collections import deque
import ctypes
from dataclasses import dataclass
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

from elftools.elf.elffile import ELFFile
from elftools.elf.sections import SymbolTableSection
from PIL import Image, ImageOps, ImageTk


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 2331
DEFAULT_RTT_PORT = 19021
DEFAULT_GDB_SERVER = "JLinkGDBServerCLExe"
DEFAULT_GDB = "arm-none-eabi-gdb"
DEFAULT_DEVICE = "STM32H753II"
DEFAULT_INTERFACE = "JTAG"
DEFAULT_SPEED = 4000
DEFAULT_SERVER_TIMEOUT = 15
DEFAULT_PROGRAM_TIMEOUT = 120
DEFAULT_SYMBOL = "displayBuf"
DEFAULT_RTT_SYMBOL = "_SEGGER_RTT"
DEFAULT_MONITOR_SYMBOL = "debugMonitorSnapshot"
DEFAULT_WIDTH = 128
DEFAULT_HEIGHT = 64
DEFAULT_FPS = 5.0
DEFAULT_SCALE = 6
SOCKET_TIMEOUT = 0.5
RECONNECT_DELAY = 1.0
RTT_CONTROL_PERIOD = 0.05
KEY_RELEASE_DELAY_MS = 30
MAX_READ_CHUNK = 1024
STABLE_READ_COMPARISONS = 3
DEFAULT_LOG_HEIGHT = 240
MAX_LOG_LINES = 5000
MAX_PENDING_RTT_CHUNKS = 256
DEFAULT_MONITOR_WIDTH = 520
CONTROL_BAR_HEIGHT = 38
MONITOR_MAGIC = 0x4D585445
MONITOR_VERSION = 1
MONITOR_LABEL_LENGTH = 8
MONITOR_MAX_KEYS = 16
MONITOR_MAX_SWITCHES = 20
MONITOR_MAX_ANALOGS = 22
MONITOR_MAX_INPUTS = 32
MONITOR_MAX_CHANNELS = 32
MONITOR_MAX_MODULES = 2

KEY_DEFAULT_NAMES = (
    "MENU",
    "EXIT",
    "ENTER",
    "PAGE UP",
    "PAGE DOWN",
    "UP",
    "DOWN",
    "LEFT",
    "RIGHT",
    "PLUS",
    "MINUS",
    "MODEL",
    "TELEMETRY",
    "SYSTEM",
    "SHIFT",
    "BIND",
)
KEY_INDEX = {name: index for index, name in enumerate(KEY_DEFAULT_NAMES)}
KEYBOARD_KEYS = {
    "Up": KEY_INDEX["UP"],
    "Down": KEY_INDEX["DOWN"],
    "Left": KEY_INDEX["LEFT"],
    "Right": KEY_INDEX["RIGHT"],
    "Prior": KEY_INDEX["PAGE UP"],
    "Next": KEY_INDEX["PAGE DOWN"],
    "Escape": KEY_INDEX["EXIT"],
    "Return": KEY_INDEX["ENTER"],
    "KP_Enter": KEY_INDEX["ENTER"],
}
SWITCH_TYPE_NAMES = ("2-position", "3-position", "ADC")
SWITCH_POSITION_NAMES = ("UP", "MID", "DOWN")
MODULE_NAMES = ("Internal", "External")
PROTOCOL_NAMES = (
    "Uninitialized",
    "None",
    "PPM",
    "PXX1",
    "DSM2",
    "CRSF",
    "Multi",
    "SBUS",
    "PXX2",
    "AFHDS2A",
    "AFHDS3",
    "Ghost",
    "DSMP",
)
MODULE_PORT_NAMES = ("UART", "Timer", "Soft serial", "S.Port", "S.Port inverted")
MODULE_TYPE_NAMES = {0: "None", 1: "Timer", 2: "Serial"}


class DebugMonitorModulePort(ctypes.LittleEndianStructure):
    _fields_ = (
        ("baudrate", ctypes.c_uint32),
        ("active", ctypes.c_uint8),
        ("type", ctypes.c_uint8),
        ("port", ctypes.c_uint8),
        ("direction", ctypes.c_uint8),
    )


class DebugMonitorModule(ctypes.LittleEndianStructure):
    _fields_ = (
        ("tx", DebugMonitorModulePort),
        ("rx", DebugMonitorModulePort),
        ("protocol", ctypes.c_uint8),
        ("powered", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 2),
    )


MonitorLabel = ctypes.c_char * MONITOR_LABEL_LENGTH


class DebugMonitorSnapshot(ctypes.LittleEndianStructure):
    _fields_ = (
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint16),
        ("size", ctypes.c_uint16),
        ("sequence", ctypes.c_uint32),
        ("tick10ms", ctypes.c_uint32),
        ("keysSupported", ctypes.c_uint32),
        ("keysPressed", ctypes.c_uint32),
        ("trimsPressed", ctypes.c_uint32),
        ("activeInputs", ctypes.c_uint32),
        ("activeChannels", ctypes.c_uint32),
        ("functionSwitches", ctypes.c_uint32),
        ("keyCount", ctypes.c_uint8),
        ("trimCount", ctypes.c_uint8),
        ("switchCount", ctypes.c_uint8),
        ("analogCount", ctypes.c_uint8),
        ("mainAnalogCount", ctypes.c_uint8),
        ("flexAnalogCount", ctypes.c_uint8),
        ("batteryAnalogCount", ctypes.c_uint8),
        ("rtcBatteryAnalogCount", ctypes.c_uint8),
        ("moduleCount", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
        ("keyNames", MonitorLabel * MONITOR_MAX_KEYS),
        ("switchNames", MonitorLabel * MONITOR_MAX_SWITCHES),
        ("switchTypes", ctypes.c_uint8 * MONITOR_MAX_SWITCHES),
        ("switchPositions", ctypes.c_uint8 * MONITOR_MAX_SWITCHES),
        ("functionSwitchPhysical", ctypes.c_uint8 * MONITOR_MAX_SWITCHES),
        ("functionSwitchLogical", ctypes.c_uint8 * MONITOR_MAX_SWITCHES),
        ("analogNames", MonitorLabel * MONITOR_MAX_ANALOGS),
        ("analogRaw", ctypes.c_uint16 * MONITOR_MAX_ANALOGS),
        ("analogFiltered", ctypes.c_uint16 * MONITOR_MAX_ANALOGS),
        ("inputNames", MonitorLabel * MONITOR_MAX_INPUTS),
        ("inputs", ctypes.c_int16 * MONITOR_MAX_INPUTS),
        ("channelNames", MonitorLabel * MONITOR_MAX_CHANNELS),
        ("mixers", ctypes.c_int16 * MONITOR_MAX_CHANNELS),
        ("outputs", ctypes.c_int16 * MONITOR_MAX_CHANNELS),
        ("modules", DebugMonitorModule * MONITOR_MAX_MODULES),
    )


MONITOR_SIZE = ctypes.sizeof(DebugMonitorSnapshot)


class ViewerError(Exception):
    """Report an error that the user can correct."""


class RemoteProtocolError(ViewerError):
    """Report an invalid or failed GDB remote operation."""


class ManagedGdbServer:
    """Manage one J-Link GDB server process."""

    def __init__(self, command, host, port, timeout):
        self.command = command
        self.host = host
        self.port = port
        self.timeout = timeout
        self.process = None
        self.output = deque(maxlen=80)
        self.output_lock = threading.Lock()
        self.output_thread = None

    def _capture_output(self):
        if self.process is None or self.process.stdout is None:
            return
        for line in self.process.stdout:
            with self.output_lock:
                self.output.append(line.rstrip())

    def _output_tail(self):
        with self.output_lock:
            return "\n".join(self.output)

    @staticmethod
    def _port_is_open(host, port):
        try:
            with socket.create_connection((host, port), timeout=0.1):
                return True
        except OSError:
            return False

    def start(self):
        if self._port_is_open(self.host, self.port):
            raise ViewerError(
                f"GDB port {self.host}:{self.port} is in use. Use --attach "
                "for an existing server."
            )

        try:
            self.process = subprocess.Popen(
                self.command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except OSError as error:
            raise ViewerError(
                f"cannot start J-Link GDB server '{self.command[0]}': {error}"
            ) from error

        self.output_thread = threading.Thread(
            target=self._capture_output,
            daemon=True,
        )
        self.output_thread.start()

        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                if self.output_thread is not None:
                    self.output_thread.join(timeout=0.25)
                detail = self._output_tail()
                message = f"J-Link GDB server exited with {self.process.returncode}"
                if detail:
                    message += f":\n{detail}"
                raise ViewerError(message)
            if self._port_is_open(self.host, self.port):
                return
            time.sleep(0.05)

        raise ViewerError(
            f"J-Link GDB server did not open {self.host}:{self.port} "
            f"within {self.timeout} seconds"
        )

    def stop(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1)
        if self.output_thread is not None:
            self.output_thread.join(timeout=0.25)


@dataclass(frozen=True)
class ElfSymbol:
    address: int
    size: int


def positive_integer(value):
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("the value must be greater than zero")
    return result


def valid_port(value):
    result = int(value)
    if not 1 <= result <= 65535:
        raise argparse.ArgumentTypeError("the port must be between 1 and 65535")
    return result


def valid_frame_rate(value):
    result = float(value)
    if not 0.0 < result <= 60.0:
        raise argparse.ArgumentTypeError("the frame rate must be above 0 and at most 60")
    return result


def resolve_symbol_info(elf_path, symbol_name, expected_size=None):
    try:
        elf_file = elf_path.open("rb")
    except OSError as error:
        raise ViewerError(f"cannot open ELF file '{elf_path}': {error}") from error

    with elf_file:
        elf = ELFFile(elf_file)
        matches = []
        for section in elf.iter_sections():
            if not isinstance(section, SymbolTableSection):
                continue
            for symbol in section.get_symbol_by_name(symbol_name) or []:
                if symbol["st_shndx"] != "SHN_UNDEF":
                    matches.append(symbol)

        if not matches:
            raise ViewerError(
                f"ELF file '{elf_path}' has no defined '{symbol_name}' symbol"
            )

        sized_matches = matches
        if expected_size is not None:
            sized_matches = [
                symbol for symbol in matches if int(symbol["st_size"]) == expected_size
            ]
            if not sized_matches:
                sizes = sorted({int(symbol["st_size"]) for symbol in matches})
                raise ViewerError(
                    f"symbol '{symbol_name}' has size {sizes}, "
                    f"expected {expected_size} bytes"
                )

        values = {
            (int(symbol["st_value"]), int(symbol["st_size"]))
            for symbol in sized_matches
        }
        if len(values) != 1:
            addresses = ", ".join(
                f"0x{address:08x}/{size}"
                for address, size in sorted(values)
            )
            raise ViewerError(
                f"symbol '{symbol_name}' has multiple addresses: {addresses}"
            )

        address, size = values.pop()
        return ElfSymbol(address, size)


def resolve_symbol(elf_path, symbol_name, expected_size=None):
    return resolve_symbol_info(elf_path, symbol_name, expected_size).address


def make_gdb_server_command(arguments):
    return [
        arguments.gdb_server,
        "-device",
        arguments.device,
        "-if",
        arguments.target_interface,
        "-speed",
        str(arguments.speed),
        "-port",
        str(arguments.port),
        "-rtttelnetport",
        str(arguments.rtt_port),
    ]


def make_program_command(arguments):
    return [
        arguments.gdb,
        "--batch",
        str(arguments.elf),
        "-ex",
        "set pagination off",
        "-ex",
        f"target remote {arguments.host}:{arguments.port}",
        "-ex",
        "monitor halt",
        "-ex",
        "load",
        "-ex",
        "monitor reset",
        "-ex",
        "monitor go",
        "-ex",
        "detach",
    ]


def program_target(arguments):
    command = make_program_command(arguments)
    try:
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=arguments.program_timeout,
            check=False,
        )
    except OSError as error:
        raise ViewerError(f"cannot start GDB '{arguments.gdb}': {error}") from error
    except subprocess.TimeoutExpired as error:
        raise ViewerError(
            f"GDB did not program the target within "
            f"{arguments.program_timeout} seconds"
        ) from error

    if result.returncode:
        output = result.stdout.strip()
        message = f"GDB exited with {result.returncode}"
        if output:
            message += f":\n{output}"
        raise ViewerError(message)


class GdbRemoteClient:
    """Provide the GDB remote operations that the LCD viewer needs."""

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = None
        self.receive_buffer = bytearray()
        self.packet_size = 4096

    def connect(self):
        self.socket = socket.create_connection(
            (self.host, self.port), timeout=SOCKET_TIMEOUT
        )
        self.socket.settimeout(SOCKET_TIMEOUT)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        supported = self.request(b"qSupported")
        match = re.search(rb"(?:^|;)PacketSize=([0-9a-fA-F]+)(?:;|$)", supported)
        if match:
            self.packet_size = int(match.group(1), 16)

    def close(self):
        if self.socket is not None:
            try:
                self.socket.close()
            finally:
                self.socket = None
                self.receive_buffer.clear()

    def _receive_byte(self):
        if not self.receive_buffer:
            if self.socket is None:
                raise RemoteProtocolError("the GDB server connection is closed")
            data = self.socket.recv(4096)
            if not data:
                raise RemoteProtocolError("the GDB server closed the connection")
            self.receive_buffer.extend(data)
        value = self.receive_buffer[0]
        del self.receive_buffer[0]
        return value

    def _send(self, data):
        if self.socket is None:
            raise RemoteProtocolError("the GDB server connection is closed")
        self.socket.sendall(data)

    @staticmethod
    def _decode_payload(payload):
        decoded = bytearray()
        index = 0
        while index < len(payload):
            value = payload[index]
            if value == ord("}"):
                index += 1
                if index == len(payload):
                    raise RemoteProtocolError("the GDB packet ends with an escape byte")
                decoded.append(payload[index] ^ 0x20)
            elif value == ord("*"):
                index += 1
                if not decoded or index == len(payload):
                    raise RemoteProtocolError("the GDB packet contains invalid run length data")
                repeat_count = payload[index] - 29
                if repeat_count < 0:
                    raise RemoteProtocolError("the GDB packet contains invalid run length data")
                decoded.extend([decoded[-1]] * repeat_count)
            else:
                decoded.append(value)
            index += 1
        return bytes(decoded)

    def _receive_packet(self, start_seen=False):
        while True:
            if not start_seen:
                while self._receive_byte() != ord("$"):
                    pass
            start_seen = False

            encoded = bytearray()
            while True:
                value = self._receive_byte()
                if value == ord("#"):
                    break
                encoded.append(value)

            checksum_text = bytes((self._receive_byte(), self._receive_byte()))
            try:
                received_checksum = int(checksum_text, 16)
            except ValueError as error:
                self._send(b"-")
                raise RemoteProtocolError("the GDB packet has an invalid checksum") from error

            expected_checksum = sum(encoded) & 0xFF
            if received_checksum != expected_checksum:
                self._send(b"-")
                continue

            self._send(b"+")
            return self._decode_payload(encoded)

    def request(self, payload):
        checksum = sum(payload) & 0xFF
        packet = b"$" + payload + b"#" + f"{checksum:02x}".encode("ascii")

        for _ in range(3):
            self._send(packet)
            while True:
                response = self._receive_byte()
                if response == ord("+"):
                    return self._receive_packet()
                if response == ord("-"):
                    break
                if response == ord("$"):
                    return self._receive_packet(start_seen=True)
        raise RemoteProtocolError("the GDB server rejected the request")

    def read_memory(self, address, size):
        result = bytearray()
        maximum_reply = max(2, self.packet_size - 32)
        chunk_size = min(MAX_READ_CHUNK, maximum_reply // 2)

        while len(result) < size:
            read_size = min(chunk_size, size - len(result))
            command = f"m{address + len(result):x},{read_size:x}".encode("ascii")
            response = self.request(command)
            if response.startswith(b"E"):
                message = response.decode("ascii", errors="replace")
                raise RemoteProtocolError(f"GDB memory read failed with {message}")
            try:
                data = bytes.fromhex(response.decode("ascii"))
            except (UnicodeDecodeError, ValueError) as error:
                raise RemoteProtocolError("GDB returned invalid memory data") from error
            if len(data) != read_size:
                raise RemoteProtocolError(
                    f"GDB returned {len(data)} bytes, expected {read_size}"
                )
            result.extend(data)

        return bytes(result)

    def monitor_command(self, command):
        encoded_command = command.encode("ascii").hex().encode("ascii")
        response = self.request(b"qRcmd," + encoded_command)

        while response.startswith(b"O") and response != b"OK":
            try:
                bytes.fromhex(response[1:].decode("ascii"))
            except (UnicodeDecodeError, ValueError) as error:
                raise RemoteProtocolError("GDB returned invalid monitor output") from error
            response = self._receive_packet()

        if response == b"OK":
            return
        if response.startswith(b"E"):
            message = response.decode("ascii", errors="replace")
            raise RemoteProtocolError(f"GDB monitor command failed with {message}")
        try:
            bytes.fromhex(response.decode("ascii"))
        except (UnicodeDecodeError, ValueError) as error:
            message = response.decode("ascii", errors="replace")
            raise RemoteProtocolError(
                f"GDB monitor command returned invalid output: {message}"
            ) from error


@dataclass(frozen=True)
class ViewerSnapshot:
    frame: bytes | None
    generation: int
    status: str
    frame_rate: float
    unstable_reads: int
    rtt_status: str
    monitor: bytes | None
    monitor_generation: int
    monitor_status: str
    host_keys: int


class ViewerState:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.generation = 0
        self.status = "not connected"
        self.frame_rate = 0.0
        self.unstable_reads = 0
        self.rtt_status = "wait for GDB"
        self.monitor = None
        self.monitor_generation = 0
        self.monitor_status = "not available"
        self.host_keys = 0
        self.rtt_chunks = deque(maxlen=MAX_PENDING_RTT_CHUNKS)

    def set_status(self, status):
        with self.lock:
            self.status = status

    def set_unstable(self):
        with self.lock:
            self.status = "connected, unstable read"
            self.unstable_reads += 1

    def set_frame(self, frame, frame_rate):
        with self.lock:
            self.frame = frame
            self.generation += 1
            self.status = "connected"
            self.frame_rate = frame_rate

    def set_rtt_status(self, status):
        with self.lock:
            self.rtt_status = status

    def set_monitor_status(self, status):
        with self.lock:
            self.monitor_status = status

    def set_monitor(self, monitor):
        with self.lock:
            self.monitor = monitor
            self.monitor_generation += 1
            self.monitor_status = "connected"

    def append_rtt(self, text):
        with self.lock:
            self.rtt_chunks.append(text)

    def take_rtt(self):
        with self.lock:
            chunks = list(self.rtt_chunks)
            self.rtt_chunks.clear()
            return chunks

    def set_host_keys(self, keys):
        with self.lock:
            self.host_keys = keys

    def get_host_keys(self):
        with self.lock:
            return self.host_keys

    def snapshot(self):
        with self.lock:
            return ViewerSnapshot(
                self.frame,
                self.generation,
                self.status,
                self.frame_rate,
                self.unstable_reads,
                self.rtt_status,
                self.monitor,
                self.monitor_generation,
                self.monitor_status,
                self.host_keys,
            )


def read_stable_frame(client, address, size):
    previous = client.read_memory(address, size)
    for _ in range(STABLE_READ_COMPARISONS):
        current = client.read_memory(address, size)
        if current == previous:
            return current
        previous = current
    return None


def decode_monitor(data):
    if len(data) != MONITOR_SIZE:
        raise ViewerError(
            f"monitor snapshot has {len(data)} bytes, expected {MONITOR_SIZE}"
        )

    monitor = DebugMonitorSnapshot.from_buffer_copy(data)
    if monitor.magic == 0 and monitor.version == 0:
        raise ViewerError("wait for the first firmware snapshot")
    if monitor.magic != MONITOR_MAGIC:
        raise ViewerError(f"monitor magic is 0x{monitor.magic:08x}")
    if monitor.version != MONITOR_VERSION:
        raise ViewerError(
            f"monitor ABI is {monitor.version}, expected {MONITOR_VERSION}"
        )
    if monitor.size != MONITOR_SIZE:
        raise ViewerError(
            f"monitor ABI size is {monitor.size}, expected {MONITOR_SIZE}"
        )

    limits = (
        (monitor.switchCount, MONITOR_MAX_SWITCHES, "switch"),
        (monitor.analogCount, MONITOR_MAX_ANALOGS, "analog"),
        (monitor.moduleCount, MONITOR_MAX_MODULES, "module"),
    )
    for value, maximum, name in limits:
        if value > maximum:
            raise ViewerError(
                f"monitor {name} count is {value}, maximum is {maximum}"
            )
    return monitor


def read_stable_monitor(client, address):
    sequence_address = address + DebugMonitorSnapshot.sequence.offset
    for _ in range(STABLE_READ_COMPARISONS + 1):
        before = int.from_bytes(client.read_memory(sequence_address, 4), "little")
        if before & 1:
            continue
        data = client.read_memory(address, MONITOR_SIZE)
        after = int.from_bytes(client.read_memory(sequence_address, 4), "little")
        stored = int.from_bytes(
            data[
                DebugMonitorSnapshot.sequence.offset:
                DebugMonitorSnapshot.sequence.offset + 4
            ],
            "little",
        )
        if before == after == stored and not after & 1:
            decode_monitor(data)
            return data
    return None


def framebuffer_image(frame, width, height, invert):
    pixels = bytearray(width * height)
    for y in range(height):
        page_offset = (y // 8) * width
        bit = 1 << (y % 8)
        for x in range(width):
            pixel_set = bool(frame[page_offset + x] & bit)
            if invert:
                pixel_set = not pixel_set
            pixels[y * width + x] = 0 if pixel_set else 255

    monochrome = Image.frombytes("L", (width, height), bytes(pixels))
    return ImageOps.colorize(monochrome, black="#182018", white="#dce8c4")


def poll_framebuffer(
    stop_event,
    rtt_ready_event,
    state,
    host,
    port,
    address,
    size,
    frame_rate,
    rtt_address,
    monitor_address,
):
    period = 1.0 / frame_rate
    client = None

    while not stop_event.is_set():
        if client is None:
            state.set_status(f"connect to {host}:{port}")
            candidate = GdbRemoteClient(host, port)
            try:
                candidate.connect()
                candidate.monitor_command(
                    f"exec SetRTTAddr 0x{rtt_address:08x}"
                )
                candidate.monitor_command("go")
                client = candidate
                rtt_ready_event.set()
                if monitor_address is not None:
                    state.set_monitor_status("connected, wait for snapshot")
            except (OSError, ViewerError) as error:
                rtt_ready_event.clear()
                candidate.close()
                state.set_status(f"reconnect: {error}")
                stop_event.wait(RECONNECT_DELAY)
                continue

        accepted_times = deque(maxlen=20)
        next_read = time.monotonic()
        try:
            while not stop_event.is_set():
                now = time.monotonic()
                if now < next_read:
                    stop_event.wait(next_read - now)
                    continue

                frame = read_stable_frame(client, address, size)
                accepted_at = time.monotonic()
                if frame is None:
                    state.set_unstable()
                else:
                    accepted_times.append(accepted_at)
                    actual_rate = 0.0
                    if len(accepted_times) > 1:
                        actual_rate = (len(accepted_times) - 1) / (
                            accepted_times[-1] - accepted_times[0]
                        )
                    state.set_frame(frame, actual_rate)

                if monitor_address is not None:
                    try:
                        monitor = read_stable_monitor(client, monitor_address)
                        if monitor is None:
                            state.set_monitor_status("connected, unstable read")
                        else:
                            state.set_monitor(monitor)
                    except ViewerError as error:
                        state.set_monitor_status(str(error))

                next_read += period
                if next_read < accepted_at:
                    next_read = accepted_at + period
        except (OSError, ViewerError) as error:
            rtt_ready_event.clear()
            state.set_status(f"reconnect: {error}")
            if monitor_address is not None:
                state.set_monitor_status(f"reconnect: {error}")
            client.close()
            client = None

    if client is not None:
        client.close()
    rtt_ready_event.clear()


def encode_key_frame(key_mask):
    return f"K{key_mask & 0xFFFFFFFF:08X}\n".encode("ascii")


def stream_rtt(
    stop_event, rtt_ready_event, state, host, port, rtt_address
):
    while not stop_event.is_set():
        if not rtt_ready_event.is_set():
            state.set_rtt_status("wait for GDB")
            rtt_ready_event.wait(0.2)
            continue

        state.set_rtt_status(f"connect to {host}:{port}")
        rtt_socket = None
        try:
            rtt_socket = socket.create_connection(
                (host, port), timeout=SOCKET_TIMEOUT
            )
            rtt_socket.settimeout(RTT_CONTROL_PERIOD)
            configuration = (
                "$$SEGGER_TELNET_ConfigStr=RTTCh;0;"
                f"SetRTTAddr;0x{rtt_address:08x};$$"
            )
            rtt_socket.sendall(configuration.encode("ascii"))
            state.set_rtt_status("connected")
            decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
            next_control = time.monotonic()

            while not stop_event.is_set():
                now = time.monotonic()
                if now >= next_control:
                    key_mask = state.get_host_keys()
                    rtt_socket.sendall(encode_key_frame(key_mask))
                    next_control = now + RTT_CONTROL_PERIOD
                try:
                    data = rtt_socket.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    raise ConnectionError("the RTT server closed the connection")
                text = decoder.decode(data).replace("\r", "")
                if text:
                    state.append_rtt(text)
        except OSError as error:
            state.set_rtt_status(f"reconnect: {error}")
            stop_event.wait(RECONNECT_DELAY)
        finally:
            if rtt_socket is not None:
                try:
                    rtt_socket.sendall(b"K00000000\n")
                except OSError:
                    pass
                rtt_socket.close()


class ViewerWindow:
    def __init__(
        self,
        root,
        state,
        width,
        height,
        scale,
        invert,
        address,
        rtt_address,
        monitor_address,
    ):
        self.root = root
        self.state = state
        self.width = width
        self.height = height
        self.invert = invert
        self.address = address
        self.rtt_address = rtt_address
        self.monitor_address = monitor_address
        self.generation = -1
        self.monitor_generation = -1
        self.base_image = None
        self.photo = None
        self.keyboard_keys = set()
        self.button_keys = set()
        self.pending_key_releases = {}
        self.host_key_status = tk.StringVar(value="Host keys: none")

        left_width = width * scale
        display_height = height * scale + CONTROL_BAR_HEIGHT
        initial_height = display_height + DEFAULT_LOG_HEIGHT
        self.root.geometry(
            f"{left_width + DEFAULT_MONITOR_WIDTH}x{initial_height}"
        )
        self.root.minsize(width + 320, height + 80)

        self.main_panes = tk.PanedWindow(
            root,
            orient=tk.HORIZONTAL,
            background="#303030",
            sashrelief=tk.RAISED,
            sashwidth=6,
        )
        self.main_panes.pack(fill=tk.BOTH, expand=True)

        left_frame = tk.Frame(self.main_panes, background="#303030")
        self.main_panes.add(left_frame, minsize=width)

        self.panes = tk.PanedWindow(
            left_frame,
            orient=tk.VERTICAL,
            background="#303030",
            sashrelief=tk.RAISED,
            sashwidth=6,
        )
        self.panes.pack(fill=tk.BOTH, expand=True)

        display_frame = tk.Frame(self.panes, background="#303030")
        display_frame.rowconfigure(0, weight=1)
        display_frame.columnconfigure(0, weight=1)
        self.canvas = tk.Canvas(
            display_frame,
            width=width * scale,
            height=height * scale,
            background="#303030",
            highlightthickness=0,
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", self._resize)

        control_frame = tk.Frame(display_frame, background="#202020")
        control_frame.grid(row=1, column=0, sticky="ew")
        tk.Label(
            control_frame,
            text="Arrows | PgUp/PgDn | Esc | Enter",
            background="#202020",
            foreground="#d8d8d8",
        ).pack(side=tk.LEFT, padx=(8, 10), pady=5)
        tk.Label(
            control_frame,
            textvariable=self.host_key_status,
            background="#202020",
            foreground="#9ea8b0",
        ).pack(side=tk.RIGHT, padx=8)
        for label, key_name in (
            ("SYS", "SYSTEM"),
            ("MODEL", "MODEL"),
            ("TELE", "TELEMETRY"),
        ):
            button = ttk.Button(control_frame, text=label, takefocus=False)
            key_index = KEY_INDEX[key_name]
            button.bind(
                "<ButtonPress-1>",
                lambda _event, index=key_index: self._button_press(index),
            )
            button.bind(
                "<ButtonRelease-1>",
                lambda _event, index=key_index: self._button_release(index),
            )
            button.bind(
                "<Leave>",
                lambda _event, index=key_index: self._button_release(index),
            )
            button.pack(side=tk.LEFT, padx=3, pady=4)
        self.panes.add(display_frame, minsize=height + CONTROL_BAR_HEIGHT)

        log_frame = tk.Frame(self.panes, background="#101010")
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        self.log = tk.Text(
            log_frame,
            background="#101010",
            foreground="#e8e8e8",
            font="TkFixedFont",
            height=10,
            insertbackground="#e8e8e8",
            state=tk.DISABLED,
            wrap=tk.NONE,
        )
        vertical_scroll = tk.Scrollbar(
            log_frame, orient=tk.VERTICAL, command=self.log.yview
        )
        horizontal_scroll = tk.Scrollbar(
            log_frame, orient=tk.HORIZONTAL, command=self.log.xview
        )
        self.log.configure(
            yscrollcommand=vertical_scroll.set,
            xscrollcommand=horizontal_scroll.set,
        )
        self.log.grid(row=0, column=0, sticky="nsew")
        vertical_scroll.grid(row=0, column=1, sticky="ns")
        horizontal_scroll.grid(row=1, column=0, sticky="ew")
        self.panes.add(log_frame, minsize=80)

        monitor_frame = tk.Frame(self.main_panes, background="#202020")
        monitor_frame.rowconfigure(0, weight=1)
        monitor_frame.columnconfigure(0, weight=1)
        self.monitor_tree = ttk.Treeview(
            monitor_frame,
            columns=("state", "value", "detail"),
            show="tree headings",
            selectmode="browse",
        )
        self.monitor_tree.heading("#0", text="Input")
        self.monitor_tree.heading("state", text="State")
        self.monitor_tree.heading("value", text="Value")
        self.monitor_tree.heading("detail", text="Detail")
        self.monitor_tree.column("#0", width=165, minwidth=110, stretch=True)
        self.monitor_tree.column("state", width=90, minwidth=70, stretch=False)
        self.monitor_tree.column("value", width=85, minwidth=65, stretch=False)
        self.monitor_tree.column("detail", width=145, minwidth=90, stretch=True)
        monitor_scroll = ttk.Scrollbar(
            monitor_frame, orient=tk.VERTICAL, command=self.monitor_tree.yview
        )
        self.monitor_tree.configure(yscrollcommand=monitor_scroll.set)
        self.monitor_tree.grid(row=0, column=0, sticky="nsew")
        monitor_scroll.grid(row=0, column=1, sticky="ns")
        self.monitor_tree.tag_configure("unused", foreground="#777777")
        self.monitor_tree.tag_configure("active", foreground="#006a00")
        self.monitor_tree.tag_configure("error", foreground="#a02020")
        self.monitor_tree.tag_configure("section", font="TkDefaultFont 9 bold")
        self.main_panes.add(monitor_frame, minsize=320, width=DEFAULT_MONITOR_WIDTH)

        self.root.after_idle(
            lambda: self.panes.sash_place(0, 0, display_height)
        )
        self.root.after_idle(
            lambda: self.main_panes.sash_place(0, left_width, 0)
        )
        self._set_monitor_status("wait for firmware snapshot")
        self.root.bind_all("<KeyPress>", self._key_press, add="+")
        self.root.bind_all("<KeyRelease>", self._key_release, add="+")
        self.root.bind_all("<FocusOut>", self._focus_out, add="+")
        self.root.after(50, self._update)

    def _sync_host_keys(self):
        key_mask = 0
        for key_symbol in self.keyboard_keys:
            key_mask |= 1 << KEYBOARD_KEYS[key_symbol]
        for key_index in self.button_keys:
            key_mask |= 1 << key_index
        self.state.set_host_keys(key_mask)

    def _key_press(self, event):
        if event.keysym not in KEYBOARD_KEYS:
            return None

        pending = self.pending_key_releases.pop(event.keysym, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self.keyboard_keys.add(event.keysym)
        self._sync_host_keys()
        return "break"

    def _key_release(self, event):
        if event.keysym not in KEYBOARD_KEYS:
            return None

        pending = self.pending_key_releases.pop(event.keysym, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self.pending_key_releases[event.keysym] = self.root.after(
            KEY_RELEASE_DELAY_MS,
            lambda key_symbol=event.keysym: self._finish_key_release(key_symbol),
        )
        return "break"

    def _finish_key_release(self, key_symbol):
        self.pending_key_releases.pop(key_symbol, None)
        self.keyboard_keys.discard(key_symbol)
        self._sync_host_keys()

    def _button_press(self, key_index):
        self.button_keys.add(key_index)
        self._sync_host_keys()

    def _button_release(self, key_index):
        self.button_keys.discard(key_index)
        self._sync_host_keys()

    def _focus_out(self, _event):
        self.root.after_idle(self._clear_keys_if_unfocused)

    def _clear_keys_if_unfocused(self):
        if self.root.focus_displayof() is not None:
            return
        for pending in self.pending_key_releases.values():
            self.root.after_cancel(pending)
        self.pending_key_releases.clear()
        self.keyboard_keys.clear()
        self.button_keys.clear()
        self._sync_host_keys()

    def _resize(self, _event=None):
        if self.base_image is not None:
            self.root.after_idle(self._render)

    def _render(self):
        canvas_width = max(1, self.canvas.winfo_width())
        canvas_height = max(1, self.canvas.winfo_height())
        scale = min(canvas_width / self.width, canvas_height / self.height)
        image_width = max(1, round(self.width * scale))
        image_height = max(1, round(self.height * scale))
        resized = self.base_image.resize(
            (image_width, image_height), Image.Resampling.NEAREST
        )
        self.photo = ImageTk.PhotoImage(resized)
        self.canvas.delete("framebuffer")
        self.canvas.create_image(
            canvas_width // 2,
            canvas_height // 2,
            image=self.photo,
            anchor=tk.CENTER,
            tags="framebuffer",
        )

    @staticmethod
    def _label_text(label):
        raw = bytes(label).split(b"\0", 1)[0]
        text = raw.decode("ascii", errors="replace").strip()
        return "".join(character if character.isprintable() else "?" for character in text)

    @staticmethod
    def _percent(value):
        return f"{value * 100.0 / 1024.0:+.1f}%"

    @staticmethod
    def _enum_name(names, value, prefix):
        if value < len(names):
            return names[value]
        return f"{prefix} {value}"

    def _set_tree_item(
        self, item, parent, text, values=("", "", ""), tags=(), open_item=False
    ):
        if self.monitor_tree.exists(item):
            self.monitor_tree.item(item, text=text, values=values, tags=tags)
        else:
            self.monitor_tree.insert(
                parent,
                tk.END,
                iid=item,
                text=text,
                values=values,
                tags=tags,
                open=open_item,
            )

    def _set_monitor_status(self, status, error=False):
        self._set_tree_item(
            "monitor.status",
            "",
            "Monitor",
            (status, "", ""),
            ("error",) if error else (),
        )

    def _update_fixed_inputs(self, monitor):
        self._set_tree_item(
            "fixed", "", "Fixed inputs", tags=("section",), open_item=True
        )
        for index in range(MONITOR_MAX_KEYS):
            if not monitor.keysSupported & (1 << index):
                continue
            label = self._label_text(monitor.keyNames[index])
            if not label:
                label = KEY_DEFAULT_NAMES[index]
            pressed = bool(monitor.keysPressed & (1 << index))
            self._set_tree_item(
                f"fixed.key.{index}",
                "fixed",
                label,
                ("PRESSED" if pressed else "released", "", ""),
                ("active",) if pressed else (),
            )

        for index in range(monitor.trimCount):
            pressed = bool(monitor.trimsPressed & (1 << index))
            direction = "-" if index % 2 == 0 else "+"
            self._set_tree_item(
                f"fixed.trim.{index}",
                "fixed",
                f"Trim {index // 2 + 1} {direction}",
                ("PRESSED" if pressed else "released", "", ""),
                ("active",) if pressed else (),
            )

    def _update_hardware(self, monitor):
        self._set_tree_item(
            "hardware", "", "Hardware", tags=("section",), open_item=True
        )
        self._set_tree_item(
            "hardware.switches",
            "hardware",
            "Switches",
            tags=("section",),
            open_item=True,
        )
        for index in range(monitor.switchCount):
            name = (
                self._label_text(monitor.switchNames[index])
                or f"Switch {index + 1}"
            )
            switch_type = self._enum_name(
                SWITCH_TYPE_NAMES, monitor.switchTypes[index], "type"
            )
            position = self._enum_name(
                SWITCH_POSITION_NAMES, monitor.switchPositions[index], "position"
            )
            if monitor.functionSwitches & (1 << index):
                physical = bool(monitor.functionSwitchPhysical[index])
                logical = bool(monitor.functionSwitchLogical[index])
                self._set_tree_item(
                    f"hardware.switch.{index}",
                    "hardware.switches",
                    name,
                    (
                        "PRESSED" if physical else "released",
                        "ON" if logical else "off",
                        switch_type,
                    ),
                    ("active",) if physical or logical else (),
                )
            else:
                self._set_tree_item(
                    f"hardware.switch.{index}",
                    "hardware.switches",
                    name,
                    (position, "", switch_type),
                )

        self._set_tree_item(
            "hardware.adcs",
            "hardware",
            "ADC inputs",
            tags=("section",),
            open_item=True,
        )
        main_end = monitor.mainAnalogCount
        flex_end = main_end + monitor.flexAnalogCount
        battery_end = flex_end + monitor.batteryAnalogCount
        for index in range(monitor.analogCount):
            name = self._label_text(monitor.analogNames[index]) or f"ADC {index + 1}"
            filtered = monitor.analogFiltered[index]
            if index < main_end:
                category = "main"
            elif index < flex_end:
                category = "flex"
            elif index < battery_end:
                category = "battery"
            else:
                category = "RTC battery"
            detail = category
            if index < flex_end:
                percent = (filtered - 1024) * 100.0 / 1024.0
                detail = f"{category}, {percent:+.1f}%"
            self._set_tree_item(
                f"hardware.adc.{index}",
                "hardware.adcs",
                name,
                ("", str(filtered), f"raw {monitor.analogRaw[index]}, {detail}"),
            )

    def _update_value_layer(self, monitor, section, title, values, active_mask, names):
        self._set_tree_item(
            section, "", title, tags=("section",), open_item=True
        )
        prefix = "I" if section == "inputs" else "CH"
        for index, value in enumerate(values):
            custom_name = self._label_text(names[index])
            label = f"{prefix}{index + 1:02d}"
            if custom_name:
                label += f" {custom_name}"
            active = bool(active_mask & (1 << index))
            self._set_tree_item(
                f"{section}.{index}",
                section,
                label,
                (
                    "active" if active else "unused",
                    str(value),
                    self._percent(value),
                ),
                () if active else ("unused",),
            )

    def _format_module_port(self, port):
        if not port.active:
            return "inactive"
        port_name = self._enum_name(MODULE_PORT_NAMES, port.port, "port")
        type_name = MODULE_TYPE_NAMES.get(port.type, f"type {port.type}")
        if port.baudrate:
            return f"{port_name}, {type_name}, {port.baudrate} baud"
        return f"{port_name}, {type_name}"

    def _update_modules(self, monitor):
        self._set_tree_item(
            "modules", "", "Modules", tags=("section",), open_item=True
        )
        for index in range(monitor.moduleCount):
            module = monitor.modules[index]
            name = (
                MODULE_NAMES[index]
                if index < len(MODULE_NAMES)
                else f"Module {index}"
            )
            protocol = self._enum_name(PROTOCOL_NAMES, module.protocol, "protocol")
            active = bool(module.tx.active or module.rx.active)
            power = "powered" if module.powered else "power off"
            self._set_tree_item(
                f"modules.{index}",
                "modules",
                name,
                (power, protocol, "active" if active else "inactive"),
                () if active else ("unused",),
                open_item=True,
            )
            self._set_tree_item(
                f"modules.{index}.tx",
                f"modules.{index}",
                "TX port",
                (
                    "active" if module.tx.active else "inactive",
                    "",
                    self._format_module_port(module.tx),
                ),
                () if module.tx.active else ("unused",),
            )
            self._set_tree_item(
                f"modules.{index}.rx",
                f"modules.{index}",
                "RX port",
                (
                    "active" if module.rx.active else "inactive",
                    "",
                    self._format_module_port(module.rx),
                ),
                () if module.rx.active else ("unused",),
            )

    def _update_monitor(self, data):
        monitor = decode_monitor(data)
        self._set_monitor_status(
            f"connected at 0x{self.monitor_address:08x}"
            if self.monitor_address is not None
            else "connected"
        )
        self._update_fixed_inputs(monitor)
        self._update_hardware(monitor)
        self._update_value_layer(
            monitor,
            "inputs",
            "Input level",
            monitor.inputs,
            monitor.activeInputs,
            monitor.inputNames,
        )
        self._update_value_layer(
            monitor,
            "mixers",
            "Mixer level",
            monitor.mixers,
            monitor.activeChannels,
            monitor.channelNames,
        )
        self._update_value_layer(
            monitor,
            "outputs",
            "Channel outputs",
            monitor.outputs,
            monitor.activeChannels,
            monitor.channelNames,
        )
        self._update_modules(monitor)

    def _append_rtt(self, chunks):
        if not chunks:
            return

        follow_log = self.log.yview()[1] >= 0.999
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, "".join(chunks))

        line_count = int(self.log.index("end-1c").split(".")[0])
        if line_count > MAX_LOG_LINES:
            self.log.delete("1.0", f"{line_count - MAX_LOG_LINES + 1}.0")

        self.log.configure(state=tk.DISABLED)
        if follow_log:
            self.log.see(tk.END)

    def _update(self):
        snapshot = self.state.snapshot()
        if snapshot.host_keys:
            active_names = (
                KEY_DEFAULT_NAMES[index]
                for index in range(len(KEY_DEFAULT_NAMES))
                if snapshot.host_keys & (1 << index)
            )
            self.host_key_status.set("Host keys: " + ", ".join(active_names))
        else:
            self.host_key_status.set("Host keys: none")
        self.root.title(
            f"EdgeTX LCD 0x{self.address:08x} | LCD {snapshot.status} | "
            f"{snapshot.frame_rate:.1f} Hz | RTT 0x{self.rtt_address:08x} "
            f"{snapshot.rtt_status} | Monitor {snapshot.monitor_status}"
        )
        self._append_rtt(self.state.take_rtt())
        if snapshot.monitor_status != "connected":
            self._set_monitor_status(
                snapshot.monitor_status,
                error="error" in snapshot.monitor_status
                or "expected" in snapshot.monitor_status,
            )
        if (
            snapshot.monitor is not None
            and snapshot.monitor_generation != self.monitor_generation
        ):
            self.monitor_generation = snapshot.monitor_generation
            try:
                self._update_monitor(snapshot.monitor)
            except ViewerError as error:
                self._set_monitor_status(str(error), error=True)
        if snapshot.frame is not None and snapshot.generation != self.generation:
            self.generation = snapshot.generation
            self.base_image = framebuffer_image(
                snapshot.frame, self.width, self.height, self.invert
            )
            self._render()
        self.root.after(50, self._update)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Display an EdgeTX framebuffer, RTT log, and live inputs.",
        epilog=(
            "The default mode starts J-Link, loads the ELF, resets the target, and "
            "resumes execution. Use --attach to keep the previous workflow with an "
            "existing GDB server and no target programming or reset."
        ),
    )
    parser.add_argument("elf", type=Path, help="ELF file that runs on the target")
    parser.add_argument("--host", default=DEFAULT_HOST, help="GDB server host")
    parser.add_argument(
        "--port", type=valid_port, default=DEFAULT_PORT, help="GDB server port"
    )
    parser.add_argument(
        "--rtt-port",
        type=valid_port,
        default=DEFAULT_RTT_PORT,
        help="J-Link RTT telnet port",
    )
    startup = parser.add_argument_group("target startup")
    startup.add_argument(
        "--attach",
        action="store_true",
        help="use an existing GDB server without loading or resetting the target",
    )
    startup.add_argument(
        "--gdb-server",
        default=DEFAULT_GDB_SERVER,
        help="J-Link GDB server executable",
    )
    startup.add_argument(
        "--gdb",
        default=DEFAULT_GDB,
        help="Arm GDB executable",
    )
    startup.add_argument(
        "--device",
        default=DEFAULT_DEVICE,
        help="J-Link target device",
    )
    startup.add_argument(
        "--interface",
        dest="target_interface",
        default=DEFAULT_INTERFACE,
        help="J-Link target interface",
    )
    startup.add_argument(
        "--speed",
        type=positive_integer,
        default=DEFAULT_SPEED,
        help="J-Link interface speed in kHz",
    )
    startup.add_argument(
        "--server-timeout",
        type=positive_integer,
        default=DEFAULT_SERVER_TIMEOUT,
        help="maximum seconds for J-Link server startup",
    )
    startup.add_argument(
        "--program-timeout",
        type=positive_integer,
        default=DEFAULT_PROGRAM_TIMEOUT,
        help="maximum seconds for target programming",
    )
    parser.add_argument(
        "--symbol", default=DEFAULT_SYMBOL, help="framebuffer ELF symbol"
    )
    parser.add_argument(
        "--rtt-symbol",
        default=DEFAULT_RTT_SYMBOL,
        help="RTT control block ELF symbol",
    )
    parser.add_argument(
        "--monitor-symbol",
        default=DEFAULT_MONITOR_SYMBOL,
        help="development monitor ELF symbol",
    )
    parser.add_argument(
        "--width", type=positive_integer, default=DEFAULT_WIDTH, help="pixel width"
    )
    parser.add_argument(
        "--height", type=positive_integer, default=DEFAULT_HEIGHT, help="pixel height"
    )
    parser.add_argument(
        "--fps",
        type=valid_frame_rate,
        default=DEFAULT_FPS,
        help="target read rate, from above 0 through 60",
    )
    parser.add_argument(
        "--scale",
        type=positive_integer,
        default=DEFAULT_SCALE,
        help="initial integer window scale",
    )
    parser.add_argument(
        "--invert", action="store_true", help="reverse the framebuffer polarity"
    )
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    framebuffer_size = arguments.width * ((arguments.height + 7) // 8)

    try:
        address = resolve_symbol(
            arguments.elf, arguments.symbol, framebuffer_size
        )
        rtt_address = resolve_symbol(arguments.elf, arguments.rtt_symbol)
    except (OSError, ViewerError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    monitor_address = None
    monitor_status = "not available"
    try:
        monitor_symbol = resolve_symbol_info(
            arguments.elf, arguments.monitor_symbol
        )
        if monitor_symbol.size == MONITOR_SIZE:
            monitor_address = monitor_symbol.address
            monitor_status = "wait for GDB"
        else:
            monitor_status = (
                f"ELF symbol has {monitor_symbol.size} bytes, expected {MONITOR_SIZE}"
            )
    except ViewerError as error:
        monitor_status = str(error)

    state = ViewerState()
    state.set_monitor_status(monitor_status)
    stop_event = threading.Event()
    rtt_ready_event = threading.Event()
    framebuffer_worker = threading.Thread(
        target=poll_framebuffer,
        args=(
            stop_event,
            rtt_ready_event,
            state,
            arguments.host,
            arguments.port,
            address,
            framebuffer_size,
            arguments.fps,
            rtt_address,
            monitor_address,
        ),
        daemon=True,
    )
    rtt_worker = threading.Thread(
        target=stream_rtt,
        args=(
            stop_event,
            rtt_ready_event,
            state,
            arguments.host,
            arguments.rtt_port,
            rtt_address,
        ),
        daemon=True,
    )

    try:
        root = tk.Tk()
    except tk.TclError as error:
        print(f"error: cannot open the LCD window: {error}", file=sys.stderr)
        return 2

    managed_server = None
    return_code = 0
    try:
        if not arguments.attach:
            server_command = make_gdb_server_command(arguments)
            managed_server = ManagedGdbServer(
                server_command,
                arguments.host,
                arguments.port,
                arguments.server_timeout,
            )
            print(f"Start J-Link GDB server on {arguments.host}:{arguments.port}")
            managed_server.start()
            print(f"Load {arguments.elf} and reset the target")
            program_target(arguments)

        window = ViewerWindow(
            root,
            state,
            arguments.width,
            arguments.height,
            arguments.scale,
            arguments.invert,
            address,
            rtt_address,
            monitor_address,
        )

        def close_window():
            state.set_host_keys(0)
            stop_event.set()
            root.quit()

        root.protocol("WM_DELETE_WINDOW", close_window)
        signal.signal(signal.SIGINT, lambda _signum, _frame: close_window())
        framebuffer_worker.start()
        rtt_worker.start()
        root.mainloop()
    except ViewerError as error:
        print(f"error: {error}", file=sys.stderr)
        return_code = 2
    except KeyboardInterrupt:
        return_code = 130
    finally:
        stop_event.set()
        if framebuffer_worker.is_alive():
            framebuffer_worker.join(timeout=SOCKET_TIMEOUT + 0.25)
        if rtt_worker.is_alive():
            rtt_worker.join(timeout=SOCKET_TIMEOUT + 0.25)
        try:
            root.destroy()
        except tk.TclError:
            pass
        if managed_server is not None:
            managed_server.stop()
    return return_code


if __name__ == "__main__":
    sys.exit(main())
