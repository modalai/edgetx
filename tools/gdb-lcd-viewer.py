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

"""Display an EdgeTX monochrome framebuffer and RTT log through J-Link."""

import argparse
import codecs
from collections import deque
from dataclasses import dataclass
from pathlib import Path
import re
import signal
import socket
import sys
import threading
import time
import tkinter as tk

from elftools.elf.elffile import ELFFile
from elftools.elf.sections import SymbolTableSection
from PIL import Image, ImageOps, ImageTk


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 2331
DEFAULT_RTT_PORT = 19021
DEFAULT_SYMBOL = "displayBuf"
DEFAULT_RTT_SYMBOL = "_SEGGER_RTT"
DEFAULT_WIDTH = 128
DEFAULT_HEIGHT = 64
DEFAULT_FPS = 5.0
DEFAULT_SCALE = 6
SOCKET_TIMEOUT = 0.5
RECONNECT_DELAY = 1.0
MAX_READ_CHUNK = 1024
STABLE_READ_COMPARISONS = 3
DEFAULT_LOG_HEIGHT = 240
MAX_LOG_LINES = 5000
MAX_PENDING_RTT_CHUNKS = 256


class ViewerError(Exception):
    """Report an error that the user can correct."""


class RemoteProtocolError(ViewerError):
    """Report an invalid or failed GDB remote operation."""


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


def resolve_symbol(elf_path, symbol_name, expected_size=None):
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

        addresses = {int(symbol["st_value"]) for symbol in sized_matches}
        if len(addresses) != 1:
            values = ", ".join(f"0x{address:08x}" for address in sorted(addresses))
            raise ViewerError(f"symbol '{symbol_name}' has multiple addresses: {values}")

        return addresses.pop()


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


class ViewerState:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.generation = 0
        self.status = "not connected"
        self.frame_rate = 0.0
        self.unstable_reads = 0
        self.rtt_status = "wait for GDB"
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

    def append_rtt(self, text):
        with self.lock:
            self.rtt_chunks.append(text)

    def take_rtt(self):
        with self.lock:
            chunks = list(self.rtt_chunks)
            self.rtt_chunks.clear()
            return chunks

    def snapshot(self):
        with self.lock:
            return ViewerSnapshot(
                self.frame,
                self.generation,
                self.status,
                self.frame_rate,
                self.unstable_reads,
                self.rtt_status,
            )


def read_stable_frame(client, address, size):
    previous = client.read_memory(address, size)
    for _ in range(STABLE_READ_COMPARISONS):
        current = client.read_memory(address, size)
        if current == previous:
            return current
        previous = current
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

                next_read += period
                if next_read < accepted_at:
                    next_read = accepted_at + period
        except (OSError, ViewerError) as error:
            rtt_ready_event.clear()
            state.set_status(f"reconnect: {error}")
            client.close()
            client = None

    if client is not None:
        client.close()
    rtt_ready_event.clear()


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
            rtt_socket.settimeout(SOCKET_TIMEOUT)
            configuration = (
                "$$SEGGER_TELNET_ConfigStr=RTTCh;0;"
                f"SetRTTAddr;0x{rtt_address:08x};$$"
            )
            rtt_socket.sendall(configuration.encode("ascii"))
            state.set_rtt_status("connected")
            decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")

            while not stop_event.is_set():
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
                rtt_socket.close()


class ViewerWindow:
    def __init__(
        self, root, state, width, height, scale, invert, address, rtt_address
    ):
        self.root = root
        self.state = state
        self.width = width
        self.height = height
        self.invert = invert
        self.address = address
        self.rtt_address = rtt_address
        self.generation = -1
        self.base_image = None
        self.photo = None

        self.root.geometry(
            f"{width * scale}x{height * scale + DEFAULT_LOG_HEIGHT}"
        )
        self.root.minsize(width, height + 80)

        self.panes = tk.PanedWindow(
            root,
            orient=tk.VERTICAL,
            background="#303030",
            sashrelief=tk.RAISED,
            sashwidth=6,
        )
        self.panes.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(
            self.panes,
            width=width * scale,
            height=height * scale,
            background="#303030",
            highlightthickness=0,
        )
        self.panes.add(self.canvas, minsize=height)
        self.canvas.bind("<Configure>", self._resize)

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
        self.root.after_idle(
            lambda: self.panes.sash_place(0, 0, height * scale)
        )
        self.root.after(50, self._update)

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
        self.root.title(
            f"EdgeTX LCD 0x{self.address:08x} | LCD {snapshot.status} | "
            f"{snapshot.frame_rate:.1f} Hz | RTT 0x{self.rtt_address:08x} "
            f"{snapshot.rtt_status}"
        )
        self._append_rtt(self.state.take_rtt())
        if snapshot.frame is not None and snapshot.generation != self.generation:
            self.generation = snapshot.generation
            self.base_image = framebuffer_image(
                snapshot.frame, self.width, self.height, self.invert
            )
            self._render()
        self.root.after(50, self._update)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Display an EdgeTX monochrome framebuffer and RTT log.",
        epilog=(
            "The viewer opens a second connection to an existing J-Link GDB server. "
            "It configures RTT, resumes the target, and reads RTT channel 0. The "
            "server must expose its RTT telnet port. The viewer never resets, halts, "
            "or writes target memory."
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
    parser.add_argument(
        "--symbol", default=DEFAULT_SYMBOL, help="framebuffer ELF symbol"
    )
    parser.add_argument(
        "--rtt-symbol",
        default=DEFAULT_RTT_SYMBOL,
        help="RTT control block ELF symbol",
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

    state = ViewerState()
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

    window = ViewerWindow(
        root,
        state,
        arguments.width,
        arguments.height,
        arguments.scale,
        arguments.invert,
        address,
        rtt_address,
    )

    def close_window():
        stop_event.set()
        root.quit()

    root.protocol("WM_DELETE_WINDOW", close_window)
    signal.signal(signal.SIGINT, lambda _signum, _frame: close_window())
    framebuffer_worker.start()
    rtt_worker.start()
    try:
        root.mainloop()
    finally:
        stop_event.set()
        framebuffer_worker.join(timeout=SOCKET_TIMEOUT + 0.25)
        rtt_worker.join(timeout=SOCKET_TIMEOUT + 0.25)
        root.destroy()
        del window
    return 0


if __name__ == "__main__":
    sys.exit(main())
