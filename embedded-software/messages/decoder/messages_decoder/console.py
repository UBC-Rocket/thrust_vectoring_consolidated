"""messages-console — minimal bidirectional CLI for the message registry.

Sends one command request to firmware and decodes the response. Speaks
the locked envelope format directly: length + class + module + msg/cmd
+ t_us + payload + CRC. No COBS yet — Phase-3 wire framing (COBS over
UART) lands when the VCP RX driver does. For now this works against any
byte transport that delivers frames intact (test fixtures, future
TCP/UDP shim).

Usage:
    messages-console <transport> <module>.<command> [--args JSON]
      <transport>:  /dev/cu.usbmodemNNN (needs pyserial),
                    OR "-" to use stdin/stdout (paired with a firmware
                    or test harness via shell pipe).
      <module>.<command>: e.g. system.get_build_info
      --args:       JSON object of request field values; defaults to {}.
      --registry:   path to registry.json (default: in-tree)
      --timeout:    seconds to wait for response (default 2.0)

Exit codes:
  0  success — response decoded
  1  unknown command / decode error
  2  transport error (timeout, port not opened)
  3  registry mismatch (firmware's registry_hash != ours)
"""
from __future__ import annotations

import argparse
import io
import json
import os
import struct
import sys
import time
from typing import Any, BinaryIO

from .encoder import (
    CLASS_CMD_REQ,
    CLASS_CMD_RESP,
    decode_command_response,
    encode_command_request,
)
from .registry import load_registry
from .wire import (
    CRC_SIZE,
    ENVELOPE_SIZE,
    LENGTH_PREFIX_SIZE,
    _ENVELOPE_STRUCT,
    crc16_ccitt_false,
)
from .wire_cobs import cobs_decode, cobs_encode


def _open_transport(spec: str) -> tuple[BinaryIO, BinaryIO]:
    """Return (reader, writer) byte streams for the chosen transport."""
    if spec == "-":
        # Pair with a firmware/test harness via shell pipe:
        #   ./firmware-host-stub | messages-console - <cmd> | ...
        return sys.stdin.buffer, sys.stdout.buffer
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError:
        raise SystemExit(
            "serial transport requested but pyserial is not installed.\n"
            "  pip install pyserial\n"
            "or pass `-` to use stdin/stdout instead."
        )
    s = serial.Serial(spec, baudrate=921600, timeout=0.1)
    # serial.Serial implements read()/write() but isn't a BinaryIO subclass;
    # the rest of this code only uses .read(n) / .write(b) so it duck-types fine.
    return s, s   # type: ignore[return-value]


def _read_exact(stream: BinaryIO, n: int, timeout_s: float) -> bytes:
    """Read exactly n bytes from stream, respecting a wall-clock deadline."""
    buf = bytearray()
    deadline = time.monotonic() + timeout_s
    while len(buf) < n:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"timed out after {timeout_s:.2f}s waiting for {n} bytes (got {len(buf)})")
        chunk = stream.read(n - len(buf))
        if not chunk:
            time.sleep(0.005)
            continue
        buf.extend(chunk)
    return bytes(buf)


def _read_frame(stream: BinaryIO, timeout_s: float) -> bytes:
    """Read one length-prefixed record from the stream (no framing)."""
    len_bytes = _read_exact(stream, LENGTH_PREFIX_SIZE, timeout_s)
    (length,) = struct.unpack("<H", len_bytes)
    body = _read_exact(stream, length, timeout_s)
    return len_bytes + body


def _read_cobs_frame(stream: BinaryIO, timeout_s: float) -> bytes:
    """Read bytes until 0x00 (COBS delimiter), decode, return raw record."""
    buf = bytearray()
    deadline = time.monotonic() + timeout_s
    while True:
        if time.monotonic() > deadline:
            raise TimeoutError(f"timed out after {timeout_s:.2f}s waiting for COBS frame (got {len(buf)} bytes)")
        b = stream.read(1)
        if not b:
            time.sleep(0.005)
            continue
        if b == b"\x00":
            if not buf:
                continue  # stray delimiter; keep waiting
            return cobs_decode(bytes(buf))
        buf += b


def _decode_response_frame(frame: bytes) -> tuple[int, int, int, int, bytes]:
    """Parse a CMD_RESP envelope. Returns (class, module_id, cmd_id, t_us, payload)."""
    (length,) = struct.unpack("<H", frame[:LENGTH_PREFIX_SIZE])
    body = frame[LENGTH_PREFIX_SIZE:]
    if len(body) != length:
        raise ValueError(f"truncated response: length={length}, got {len(body)} bytes")
    envelope = body[:ENVELOPE_SIZE]
    payload = body[ENVELOPE_SIZE : len(body) - CRC_SIZE]
    (got_crc,) = struct.unpack("<H", body[len(body) - CRC_SIZE :])
    want_crc = crc16_ccitt_false(body[: len(body) - CRC_SIZE])
    if got_crc != want_crc:
        raise ValueError(f"crc mismatch: got 0x{got_crc:04x} want 0x{want_crc:04x}")
    class_byte, module_id, cmd_id, t_us = _ENVELOPE_STRUCT.unpack(envelope)
    return class_byte, module_id, cmd_id, t_us, payload


def main(argv: list[str] | None = None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    default_registry = os.path.normpath(
        os.path.join(here, "..", "..", "registry.json")
    )

    parser = argparse.ArgumentParser(
        prog="messages-console",
        description="Send a command request to firmware and decode the response.",
    )
    parser.add_argument("transport", help="Serial port path, or '-' for stdin/stdout.")
    parser.add_argument("command", help="<module>.<command>, e.g. system.get_build_info")
    parser.add_argument("--args", default="{}", help="JSON object of request fields")
    parser.add_argument("--registry", default=default_registry,
                        help=f"Path to registry.json (default {default_registry})")
    parser.add_argument("--timeout", type=float, default=2.0,
                        help="Seconds to wait for response (default 2.0)")
    parser.add_argument("--framing", choices=("auto", "none", "cobs"), default="auto",
                        help="Wire framing. 'auto' picks cobs for serial transports "
                             "and none for '-' (stdin/stdout). The firmware VCP "
                             "channel uses cobs; matches registry.channels.vcp.framing.")
    args = parser.parse_args(argv)

    try:
        module_name, command_name = args.command.split(".", 1)
    except ValueError:
        print(f"command must be <module>.<command>, got {args.command!r}", file=sys.stderr)
        return 1

    try:
        request_fields = json.loads(args.args)
    except json.JSONDecodeError as e:
        print(f"--args is not valid JSON: {e}", file=sys.stderr)
        return 1

    registry = load_registry(args.registry)
    try:
        cmd_frame = encode_command_request(
            registry, module_name, command_name, request_fields
        )
    except KeyError as e:
        print(f"unknown command in registry: {e}", file=sys.stderr)
        return 1

    try:
        reader, writer = _open_transport(args.transport)
    except OSError as e:
        print(f"transport open failed: {e}", file=sys.stderr)
        return 2

    use_cobs = (args.framing == "cobs"
                or (args.framing == "auto" and args.transport != "-"))

    if use_cobs:
        wire_bytes = cobs_encode(cmd_frame) + b"\x00"
    else:
        wire_bytes = cmd_frame
    writer.write(wire_bytes)
    if hasattr(writer, "flush"):
        writer.flush()

    try:
        if use_cobs:
            frame = _read_cobs_frame(reader, args.timeout)
        else:
            frame = _read_frame(reader, args.timeout)
    except TimeoutError as e:
        print(f"timeout: {e}", file=sys.stderr)
        return 2
    except ValueError as e:
        print(f"frame decode failed: {e}", file=sys.stderr)
        return 1

    try:
        class_byte, module_id, cmd_id, t_us, payload = _decode_response_frame(frame)
    except ValueError as e:
        print(f"response decode failed: {e}", file=sys.stderr)
        return 1

    if class_byte != CLASS_CMD_RESP:
        print(f"unexpected class 0x{class_byte:02x} (want CMD_RESP 0x{CLASS_CMD_RESP:02x})",
              file=sys.stderr)
        return 1

    try:
        fields = decode_command_response(registry, module_name, command_name, payload)
    except Exception as e:
        print(f"response payload decode failed: {e}", file=sys.stderr)
        return 1

    out = {
        "command": f"{module_name}.{command_name}",
        "t_us": t_us,
        "fields": fields,
    }

    # If the response carries registry_hash, verify against ours.
    if "registry_hash" in fields:
        if fields["registry_hash"] != registry.crc32:
            print(
                f"WARN: firmware registry_hash 0x{fields['registry_hash']:08x} "
                f"!= host registry_hash 0x{registry.crc32:08x}",
                file=sys.stderr,
            )
            print(json.dumps(out))
            return 3

    print(json.dumps(out))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
