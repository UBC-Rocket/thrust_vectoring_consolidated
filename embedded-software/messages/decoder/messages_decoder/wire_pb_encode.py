"""Pure-Python proto3 wire-format encoder.

Companion to wire_pb (decoder). Encodes a field dict into proto3 bytes
using a descriptor produced by types._build_pb_descriptor — the same
schema firmware uses on the decode side.

Supports the type set the registry can express:
  * uint32 / uint64 / int32 / int64 (varint)
  * float (32-bit LE)
  * double (64-bit LE)
  * bool (varint 0/1)
  * string / bytes (length-prefixed)
  * message:<name> (nested — recursive encode via sub_encoders)

Repeated fields encode as packed-or-unpacked per proto3 conventions —
we go unpacked (one tag per element) which is simpler and proto3-
default for non-scalars; for scalars it's also legal and matches the
nanopb default unless (nanopb).packed_repeated is on.
"""
from __future__ import annotations

import struct
from typing import Any, Callable, Dict, Tuple


WIRE_VARINT = 0
WIRE_64BIT = 1
WIRE_LENGTH_DELIM = 2
WIRE_32BIT = 5


def _encode_varint(value: int) -> bytes:
    """Encode an unsigned int as a varint (proto3, no zigzag)."""
    if value < 0:
        # int32/int64 negative — sign-extend to 64 bits as proto3 does.
        value &= (1 << 64) - 1
    out = bytearray()
    while value > 0x7F:
        out.append((value & 0x7F) | 0x80)
        value >>= 7
    out.append(value & 0x7F)
    return bytes(out)


def _wire_type_for(kind: str) -> int:
    if kind in ("uint32", "uint64", "int32", "int64", "bool"):
        return WIRE_VARINT
    if kind == "float":
        return WIRE_32BIT
    if kind == "double":
        return WIRE_64BIT
    if kind in ("string", "bytes") or kind.startswith("message:"):
        return WIRE_LENGTH_DELIM
    raise ValueError(f"unknown proto kind {kind!r}")


def _encode_scalar(kind: str, value: Any,
                    sub_encoders: Dict[str, Callable[[Dict[str, Any]], bytes]]) -> bytes:
    if kind in ("uint32", "uint64"):
        return _encode_varint(int(value))
    if kind in ("int32", "int64"):
        return _encode_varint(int(value))
    if kind == "bool":
        return _encode_varint(1 if value else 0)
    if kind == "float":
        return struct.pack("<f", float(value))
    if kind == "double":
        return struct.pack("<d", float(value))
    if kind == "string":
        b = value.encode("utf-8") if isinstance(value, str) else bytes(value)
        return _encode_varint(len(b)) + b
    if kind == "bytes":
        b = bytes(value)
        return _encode_varint(len(b)) + b
    if kind.startswith("message:"):
        sub_key = kind.split(":", 1)[1]
        enc = sub_encoders.get(sub_key)
        if enc is None:
            raise ValueError(f"no sub-encoder for nested {sub_key}")
        sub_bytes = enc(value)  # value is expected to be a dict
        return _encode_varint(len(sub_bytes)) + sub_bytes
    raise ValueError(f"unknown proto kind {kind!r}")


def encode_message(
    descriptor: Dict[int, Tuple[str, str, bool]],
    fields: Dict[str, Any],
    sub_encoders: Dict[str, Callable[[Dict[str, Any]], bytes]] | None = None,
) -> bytes:
    """Encode a single message per the given descriptor.

    descriptor: {field_number: (field_name, proto_kind, is_repeated)}
    fields:    {field_name: python_value}; missing fields omitted (proto3
               default).
    sub_encoders: name -> callable for nested message types.
    """
    sub_encoders = sub_encoders or {}
    out = bytearray()
    for fn, (name, kind, repeated) in descriptor.items():
        if name not in fields:
            continue
        value = fields[name]
        wire = _wire_type_for(kind)
        tag = _encode_varint((fn << 3) | wire)
        if repeated:
            for elem in value:
                out += tag
                out += _encode_scalar(kind, elem, sub_encoders)
        else:
            out += tag
            out += _encode_scalar(kind, value, sub_encoders)
    return bytes(out)
