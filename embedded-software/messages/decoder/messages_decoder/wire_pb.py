"""Pure-Python proto3 wire-format reader.

Just enough proto3 to decode the nanopb output produced by the codegen
.proto emitter, using registry field schemas as the descriptor source.
Avoids dragging the protobuf python package + grpcio-tools into the
decoder's runtime deps; the package stays stdlib-only.

Wire format reminder (proto3):
  Each field is [tag varint][value], where
    tag = (field_number << 3) | wire_type
    wire types: 0=VARINT, 1=64-BIT, 2=LENGTH-DELIM, 5=32-BIT
  Field numbers come from the registry's field declaration order
  (start at 1), matching what emit_proto.py uses.

Unknown fields (wire types we don't expect, or field numbers we don't
recognise) are skipped — proto3's forward-compat story.
"""
from __future__ import annotations

import struct
from typing import Any, Dict, List, Tuple


WIRE_VARINT = 0
WIRE_64BIT = 1
WIRE_LENGTH_DELIM = 2
WIRE_32BIT = 5


class ProtoDecodeError(Exception):
    pass


def _read_varint(buf: bytes, pos: int) -> Tuple[int, int]:
    """Read one varint starting at pos; return (value, new_pos)."""
    shift = 0
    result = 0
    while True:
        if pos >= len(buf):
            raise ProtoDecodeError("truncated varint")
        b = buf[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            return result, pos
        shift += 7
        if shift > 63:
            raise ProtoDecodeError("varint too long")


def _zigzag_decode(n: int) -> int:
    """proto3 sint32/sint64 use zigzag; we don't emit sint, but harmless to have."""
    return (n >> 1) ^ -(n & 1)


def _decode_value(buf: bytes, pos: int, wire_type: int, proto_kind: str) -> Tuple[Any, int]:
    """Decode one value of the given wire type, interpreted per proto_kind.

    proto_kind: 'uint32', 'uint64', 'int32', 'int64', 'float', 'double',
                'bool', 'string', 'bytes', 'message:<msg_name>'.
    """
    if wire_type == WIRE_VARINT:
        v, pos = _read_varint(buf, pos)
        if proto_kind == "bool":
            return bool(v), pos
        if proto_kind in ("int32", "int64"):
            # proto3 int32/int64 are stored as varint with no zigzag;
            # negatives sign-extend to 10 bytes. Recover sign by checking
            # the appropriate bit width.
            if proto_kind == "int32":
                if v >= 1 << 31:
                    v -= 1 << 32
            else:
                if v >= 1 << 63:
                    v -= 1 << 64
            return v, pos
        # uint32 / uint64 — return as-is.
        return v, pos
    if wire_type == WIRE_64BIT:
        if pos + 8 > len(buf):
            raise ProtoDecodeError("truncated 64-bit field")
        chunk = buf[pos : pos + 8]
        pos += 8
        if proto_kind == "double":
            (v,) = struct.unpack("<d", chunk)
            return v, pos
        if proto_kind in ("fixed64", "uint64"):
            (v,) = struct.unpack("<Q", chunk)
            return v, pos
        if proto_kind == "sfixed64":
            (v,) = struct.unpack("<q", chunk)
            return v, pos
        return chunk, pos  # unknown — return raw
    if wire_type == WIRE_32BIT:
        if pos + 4 > len(buf):
            raise ProtoDecodeError("truncated 32-bit field")
        chunk = buf[pos : pos + 4]
        pos += 4
        if proto_kind == "float":
            (v,) = struct.unpack("<f", chunk)
            return v, pos
        if proto_kind in ("fixed32", "uint32"):
            (v,) = struct.unpack("<I", chunk)
            return v, pos
        if proto_kind == "sfixed32":
            (v,) = struct.unpack("<i", chunk)
            return v, pos
        return chunk, pos
    if wire_type == WIRE_LENGTH_DELIM:
        length, pos = _read_varint(buf, pos)
        if pos + length > len(buf):
            raise ProtoDecodeError("truncated length-delimited field")
        chunk = buf[pos : pos + length]
        pos += length
        if proto_kind == "string":
            return chunk.decode("utf-8", errors="replace"), pos
        if proto_kind == "bytes":
            return bytes(chunk), pos
        if proto_kind.startswith("message:"):
            return chunk, pos  # caller decodes recursively using its descriptor
        return bytes(chunk), pos
    raise ProtoDecodeError(f"unsupported wire type {wire_type}")


def _skip_unknown(buf: bytes, pos: int, wire_type: int) -> int:
    """Advance past one field we don't have a descriptor for."""
    if wire_type == WIRE_VARINT:
        _, pos = _read_varint(buf, pos)
        return pos
    if wire_type == WIRE_64BIT:
        return pos + 8
    if wire_type == WIRE_32BIT:
        return pos + 4
    if wire_type == WIRE_LENGTH_DELIM:
        length, pos = _read_varint(buf, pos)
        return pos + length
    raise ProtoDecodeError(f"can't skip wire type {wire_type}")


def decode_message(
    buf: bytes,
    descriptor: Dict[int, Tuple[str, str, bool]],
    sub_decoders: Dict[str, "Callable[[bytes], Dict[str, Any]]"] | None = None,
) -> Dict[str, Any]:
    """Decode one proto3-encoded message.

    descriptor: {field_number: (field_name, proto_kind, is_repeated)}
        proto_kind is one of the strings _decode_value recognises, OR
        "message:<key>" where sub_decoders[key] is a callable producing
        a dict from raw nested-message bytes.
    sub_decoders: name -> decode-callable for nested messages.

    Defaults applied per proto3:
      * missing scalar field -> Python equivalent (0 / 0.0 / False / "" / b"")
      * missing repeated field -> empty list
      * missing nested message -> None
    """
    sub_decoders = sub_decoders or {}

    out: Dict[str, Any] = {}
    # Pre-populate with proto3 defaults so missing fields are explicit.
    for fn, (name, kind, repeated) in descriptor.items():
        if repeated:
            out[name] = []
        elif kind == "string":
            out[name] = ""
        elif kind == "bytes":
            out[name] = b""
        elif kind == "bool":
            out[name] = False
        elif kind in ("float", "double"):
            out[name] = 0.0
        elif kind.startswith("message:"):
            out[name] = None
        else:
            out[name] = 0

    pos = 0
    while pos < len(buf):
        tag, pos = _read_varint(buf, pos)
        wire_type = tag & 0x7
        field_number = tag >> 3
        if field_number not in descriptor:
            pos = _skip_unknown(buf, pos, wire_type)
            continue
        name, kind, repeated = descriptor[field_number]
        v, pos = _decode_value(buf, pos, wire_type, kind)
        if kind.startswith("message:"):
            sub_key = kind.split(":", 1)[1]
            decode = sub_decoders.get(sub_key)
            if decode is None:
                raise ProtoDecodeError(f"no sub-decoder for nested {sub_key}")
            v = decode(v)
        if repeated:
            out[name].append(v)
        else:
            out[name] = v

    return out
