"""Wire-format primitives: CRC16-CCITT/FALSE and the SD record framing.

Record layout (all little-endian, concatenated back-to-back):

    [u16 length]               # bytes that follow this field
    [u8  class]                # 0x41='A', 0x42='B'
    [u8  module_id]
    [u16 msg_id]
    [u64 t_us_publish]         # envelope time
    [u8  payload[N]]           # Class A: packed struct; Class B: nanopb
    [u16 crc16_ccitt]          # covers [class..payload], i.e. 12 + N bytes

Total record on disk = 2 + length = 16 + N bytes.
``length`` itself is therefore ``12 + N + 2 = 14 + N`` bytes.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional

# Envelope = class(1) + module_id(1) + msg_id(2) + t_us_publish(8) = 12 bytes.
ENVELOPE_SIZE = 12
CRC_SIZE = 2
# The smallest legal record covers an envelope + CRC (no payload bytes).
MIN_RECORD_LENGTH = ENVELOPE_SIZE + CRC_SIZE  # 14
LENGTH_PREFIX_SIZE = 2

CLASS_A = 0x41
CLASS_B = 0x42

# Pre-compiled envelope struct: <class:u8><module:u8><msg:u16><t_us:u64>.
_ENVELOPE_STRUCT = struct.Struct("<BBHQ")


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE.

    Polynomial 0x1021, init 0xFFFF, no input/output reflection, no final
    xor. The classic catch-22 test vector ``crc16("123456789") == 0x29B1``
    is enforced by the test-suite.
    """

    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass
class Record:
    """Base class for all decoded record objects yielded by the decoder.

    Generated per-message dataclasses inherit from this and add their
    payload fields plus the envelope ``t_us_publish``. ``class_byte``,
    ``module_id`` and ``msg_id`` come straight off the wire.
    """

    t_us_publish: int


@dataclass
class UnsupportedClassBRecord:
    """Backwards-compat placeholder. No longer yielded by the decoder —
    Class B records now decode to ``ClassBRecord``. Retained so existing
    callers that ``isinstance(rec, UnsupportedClassBRecord)`` keep linking.
    """

    module_id: int
    msg_id: int
    t_us_publish: int
    payload: bytes


@dataclass
class ClassBRecord:
    """Decoded Class B (nanopb/proto3) record.

    Carries the envelope identifiers + a ``fields`` dict produced by the
    pure-Python proto3 reader against a descriptor derived from the
    registry. Nested user types decode to nested dicts; ``enum:*`` fields
    decode as plain ``int`` (consumers can map them through the registry
    enums if needed — kept untyped here to keep the record class
    structurally identical across every Class B message).
    """

    module_id: int
    msg_id: int
    t_us_publish: int
    full_name: str        # "<module>.<message>"
    fields: dict          # decoded payload, keyed by field name


@dataclass
class UnknownMsgRecord:
    """Yielded when a record's (module_id, msg_id) isn't in the registry.

    Decoding continues; this is normal during cross-version playback.
    """

    class_byte: int
    module_id: int
    msg_id: int
    t_us_publish: int
    payload: bytes


@dataclass
class DecodeError:
    """Yielded when a record fails CRC or struct unpack.

    ``offset`` is the byte offset of the length prefix within the log
    file; ``reason`` is a short human string suitable for logging.
    """

    offset: int
    reason: str
    raw: Optional[bytes] = None
