"""Host-side decoder for UBC Rocket message registry SD log records.

Phase 1 scope:
- Decode Class A (packed) messages emitted to the SD channel.
- User-defined composite types become nested dataclasses.
- Enum-typed fields decode to a dynamically-built IntEnum subclass per
  (module, error-namespace) so the consumer gets both the numeric code
  (via ``int(value)``) and the symbolic name (via ``value.name``).
- Class B (nanopb/proto3) records decode via a pure-Python proto3 reader
  driven by descriptors built from the registry; yielded as
  ``ClassBRecord`` instances with a ``fields`` dict. Nested user types
  decode to nested dicts.
- CRC failures yield a ``DecodeError`` sentinel; decoding continues.
- Unknown msg_ids yield an ``UnknownMsgRecord``; decoding continues.

The decoder reads the registry at runtime — there is no ahead-of-time
codegen on the Python side. The runtime registry CRC32 is computed for a
future fail-closed check against an in-log header (firmware does not emit
that header yet; see ``Decoder._check_registry_crc``).
"""

from .registry import load_registry, Registry, registry_crc32
from .types import build_decoder, Decoder
from .wire import (
    ClassBRecord,
    Record,
    UnsupportedClassBRecord,
    DecodeError,
    UnknownMsgRecord,
    crc16_ccitt_false,
)

__all__ = [
    "load_registry",
    "Registry",
    "registry_crc32",
    "build_decoder",
    "Decoder",
    "Record",
    "ClassBRecord",
    "UnsupportedClassBRecord",
    "DecodeError",
    "UnknownMsgRecord",
    "crc16_ccitt_false",
]
