"""Host-side encoder for command requests + Class B publishes.

Mirrors firmware's messages_publish_b + the dispatcher's request side.
Same wire format the runtime expects: [u16 length][envelope][payload][u16 crc].

Used by the messages-console CLI to ship command requests to firmware
over a serial link (or any byte transport).

Pure-Python — uses the proto3 encoder in wire_pb_encode (companion to
wire_pb's decoder).
"""
from __future__ import annotations

import struct
from typing import Any, Dict

from .registry import Registry
from .wire import (
    CLASS_B,
    CRC_SIZE,
    ENVELOPE_SIZE,
    LENGTH_PREFIX_SIZE,
    _ENVELOPE_STRUCT,
    crc16_ccitt_false,
)
from .wire_pb_encode import encode_message
from .types import _build_pb_descriptor


# Wire class for command request / response. Must match firmware
# generated/messages/registry.h.
CLASS_CMD_REQ = 0x43
CLASS_CMD_RESP = 0x44


def _build_envelope(msg_class: int, module_id: int, msg_id: int,
                    t_us_publish: int, payload: bytes) -> bytes:
    envelope = _ENVELOPE_STRUCT.pack(msg_class, module_id, msg_id, t_us_publish)
    body = envelope + payload
    crc = crc16_ccitt_false(body)
    record = body + struct.pack("<H", crc)
    return struct.pack("<H", len(record)) + record


def encode_command_request(registry: Registry, module_name: str,
                            command_name: str, fields: Dict[str, Any] | None = None,
                            t_us_publish: int = 0) -> bytes:
    """Encode a command request frame ready to ship over the wire.

    fields: dict of request-payload field names → Python values (per the
            registry's command.request schema). Missing fields default to
            proto3 zero values.
    Returns the framed bytes (length-prefixed envelope + crc).
    """
    module = registry.modules[module_name]
    module_id = module["module_id"]
    cmd = module["commands"][command_name]
    cmd_id = cmd["cmd_id"]

    descriptor = _build_pb_descriptor(cmd["request"])
    payload = encode_message(descriptor, fields or {})
    return _build_envelope(CLASS_CMD_REQ, module_id, cmd_id, t_us_publish, payload)


def decode_command_response(registry: Registry, module_name: str,
                             command_name: str, payload: bytes) -> Dict[str, Any]:
    """Decode a response payload (post-envelope) into a field dict.

    Caller is responsible for stripping the envelope (length, class,
    module_id, msg_id, t_us, payload, crc). For raw-frame decode use
    Decoder.iter_records — but iter_records doesn't yet know about
    CMD_RESP class; the messages-console CLI handles that itself.
    """
    from .wire_pb import decode_message
    cmd = registry.modules[module_name]["commands"][command_name]
    descriptor = _build_pb_descriptor(cmd["response"]["fields"])
    return decode_message(payload, descriptor)
