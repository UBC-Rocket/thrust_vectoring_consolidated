"""Unit tests for the messages_decoder library + CLI.

Phase-1 coverage:

  * CRC16-CCITT/FALSE: catch-22 "123456789" → 0x29B1 vector.
  * Hand-crafted state_estimation.state_estimate record decodes
    byte-for-byte to the expected dataclass values.
  * CRC mismatch yields ``DecodeError`` and decoding continues.
  * Unknown msg_id yields ``UnknownMsgRecord`` and decoding continues.
  * Class B record yields ``UnsupportedClassBRecord``.
  * The CLI in jsonl and csv modes produces the expected output.
"""

from __future__ import annotations

import io
import json
import os
import struct
import subprocess
import sys
import tempfile
import unittest
import warnings

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
DECODER_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))

# Ensure the package is importable without install.
sys.path.insert(0, DECODER_ROOT)

from messages_decoder import (  # noqa: E402
    build_decoder,
    crc16_ccitt_false,
    load_registry,
    ClassBRecord,
    Record,
    UnsupportedClassBRecord,
    DecodeError,
    UnknownMsgRecord,
)
from messages_decoder.wire import CLASS_A, CLASS_B  # noqa: E402


def _proto3_varint(value: int) -> bytes:
    """Encode a varint the way proto3 does (no zigzag)."""
    out = bytearray()
    while value > 0x7F:
        out.append((value & 0x7F) | 0x80)
        value >>= 7
    out.append(value & 0x7F)
    return bytes(out)


def _proto3_field(number: int, wire_type: int, body: bytes) -> bytes:
    return _proto3_varint((number << 3) | wire_type) + body


REGISTRY_PATH = os.path.join(REPO_ROOT, "embedded-software", "messages", "registry.json")


def make_record(class_byte: int, module_id: int, msg_id: int,
                t_us_publish: int, payload: bytes,
                *, corrupt_crc: bool = False) -> bytes:
    """Build one wire-format SD record. Mirrors the encoder side bit-for-bit."""

    envelope = struct.pack("<BBHQ", class_byte, module_id, msg_id, t_us_publish)
    body = envelope + payload
    crc = crc16_ccitt_false(body)
    if corrupt_crc:
        crc ^= 0xFFFF
    record = body + struct.pack("<H", crc)
    return struct.pack("<H", len(record)) + record


class TestCRC(unittest.TestCase):
    def test_catch22(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_empty(self):
        # Init value, no bytes processed.
        self.assertEqual(crc16_ccitt_false(b""), 0xFFFF)


class TestStateEstimateDecode(unittest.TestCase):
    def setUp(self):
        self.registry = load_registry(REGISTRY_PATH)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            self.decoder = build_decoder(self.registry)

    def _state_estimate_payload(self):
        # Field order: t_us (u64), position (3 f32), velocity (3 f32),
        # attitude (4 f32), gyro_bias (3 f32), accel_bias (3 f32)
        # = 8 + (3+3+4+3+3)*4 = 8 + 64 = 72 bytes.
        # Use values that round-trip exactly through f32.
        t_us = 1234567890
        pos = (1.0, 2.0, 3.0)
        vel = (-0.5, 0.25, 0.125)
        att = (1.0, 0.0, 0.0, 0.0)
        gyro_bias = (0.5, 0.25, 0.125)
        accel_bias = (0.5, 0.25, 0.125)
        fmt = "<Q" + "fff" + "fff" + "ffff" + "fff" + "fff"
        return (
            struct.pack(fmt, t_us, *pos, *vel, *att, *gyro_bias, *accel_bias),
            dict(t_us=t_us, pos=pos, vel=vel, att=att,
                 gyro_bias=gyro_bias, accel_bias=accel_bias),
        )

    def test_decode_state_estimate(self):
        payload, expected = self._state_estimate_payload()
        # state_estimation: module_id=1, msg_id=1
        rec_bytes = make_record(CLASS_A, 1, 1, t_us_publish=9999, payload=payload)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            records = list(self.decoder.iter_records(io.BytesIO(rec_bytes)))
        self.assertEqual(len(records), 1)
        rec = records[0]
        self.assertEqual(type(rec).__name__, "StateEstimationStateEstimate")
        self.assertIsInstance(rec, Record)
        self.assertEqual(rec.t_us_publish, 9999)
        self.assertEqual(rec.t_us, expected["t_us"])
        # Nested composites.
        self.assertEqual((rec.position.x, rec.position.y, rec.position.z), expected["pos"])
        self.assertEqual((rec.velocity.x, rec.velocity.y, rec.velocity.z), expected["vel"])
        self.assertEqual(
            (rec.attitude.w, rec.attitude.x, rec.attitude.y, rec.attitude.z),
            expected["att"],
        )
        self.assertAlmostEqual(rec.gyro_bias.x, expected["gyro_bias"][0])
        self.assertAlmostEqual(rec.accel_bias.z, expected["accel_bias"][2])

    def test_crc_mismatch_continues(self):
        payload, _ = self._state_estimate_payload()
        bad = make_record(CLASS_A, 1, 1, t_us_publish=1, payload=payload, corrupt_crc=True)
        good = make_record(CLASS_A, 1, 1, t_us_publish=2, payload=payload)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            records = list(self.decoder.iter_records(io.BytesIO(bad + good)))
        self.assertEqual(len(records), 2)
        self.assertIsInstance(records[0], DecodeError)
        self.assertIn("crc", records[0].reason)
        self.assertEqual(type(records[1]).__name__, "StateEstimationStateEstimate")
        self.assertEqual(records[1].t_us_publish, 2)

    def test_unknown_msg_id(self):
        # module_id=1 exists (state_estimation) but msg_id=42 does not.
        rec = make_record(CLASS_A, 1, 42, t_us_publish=7, payload=b"\x00\x01\x02\x03")
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            records = list(self.decoder.iter_records(io.BytesIO(rec)))
        self.assertEqual(len(records), 1)
        self.assertIsInstance(records[0], UnknownMsgRecord)
        self.assertEqual(records[0].module_id, 1)
        self.assertEqual(records[0].msg_id, 42)
        self.assertEqual(records[0].payload, b"\x00\x01\x02\x03")

    def test_class_b_decode(self):
        """Build a proto3-encoded system.log_event payload and confirm
        the decoder yields a ClassBRecord with the fields populated.
        Registry field order is (source_module_id, event_code, severity,
        context) so field numbers are 1..4.
        """
        # source_module_id=2 (uint32 varint), event_code=42, severity=4,
        # context=b'\xca\xfe'.
        payload = (
            _proto3_field(1, 0, _proto3_varint(2))
            + _proto3_field(2, 0, _proto3_varint(42))
            + _proto3_field(3, 0, _proto3_varint(4))
            + _proto3_field(4, 2, _proto3_varint(2) + b"\xca\xfe")
        )
        rec = make_record(CLASS_B, 0, 1, t_us_publish=11, payload=payload)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            records = list(self.decoder.iter_records(io.BytesIO(rec)))
        self.assertEqual(len(records), 1)
        self.assertIsInstance(records[0], ClassBRecord)
        self.assertEqual(records[0].full_name, "system.log_event")
        self.assertEqual(records[0].t_us_publish, 11)
        self.assertEqual(records[0].fields["source_module_id"], 2)
        self.assertEqual(records[0].fields["event_code"], 42)
        self.assertEqual(records[0].fields["severity"], 4)
        self.assertEqual(records[0].fields["context"], b"\xca\xfe")

    def test_class_b_unknown_msg_id(self):
        # module_id=0 exists but msg_id=99 is not in this registry.
        rec = make_record(CLASS_B, 0, 99, t_us_publish=7, payload=b"")
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            records = list(self.decoder.iter_records(io.BytesIO(rec)))
        self.assertEqual(len(records), 1)
        self.assertIsInstance(records[0], UnknownMsgRecord)
        self.assertEqual(records[0].class_byte, CLASS_B)
        self.assertEqual(records[0].msg_id, 99)

    def test_truncated_input(self):
        # Just a length prefix and nothing else.
        records = []
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            records = list(self.decoder.iter_records(io.BytesIO(struct.pack("<H", 50))))
        self.assertEqual(len(records), 1)
        self.assertIsInstance(records[0], DecodeError)
        self.assertIn("truncated", records[0].reason)


class TestCLI(unittest.TestCase):
    def setUp(self):
        self.registry = load_registry(REGISTRY_PATH)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            self.decoder = build_decoder(self.registry)

        # Build a fixture log with two state_estimate records.
        def payload(t_us, x, y, z):
            return struct.pack(
                "<Q" + "fff" + "fff" + "ffff" + "fff" + "fff",
                t_us,
                x, y, z,            # position
                0.0, 0.0, 0.0,      # velocity
                1.0, 0.0, 0.0, 0.0, # attitude
                0.0, 0.0, 0.0,      # gyro_bias
                0.0, 0.0, 0.0,      # accel_bias
            )

        self.fixture = b"".join([
            make_record(CLASS_A, 1, 1, t_us_publish=1000, payload=payload(100, 1, 2, 3)),
            make_record(CLASS_A, 1, 1, t_us_publish=2000, payload=payload(200, 4, 5, 6)),
        ])
        self.tmpdir = tempfile.mkdtemp(prefix="msgdec_test_")
        self.log_path = os.path.join(self.tmpdir, "fixture.log")
        with open(self.log_path, "wb") as f:
            f.write(self.fixture)

    def _run_cli(self, *args, expect_rc=0):
        env = dict(os.environ)
        env["PYTHONPATH"] = DECODER_ROOT + os.pathsep + env.get("PYTHONPATH", "")
        env["PYTHONWARNINGS"] = "ignore::RuntimeWarning"
        result = subprocess.run(
            [sys.executable, "-m", "messages_decoder", self.log_path,
             "--registry", REGISTRY_PATH, *args],
            capture_output=True, env=env, text=True,
        )
        self.assertEqual(result.returncode, expect_rc,
                         msg=f"stdout={result.stdout!r}\nstderr={result.stderr!r}")
        return result

    def test_cli_jsonl(self):
        result = self._run_cli()
        lines = [ln for ln in result.stdout.splitlines() if ln.strip()]
        self.assertEqual(len(lines), 2)
        objs = [json.loads(ln) for ln in lines]
        self.assertEqual(objs[0]["__type__"], "StateEstimationStateEstimate")
        self.assertEqual(objs[0]["t_us_publish"], 1000)
        self.assertEqual(objs[0]["t_us"], 100)
        self.assertEqual(objs[0]["position"]["x"], 1.0)
        self.assertEqual(objs[1]["position"]["x"], 4.0)

    def test_cli_csv(self):
        result = self._run_cli("--format", "csv",
                               "--msg", "state_estimation.state_estimate")
        lines = [ln for ln in result.stdout.splitlines() if ln.strip()]
        self.assertEqual(len(lines), 3)  # header + 2 rows
        self.assertIn("t_us_publish", lines[0])
        self.assertIn("position.x", lines[0])
        # Second data row should have position.x = 4.0
        self.assertIn("4.0", lines[2])


class TestEncoderRoundTrip(unittest.TestCase):
    """Encode a command request, then decode it back through the same
    descriptor — the encoder + wire_pb decoder are an inverse pair."""

    def setUp(self):
        self.registry = load_registry(REGISTRY_PATH)

    def test_set_route_roundtrip(self):
        from messages_decoder.encoder import (
            CLASS_CMD_REQ,
            encode_command_request,
        )
        from messages_decoder.wire_pb import decode_message
        from messages_decoder.types import _build_pb_descriptor

        # Build a request with all four fields set.
        frame = encode_command_request(
            self.registry, "system", "set_route",
            {"target_module_id": 1, "target_msg_id": 1, "channel_id": 1, "enabled": True},
            t_us_publish=12345,
        )
        # Length prefix matches reality.
        (length,) = struct.unpack("<H", frame[:2])
        self.assertEqual(len(frame), length + 2)
        # Class byte == CMD_REQ.
        self.assertEqual(frame[2], CLASS_CMD_REQ)
        # module_id = 0 (system); cmd_id = 1 (set_route)
        self.assertEqual(frame[3], 0)
        (cmd_id,) = struct.unpack("<H", frame[4:6])
        self.assertEqual(cmd_id, 1)
        # CRC verifies.
        body = frame[2 : -2]
        (got_crc,) = struct.unpack("<H", frame[-2:])
        self.assertEqual(got_crc, crc16_ccitt_false(body))
        # Decode payload, confirm fields survive the roundtrip.
        payload = frame[2 + 12 : -2]
        descriptor = _build_pb_descriptor(
            self.registry.modules["system"]["commands"]["set_route"]["request"]
        )
        fields = decode_message(payload, descriptor)
        self.assertEqual(fields["target_module_id"], 1)
        self.assertEqual(fields["target_msg_id"], 1)
        self.assertEqual(fields["channel_id"], 1)
        self.assertEqual(fields["enabled"], True)

    def test_empty_request(self):
        from messages_decoder.encoder import encode_command_request
        # get_build_info has no request fields — should encode to empty payload.
        frame = encode_command_request(self.registry, "system", "get_build_info", {})
        (length,) = struct.unpack("<H", frame[:2])
        # Envelope (12) + crc (2) = 14, no payload.
        self.assertEqual(length, 14)


if __name__ == "__main__":
    unittest.main()
