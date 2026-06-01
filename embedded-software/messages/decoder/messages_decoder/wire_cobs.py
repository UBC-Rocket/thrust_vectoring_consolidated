"""Pure-Python COBS encode/decode, mirror of firmware/comms/cobs.

Used by the messages-console CLI when speaking to firmware over a
channel whose registry framing is "cobs" (currently: vcp).

Reference: Cheshire & Baker 1999. Algorithm matches the firmware C
implementation byte-for-byte — see tests in the decoder package.
"""
from __future__ import annotations


def cobs_encode(src: bytes) -> bytes:
    """Encode bytes for COBS framing. Output contains no 0x00 bytes.

    Caller appends the 0x00 delimiter after the returned bytes for
    on-the-wire transmission.
    """
    if not src:
        return b"\x01"
    out = bytearray()
    code_idx = 0
    out.append(1)  # placeholder for first code byte
    code = 1
    for b in src:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(1)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(1)
                code = 1
    out[code_idx] = code
    return bytes(out)


def cobs_decode(src: bytes) -> bytes:
    """Decode one COBS frame (without the trailing 0x00 delimiter).

    Raises ValueError on malformed input.
    """
    if not src:
        return b""
    out = bytearray()
    i = 0
    n = len(src)
    while i < n:
        code = src[i]
        i += 1
        if code == 0 or i + (code - 1) > n:
            raise ValueError("malformed COBS frame")
        for _ in range(code - 1):
            out.append(src[i])
            i += 1
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)
