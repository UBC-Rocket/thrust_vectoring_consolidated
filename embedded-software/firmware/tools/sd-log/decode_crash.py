#!/usr/bin/env python3
"""
Decode a crash dump from the SD card's reserved region.

The crash dump occupies the last 16 blocks (8 KB) of the 64 MB erase region,
starting at block 131056 (byte offset 131056 * 512 = 67100672).

Usage:
    sudo python3 decode_crash.py /dev/rdisk4
    python3 decode_crash.py sd-log/bin/log_dump.bin
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

CRASH_SD_BLOCK_START = 131072 - 16  # 131056
CRASH_SD_BLOCK_COUNT = 16
CRASH_BYTE_OFFSET = CRASH_SD_BLOCK_START * 512
CRASH_BYTE_SIZE = CRASH_SD_BLOCK_COUNT * 512  # 8192 bytes

CRASH_DUMP_MAGIC = 0xDEADFA17
CRASH_STACK_DUMP_WORDS = 32

FAULT_TYPES = {
    0: "HardFault",
    1: "MemManage",
    2: "BusFault",
    3: "UsageFault",
}

TASK_STATES = {
    0: "Running",
    1: "Ready",
    2: "Blocked",
    3: "Suspended",
    4: "Deleted",
}

# crash_dump_header_t: packed
HEADER_FORMAT = "<"  # little-endian
HEADER_FORMAT += "I"   # magic
HEADER_FORMAT += "I"   # version
HEADER_FORMAT += "I"   # timestamp_us
HEADER_FORMAT += "I"   # total_size
HEADER_FORMAT += "I"   # cfsr
HEADER_FORMAT += "I"   # hfsr
HEADER_FORMAT += "I"   # bfar
HEADER_FORMAT += "I"   # mmfar
HEADER_FORMAT += "I"   # shcsr
HEADER_FORMAT += "I"   # exc_return
HEADER_FORMAT += "I"   # fault_msp
HEADER_FORMAT += "I"   # fault_psp
HEADER_FORMAT += "I"   # r0
HEADER_FORMAT += "I"   # r1
HEADER_FORMAT += "I"   # r2
HEADER_FORMAT += "I"   # r3
HEADER_FORMAT += "I"   # r12
HEADER_FORMAT += "I"   # lr_from_frame
HEADER_FORMAT += "I"   # pc
HEADER_FORMAT += "I"   # xpsr
HEADER_FORMAT += "16s" # active_task_name
HEADER_FORMAT += "I"   # active_task_number
HEADER_FORMAT += "B"   # task_count
HEADER_FORMAT += "B"   # fault_type
HEADER_FORMAT += "2s"  # reserved

HEADER_STRUCT = struct.Struct(HEADER_FORMAT)

# crash_dump_task_entry_t: packed
TASK_FORMAT = "<"
TASK_FORMAT += "16s"  # name
TASK_FORMAT += "I"    # priority
TASK_FORMAT += "I"    # stack_pointer
TASK_FORMAT += "I"    # stack_base
TASK_FORMAT += "I"    # task_number
TASK_FORMAT += "B"    # state
TASK_FORMAT += "3s"   # reserved
TASK_FORMAT += f"{CRASH_STACK_DUMP_WORDS}I"  # stack_words[32]

TASK_STRUCT = struct.Struct(TASK_FORMAT)


def decode_cfsr(cfsr: int) -> list[str]:
    """Decode Configurable Fault Status Register bits."""
    flags = []
    # MemManage (bits 0-7)
    if cfsr & (1 << 0): flags.append("IACCVIOL (instruction access violation)")
    if cfsr & (1 << 1): flags.append("DACCVIOL (data access violation)")
    if cfsr & (1 << 3): flags.append("MUNSTKERR (MemManage unstacking error)")
    if cfsr & (1 << 4): flags.append("MSTKERR (MemManage stacking error)")
    if cfsr & (1 << 5): flags.append("MLSPERR (MemManage FP lazy stacking)")
    if cfsr & (1 << 7): flags.append("MMARVALID (MMFAR valid)")
    # BusFault (bits 8-15)
    if cfsr & (1 << 8): flags.append("IBUSERR (instruction bus error)")
    if cfsr & (1 << 9): flags.append("PRECISERR (precise data bus error)")
    if cfsr & (1 << 10): flags.append("IMPRECISERR (imprecise data bus error)")
    if cfsr & (1 << 11): flags.append("UNSTKERR (BusFault unstacking error)")
    if cfsr & (1 << 12): flags.append("STKERR (BusFault stacking error)")
    if cfsr & (1 << 13): flags.append("LSPERR (BusFault FP lazy stacking)")
    if cfsr & (1 << 15): flags.append("BFARVALID (BFAR valid)")
    # UsageFault (bits 16-31)
    if cfsr & (1 << 16): flags.append("UNDEFINSTR (undefined instruction)")
    if cfsr & (1 << 17): flags.append("INVSTATE (invalid state)")
    if cfsr & (1 << 18): flags.append("INVPC (invalid PC)")
    if cfsr & (1 << 19): flags.append("NOCP (no coprocessor)")
    if cfsr & (1 << 20): flags.append("STKOF (stack overflow)")
    if cfsr & (1 << 24): flags.append("UNALIGNED (unaligned access)")
    if cfsr & (1 << 25): flags.append("DIVBYZERO (divide by zero)")
    return flags


def decode_name(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")


def main():
    parser = argparse.ArgumentParser(description="Decode crash dump from SD card")
    parser.add_argument("input", type=Path, help="SD card device or binary dump file")
    args = parser.parse_args()

    with args.input.open("rb") as f:
        f.seek(CRASH_BYTE_OFFSET)
        data = f.read(CRASH_BYTE_SIZE)

    if len(data) < HEADER_STRUCT.size:
        print("Not enough data read from crash region.", file=sys.stderr)
        sys.exit(1)

    # Check magic
    magic = struct.unpack_from("<I", data, 0)[0]
    if magic != CRASH_DUMP_MAGIC:
        if magic == 0xFFFFFFFF:
            print("No crash dump found (region is erased / 0xFF).")
        else:
            print(f"Invalid crash dump magic: 0x{magic:08X} (expected 0x{CRASH_DUMP_MAGIC:08X})")
        sys.exit(0)

    # Decode header
    hdr = HEADER_STRUCT.unpack_from(data, 0)
    (magic, version, timestamp_us, total_size,
     cfsr, hfsr, bfar, mmfar, shcsr, exc_return, fault_msp, fault_psp,
     r0, r1, r2, r3, r12, lr, pc, xpsr,
     active_task_raw, active_task_number,
     task_count, fault_type, _reserved) = hdr

    print("=" * 60)
    print("  CRASH DUMP")
    print("=" * 60)
    print(f"  Magic:        0x{magic:08X}")
    print(f"  Version:      {version}")
    print(f"  Timestamp:    {timestamp_us} us ({timestamp_us/1e6:.3f} s)")
    print(f"  Fault type:   {FAULT_TYPES.get(fault_type, f'Unknown ({fault_type})')}")
    print(f"  Active task:  {decode_name(active_task_raw)} (#{active_task_number})")
    print()

    print("── Fault Registers ──")
    print(f"  CFSR:  0x{cfsr:08X}")
    for flag in decode_cfsr(cfsr):
        print(f"         → {flag}")
    print(f"  HFSR:  0x{hfsr:08X}")
    if hfsr & (1 << 30):
        print("         → FORCED (fault escalated to HardFault)")
    print(f"  BFAR:  0x{bfar:08X}")
    print(f"  MMFAR: 0x{mmfar:08X}")
    print(f"  SHCSR: 0x{shcsr:08X}")
    print()

    print("── Exception Stack Frame ──")
    print(f"  PC:    0x{pc:08X}  ← faulting instruction")
    print(f"  LR:    0x{lr:08X}  ← return address")
    print(f"  R0:    0x{r0:08X}    R1:  0x{r1:08X}")
    print(f"  R2:    0x{r2:08X}    R3:  0x{r3:08X}")
    print(f"  R12:   0x{r12:08X}")
    print(f"  xPSR:  0x{xpsr:08X}")
    print(f"  EXC_RETURN: 0x{exc_return:08X}")
    print(f"  MSP:   0x{fault_msp:08X}    PSP: 0x{fault_psp:08X}")
    print()

    print(f"── Tasks ({task_count} registered) ──")
    offset = HEADER_STRUCT.size
    for i in range(min(task_count, 8)):
        if offset + TASK_STRUCT.size > len(data):
            print(f"  [truncated at task {i}]")
            break

        entry = TASK_STRUCT.unpack_from(data, offset)
        name = decode_name(entry[0])
        priority = entry[1]
        sp = entry[2]
        stack_base = entry[3]
        task_num = entry[4]
        state = entry[5]
        stack_words = entry[7:]  # 32 words

        state_name = TASK_STATES.get(state, f"Unknown({state})")
        print(f"  [{i}] {name:<16s} prio={priority} state={state_name} "
              f"task#={task_num} SP=0x{sp:08X} base=0x{stack_base:08X}")

        # Show top of stack (first 8 words)
        print(f"      Stack: ", end="")
        for w in range(min(8, len(stack_words))):
            print(f"{stack_words[w]:08X} ", end="")
        print()

        offset += TASK_STRUCT.size

    print()
    print("── To look up PC/LR addresses ──")
    print(f"  arm-none-eabi-addr2line -e build/debug/ulysses.elf -f 0x{pc:08X}")
    print(f"  arm-none-eabi-addr2line -e build/debug/ulysses.elf -f 0x{lr:08X}")


if __name__ == "__main__":
    main()
