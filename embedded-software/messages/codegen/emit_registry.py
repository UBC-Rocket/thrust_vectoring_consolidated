"""Emit registry.h: IDs and constants. No type definitions here."""

from __future__ import annotations

from typing import Any

from .emit_common import (
    banner,
    channels_sorted,
    errors_sorted,
    messages_sorted,
    modules_sorted,
)


def emit(registry: dict[str, Any], crc32: int) -> str:
    out: list[str] = []
    out.append(banner(crc32))
    out.append("\n")
    out.append("#ifndef MESSAGES_GENERATED_REGISTRY_H\n")
    out.append("#define MESSAGES_GENERATED_REGISTRY_H\n")
    out.append("\n")
    out.append("#include <stdint.h>\n")
    out.append("\n")

    # registry version + crc
    out.append(f"#define REGISTRY_VERSION {int(registry['version'])}\n")
    out.append(f"#define REGISTRY_CRC32 0x{crc32:08X}u\n")
    out.append("\n")

    # message class constants
    out.append("// Message class identifiers (also the on-wire 'class' byte).\n")
    out.append("#define MSG_CLASS_A 0x41\n")
    out.append("#define MSG_CLASS_B 0x42\n")
    out.append("\n")

    # channels
    out.append("// Channels (one #define per channel; CH_COUNT is the total).\n")
    channels = channels_sorted(registry)
    for chname, ch in channels:
        out.append(f"#define CH_{chname.upper()} {ch['channel_id']}\n")
    out.append(f"#define CH_COUNT {len(channels)}\n")
    out.append("\n")

    # modules
    out.append("// Modules.\n")
    mods = modules_sorted(registry)
    for modname, mod in mods:
        out.append(f"#define MOD_{modname.upper()} {mod['module_id']}\n")
    out.append("\n")

    # messages
    out.append("// Messages, grouped by module.\n")
    for modname, mod in mods:
        msgs = messages_sorted(mod)
        if not msgs:
            continue
        out.append(f"// --- module: {modname} ---\n")
        for msgname, msg in msgs:
            out.append(
                f"#define MSG_{modname.upper()}_{msgname.upper()} {msg['msg_id']}\n"
            )
    out.append("\n")

    # errors as enums
    out.append("// Per-module error namespaces.\n")
    any_errs = False
    for modname, mod in mods:
        errs = errors_sorted(mod)
        if not errs:
            continue
        any_errs = True
        out.append(f"typedef enum {{\n")
        for ename, edef in errs:
            out.append(
                f"    ERR_{modname.upper()}_{ename.upper()} = {edef['code']},\n"
            )
        out.append(f"}} {modname}_err_t;\n\n")

    if not any_errs:
        out.append("// (no error namespaces declared)\n\n")

    out.append("#endif // MESSAGES_GENERATED_REGISTRY_H\n")
    return "".join(out)
