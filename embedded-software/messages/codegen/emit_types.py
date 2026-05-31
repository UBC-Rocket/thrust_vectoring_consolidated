"""Emit types.h: packed structs for user types and Class A messages."""

from __future__ import annotations

from typing import Any

from .emit_common import banner, messages_sorted, modules_sorted
from .types_resolve import TypeResolver


def emit(registry: dict[str, Any], crc32: int, resolver: TypeResolver) -> str:
    out: list[str] = []
    out.append(banner(crc32))
    out.append("\n")
    out.append("#ifndef MESSAGES_GENERATED_TYPES_H\n")
    out.append("#define MESSAGES_GENERATED_TYPES_H\n")
    out.append("\n")
    out.append("#include <stdint.h>\n")
    out.append("\n")

    # user-defined types in dependency order
    out.append("// User-defined types.\n")
    for tname in resolver.user_type_order():
        tdef = registry["types"][tname]
        size = resolver.type_size(tname)
        out.append(f"// {tdef['description']}\n")
        out.append("typedef struct __attribute__((packed)) {\n")
        for fld in tdef["fields"]:
            out.append(f"    {resolver.field_c_decl(fld)};\n")
        out.append(f"}} {tname}_t;\n")
        out.append(
            f'_Static_assert(sizeof({tname}_t) == {size}, '
            f'"{tname}_t packed size mismatch");\n\n'
        )

    # Class A message payloads
    out.append("// Class A message payload structs.\n")
    skipped_b = 0
    for modname, mod in modules_sorted(registry):
        for msgname, msg in messages_sorted(mod):
            if msg["class"] != "A":
                skipped_b += 1
                continue
            size = resolver.message_payload_size(msg["fields"])
            struct_name = f"msg_{modname}_{msgname}_t"
            out.append(f"// {modname}.{msgname}: {msg['description']}\n")
            out.append("typedef struct __attribute__((packed)) {\n")
            for fld in msg["fields"]:
                out.append(f"    {resolver.field_c_decl(fld)};\n")
            out.append(f"}} {struct_name};\n")
            out.append(
                f'_Static_assert(sizeof({struct_name}) == {size}, '
                f'"{struct_name} packed size mismatch");\n\n'
            )

    if skipped_b:
        out.append(
            f"// (skipped {skipped_b} Class B message(s) — phase 2)\n\n"
        )

    out.append("#endif // MESSAGES_GENERATED_TYPES_H\n")
    return "".join(out)
