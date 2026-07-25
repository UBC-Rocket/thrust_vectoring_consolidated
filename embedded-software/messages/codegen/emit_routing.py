"""Emit routing.h + routing.c."""

from __future__ import annotations

from typing import Any

from .emit_common import banner, channels_sorted, messages_sorted, modules_sorted
from .types_resolve import TypeResolver


def emit_header(registry: dict[str, Any], crc32: int) -> str:
    out: list[str] = []
    out.append(banner(crc32))
    out.append("\n")
    out.append("#ifndef MESSAGES_GENERATED_ROUTING_H\n")
    out.append("#define MESSAGES_GENERATED_ROUTING_H\n")
    out.append("\n")
    out.append("#include <stddef.h>\n")
    out.append("#include <stdint.h>\n")
    out.append('#include "generated/messages/registry.h"\n')
    out.append("\n")
    out.append("typedef struct {\n")
    out.append("    uint8_t  module_id;\n")
    out.append("    uint16_t msg_id;\n")
    out.append("    uint8_t  class;            // MSG_CLASS_A or MSG_CLASS_B\n")
    out.append("    uint16_t payload_size;     // Class A: fixed size; Class B: 0 (variable)\n")
    out.append("    uint8_t  enabled_channels; // bitmask: bit channel_id set if enabled at boot\n")
    out.append("    uint16_t max_rate_hz[CH_COUNT]; // per-channel rate cap; 0 = unlimited\n")
    out.append("    // Class B: pointer to nanopb pb_msgdesc_t (declared as `const void *`\n")
    out.append("    // so consumers that don't link nanopb still see a complete type).\n")
    out.append("    // Class A: NULL.\n")
    out.append("    const void *pb_desc;\n")
    out.append("} messages_routing_entry_t;\n")
    out.append("\n")
    out.append("extern const messages_routing_entry_t messages_routing_table[];\n")
    out.append("extern const size_t messages_routing_table_len;\n")
    out.append("\n")
    out.append("#endif // MESSAGES_GENERATED_ROUTING_H\n")
    return "".join(out)


def emit_source(registry: dict[str, Any], crc32: int, resolver: TypeResolver) -> str:
    out: list[str] = []
    out.append(banner(crc32))
    out.append("\n")
    out.append('#include "generated/messages/routing.h"\n')
    out.append("\n")
    # Pull in nanopb message descriptors only when nanopb is on the build,
    # so host-side tests + non-firmware builds don't need the .pb.h header.
    out.append("#if defined(MESSAGES_HAVE_NANOPB)\n")
    out.append('  #include "generated/proto/messages.pb.h"\n')
    out.append("  #define PB_DESC(name) (&(name))\n")
    out.append("#else\n")
    out.append("  #define PB_DESC(name) ((const void *)0)\n")
    out.append("#endif\n")
    out.append("\n")
    out.append("const messages_routing_entry_t messages_routing_table[] = {\n")

    channels = channels_sorted(registry)
    # channel_ids are validated to be a dense 0..N-1 range, so slot index
    # matches channel_id directly.
    channel_id_to_name: dict[int, str] = {ch["channel_id"]: name for name, ch in channels}
    n_channels = len(channels)

    skipped = 0
    for modname, mod in modules_sorted(registry):
        for msgname, msg in messages_sorted(mod):
            cls = msg["class"]
            mod_const = f"MOD_{modname.upper()}"
            msg_const = f"MSG_{modname.upper()}_{msgname.upper()}"
            cls_const = "MSG_CLASS_A" if cls == "A" else "MSG_CLASS_B"

            if cls == "A":
                payload_size = resolver.message_payload_size(msg["fields"])
            else:
                # Class B is variable-length; runtime fills the wire length.
                payload_size = 0
                skipped += 1

            routing = msg.get("routing") or {}
            enabled_mask = 0
            rates: list[int] = []
            for slot in range(n_channels):
                chname = channel_id_to_name[slot]
                rate = 0
                if chname in routing:
                    route = routing[chname]
                    if route.get("enabled"):
                        enabled_mask |= 1 << slot
                    raw_rate = route.get("max_rate_hz", 0) or 0
                    # max_rate_hz field is u16; clamp at u16 max so a bad
                    # registry can't silently produce a truncated array entry.
                    rate = min(int(raw_rate), 0xFFFF)
                rates.append(rate)

            rates_init = ", ".join(str(r) for r in rates) if rates else "0"
            # pb_desc: Class B references the nanopb-generated descriptor;
            # naming convention from emit_proto.py is messages_<PascalMod><PascalMsg>_msg.
            if cls == "B":
                pb_name = (
                    "messages_"
                    + "".join(p.capitalize() for p in modname.split("_"))
                    + "".join(p.capitalize() for p in msgname.split("_"))
                    + "_msg"
                )
                pb_desc = f"PB_DESC({pb_name})"
            else:
                pb_desc = "NULL"
            out.append(
                f"    {{ .module_id = {mod_const}, .msg_id = {msg_const}, "
                f".class = {cls_const}, .payload_size = {payload_size}, "
                f".enabled_channels = 0x{enabled_mask:02X}, "
                f".max_rate_hz = {{ {rates_init} }}, "
                f".pb_desc = {pb_desc} }},\n"
            )

    out.append("};\n")
    out.append("\n")
    out.append(
        f"const size_t messages_routing_table_len = "
        f"sizeof(messages_routing_table) / sizeof(messages_routing_table[0]);\n"
    )
    if skipped:
        out.append(
            f"\n// Note: {skipped} Class B entry/entries above have payload_size=0; "
            "wire length is filled at runtime by nanopb encode (phase 2).\n"
        )
    return "".join(out)
