"""Emit publish.h: one PUB_<MODULE>_<NAME>(...) macro per Class A message."""

from __future__ import annotations

from typing import Any

from .emit_common import banner, messages_sorted, modules_sorted


def emit(registry: dict[str, Any], crc32: int) -> str:
    out: list[str] = []
    out.append(banner(crc32))
    out.append("\n")
    out.append("#ifndef MESSAGES_GENERATED_PUBLISH_H\n")
    out.append("#define MESSAGES_GENERATED_PUBLISH_H\n")
    out.append("\n")
    out.append('#include "messages/messages.h"\n')
    out.append('#include "generated/messages/registry.h"\n')
    out.append('#include "generated/messages/types.h"\n')
    out.append("\n")

    for modname, mod in modules_sorted(registry):
        for msgname, msg in messages_sorted(mod):
            if msg["class"] != "A":
                continue
            mod_const = f"MOD_{modname.upper()}"
            msg_const = f"MSG_{modname.upper()}_{msgname.upper()}"
            struct_name = f"msg_{modname}_{msgname}_t"
            field_names = [fld["name"] for fld in msg["fields"]]
            arg_names = [f"{n}_" for n in field_names]
            args_list = ", ".join(arg_names)
            init_pairs = ", ".join(f".{n} = ({n}_)" for n in field_names)
            macro_name = f"PUB_{modname.upper()}_{msgname.upper()}"

            out.append(f"// Publish {modname}.{msgname}.\n")
            out.append(f"#define {macro_name}({args_list}) \\\n")
            out.append("    do { \\\n")
            out.append(f"        {struct_name} _m = {{ {init_pairs} }}; \\\n")
            out.append(
                f"        messages_publish_a({mod_const}, {msg_const}, &_m, sizeof(_m)); \\\n"
            )
            out.append("    } while (0)\n")
            out.append("\n")

    out.append("#endif // MESSAGES_GENERATED_PUBLISH_H\n")
    return "".join(out)
