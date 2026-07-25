"""Emit messages.proto: proto3 schema for all Class B messages, commands,
and user types they reference.

Class A doesn't go through proto — it's pack/unpack of fixed C structs.
Only Class B + commands need a proto schema (commands' request/response
are always Class B).

Field numbers are derived from the registry's field declaration order
(starting at 1). The registry is the source of truth: any reorder will
break wire compat, which matches our policy of "only append".
"""

from __future__ import annotations

from typing import Any

from .emit_common import banner, messages_sorted, modules_sorted
from .loader import parse_field_type


PRIM_PROTO: dict[str, str] = {
    "u8": "uint32",
    "u16": "uint32",
    "u32": "uint32",
    "u64": "uint64",
    "i8": "int32",
    "i16": "int32",
    "i32": "int32",
    "i64": "int64",
    "f32": "float",
    "f64": "double",
    "bool": "bool",
    "string": "string",
    "bytes": "bytes",
}


def _pascal(snake: str) -> str:
    return "".join(p.capitalize() for p in snake.split("_"))


def _field_proto_type(raw_type: str, *, types_used: set[str] | None = None) -> str:
    """Map a registry field type string to a proto3 type token.

    Records any user-type references into types_used so we know which
    user types we actually need to emit in the proto.
    """
    parsed = parse_field_type(raw_type)
    # Arrays become repeated (we add the 'repeated' qualifier at the
    # field level; here we return the base type only).
    if parsed["kind"] == "prim":
        return PRIM_PROTO[parsed["prim"]]
    if parsed["kind"] == "type":
        name = parsed["type_name"]
        if types_used is not None:
            types_used.add(name)
        return _pascal(name)
    if parsed["kind"] == "enum":
        # We don't emit proto enums — names live in the registry, only the
        # numeric code goes on the wire. uint32 is cheap (1-byte varint
        # for codes <= 127, which covers our entire foreseeable range).
        return "uint32"
    raise AssertionError(parsed)


def _emit_message(
    name: str,
    description: str,
    fields: list[dict[str, Any]],
    *,
    types_used: set[str],
) -> list[str]:
    out: list[str] = []
    out.append(f"// {description}\n")
    out.append(f"message {name} {{\n")
    for i, fld in enumerate(fields, start=1):
        parsed = parse_field_type(fld["type"])
        proto_type = _field_proto_type(fld["type"], types_used=types_used)
        qualifier = "repeated " if parsed["array_len"] is not None else ""
        comment = fld["description"]
        if fld.get("units"):
            comment = f"{comment} [{fld['units']}]"
        out.append(f"    // {comment}\n")
        out.append(f"    {qualifier}{proto_type} {fld['name']} = {i};\n")
    out.append("}\n\n")
    return out


def emit(registry: dict[str, Any], crc32: int) -> str:
    out: list[str] = []
    out.append(banner(crc32).replace("//", "//"))
    out.append("\n")
    out.append('syntax = "proto3";\n')
    out.append("\n")
    out.append("package messages;\n")
    out.append("\n")

    types_used: set[str] = set()
    body: list[str] = []

    # Class B messages.
    body.append("// === Class B messages ===\n\n")
    class_b_count = 0
    for modname, mod in modules_sorted(registry):
        for msgname, msg in messages_sorted(mod):
            if msg["class"] != "B":
                continue
            class_b_count += 1
            proto_name = _pascal(modname) + _pascal(msgname)
            body.extend(
                _emit_message(
                    proto_name,
                    f"{modname}.{msgname}: {msg['description']}",
                    msg["fields"],
                    types_used=types_used,
                )
            )
    if class_b_count == 0:
        body.append("// (no Class B messages in this registry)\n\n")

    # Commands: request + response messages.
    body.append("// === Commands (request + response) ===\n\n")
    command_count = 0
    for modname, mod in modules_sorted(registry):
        for cmdname, cmd in sorted((mod.get("commands") or {}).items(),
                                    key=lambda kv: kv[1]["cmd_id"]):
            command_count += 1
            base = "Cmd" + _pascal(modname) + _pascal(cmdname)
            # request
            if cmd["request"]:
                body.extend(
                    _emit_message(
                        base + "Request",
                        f"{modname}.{cmdname} request",
                        cmd["request"],
                        types_used=types_used,
                    )
                )
            else:
                body.append(f"// {modname}.{cmdname} request (empty)\n")
                body.append(f"message {base}Request {{}}\n\n")
            # response
            body.extend(
                _emit_message(
                    base + "Response",
                    f"{modname}.{cmdname} response"
                    + (" (streaming)" if cmd["response"]["streaming"] else ""),
                    cmd["response"]["fields"],
                    types_used=types_used,
                )
            )
    if command_count == 0:
        body.append("// (no commands in this registry)\n\n")

    # User types — emit before messages that reference them. proto3 doesn't
    # require forward-declarations, but ordering top-down reads better.
    # Emit only types actually referenced (avoids dead messages in the .proto).
    types_section: list[str] = []
    if types_used:
        types_section.append("// === User-defined types ===\n\n")
        # transitive closure: a user type can reference another user type
        from .types_resolve import TypeResolver  # local import to avoid cycle
        resolver = TypeResolver(registry.get("types") or {})
        wanted = set(types_used)
        added = True
        while added:
            added = False
            for tname in list(wanted):
                tdef = registry["types"][tname]
                for fld in tdef["fields"]:
                    p = parse_field_type(fld["type"])
                    if p["kind"] == "type" and p["type_name"] not in wanted:
                        wanted.add(p["type_name"])
                        added = True
        # emit in dependency order
        for tname in resolver.user_type_order():
            if tname not in wanted:
                continue
            tdef = registry["types"][tname]
            types_section.extend(
                _emit_message(
                    _pascal(tname),
                    tdef["description"],
                    tdef["fields"],
                    types_used=set(),  # discard further tracking
                )
            )

    out.extend(types_section)
    out.extend(body)
    return "".join(out)
