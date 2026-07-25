"""Load + validate the registry.

Two stages:
  1. JSON Schema validation (structural shape).
  2. Hand-rolled semantic validation (uniqueness, references, cycles)
     that JSON Schema can't express.

Raises RegistryError with a clear message on any inconsistency.
"""

from __future__ import annotations

import json
import re
import zlib
from pathlib import Path
from typing import Any


class RegistryError(Exception):
    """Raised for any registry inconsistency."""


def load_registry(registry_path: Path, schema_path: Path) -> tuple[dict[str, Any], int]:
    """Load the registry, validate it, return (registry_dict, crc32_of_bytes)."""
    if not registry_path.is_file():
        raise RegistryError(f"registry not found: {registry_path}")
    if not schema_path.is_file():
        raise RegistryError(f"schema not found: {schema_path}")

    raw = registry_path.read_bytes()
    crc32 = zlib.crc32(raw) & 0xFFFFFFFF
    try:
        registry = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise RegistryError(f"{registry_path}: invalid JSON: {exc}") from exc

    schema = json.loads(schema_path.read_bytes())

    try:
        import jsonschema
    except ImportError as exc:
        raise RegistryError(
            "jsonschema is required but not installed; install jsonschema>=4 "
            "into a venv before running codegen"
        ) from exc

    try:
        validator_cls = jsonschema.validators.validator_for(schema)
        validator_cls.check_schema(schema)
        validator = validator_cls(schema)
        errors = sorted(validator.iter_errors(registry), key=lambda e: list(e.absolute_path))
        if errors:
            messages = []
            for err in errors:
                pointer = "/".join(str(p) for p in err.absolute_path) or "<root>"
                messages.append(f"  at {pointer}: {err.message}")
            raise RegistryError(
                f"{registry_path}: JSON Schema validation failed:\n" + "\n".join(messages)
            )
    except jsonschema.SchemaError as exc:
        raise RegistryError(f"meta-schema is itself invalid: {exc}") from exc

    _semantic_check(registry, registry_path)
    return registry, crc32


# ---------------------------------------------------------------------------
# semantic checks
# ---------------------------------------------------------------------------


_FIELD_TYPE_RE = re.compile(
    r"^(?:(?P<prim>u8|u16|u32|u64|i8|i16|i32|i64|f32|f64|bool|string|bytes)"
    r"|type:(?P<typeref>[a-z_][a-z0-9_]*)"
    r"|enum:(?P<enummod>[a-z_][a-z0-9_]*)\.(?P<enumname>[a-z_][a-z0-9_]*))"
    r"(?:\[(?P<arrlen>[1-9][0-9]*)\])?$"
)


def parse_field_type(raw: str) -> dict[str, Any]:
    """Return a normalised field-type dict, or raise ValueError."""
    m = _FIELD_TYPE_RE.match(raw)
    if not m:
        raise ValueError(f"unparseable field type: {raw!r}")
    arrlen = int(m.group("arrlen")) if m.group("arrlen") else None
    if m.group("prim"):
        return {"kind": "prim", "prim": m.group("prim"), "array_len": arrlen}
    if m.group("typeref"):
        return {"kind": "type", "type_name": m.group("typeref"), "array_len": arrlen}
    return {
        "kind": "enum",
        "module": m.group("enummod"),
        "name": m.group("enumname"),
        "array_len": arrlen,
    }


def _semantic_check(registry: dict[str, Any], path: Path) -> None:
    errors: list[str] = []

    # --- channels ---
    channels = registry.get("channels", {})
    seen_channel_ids: dict[int, str] = {}
    for chname, ch in channels.items():
        cid = ch["channel_id"]
        if cid in seen_channel_ids:
            errors.append(
                f"duplicate channel_id {cid} on channels {seen_channel_ids[cid]!r} and {chname!r}"
            )
        else:
            seen_channel_ids[cid] = chname
    # routing.max_rate_hz[CH_COUNT] is indexed by channel_id, so the assigned
    # ids must densely fill [0, len(channels)). If you skip a number, the
    # generated array is undersized for the channel slot you actually use.
    n = len(channels)
    if seen_channel_ids and set(seen_channel_ids.keys()) != set(range(n)):
        errors.append(
            f"channel_id values must be contiguous from 0..{n - 1} "
            f"(got {sorted(seen_channel_ids.keys())})"
        )

    # --- modules ---
    modules = registry.get("modules", {})
    seen_module_ids: dict[int, str] = {}
    for modname, mod in modules.items():
        mid = mod["module_id"]
        if mid in seen_module_ids:
            errors.append(
                f"duplicate module_id {mid} on modules {seen_module_ids[mid]!r} and {modname!r}"
            )
        else:
            seen_module_ids[mid] = modname

        # duplicate msg_id within module
        seen_msg_ids: dict[int, str] = {}
        for msgname, msg in (mod.get("messages") or {}).items():
            msg_id = msg["msg_id"]
            if msg_id in seen_msg_ids:
                errors.append(
                    f"module {modname!r}: duplicate msg_id {msg_id} on "
                    f"messages {seen_msg_ids[msg_id]!r} and {msgname!r}"
                )
            else:
                seen_msg_ids[msg_id] = msgname

        # duplicate cmd_id within module
        seen_cmd_ids: dict[int, str] = {}
        for cmdname, cmd in (mod.get("commands") or {}).items():
            cmd_id = cmd["cmd_id"]
            if cmd_id in seen_cmd_ids:
                errors.append(
                    f"module {modname!r}: duplicate cmd_id {cmd_id} on "
                    f"commands {seen_cmd_ids[cmd_id]!r} and {cmdname!r}"
                )
            else:
                seen_cmd_ids[cmd_id] = cmdname

        # duplicate error code within module
        seen_err_codes: dict[int, str] = {}
        for errname, err in (mod.get("errors") or {}).items():
            code = err["code"]
            if code in seen_err_codes:
                errors.append(
                    f"module {modname!r}: duplicate error code {code} on "
                    f"errors {seen_err_codes[code]!r} and {errname!r}"
                )
            else:
                seen_err_codes[code] = errname

    # --- field type references / routing references ---
    types = registry.get("types", {})
    for tname, tdef in types.items():
        for fld in tdef["fields"]:
            try:
                parsed = parse_field_type(fld["type"])
            except ValueError as exc:
                errors.append(f"type {tname!r} field {fld['name']!r}: {exc}")
                continue
            _check_field_ref(parsed, types, modules, errors, f"type {tname!r} field {fld['name']!r}")

    for modname, mod in modules.items():
        for msgname, msg in (mod.get("messages") or {}).items():
            for fld in msg["fields"]:
                try:
                    parsed = parse_field_type(fld["type"])
                except ValueError as exc:
                    errors.append(
                        f"module {modname!r} message {msgname!r} field {fld['name']!r}: {exc}"
                    )
                    continue
                _check_field_ref(
                    parsed,
                    types,
                    modules,
                    errors,
                    f"module {modname!r} message {msgname!r} field {fld['name']!r}",
                )

            for chname in (msg.get("routing") or {}):
                if chname not in channels:
                    errors.append(
                        f"module {modname!r} message {msgname!r}: routing references "
                        f"unknown channel {chname!r}"
                    )

    # --- type cycle detection ---
    cycle = _detect_type_cycle(types)
    if cycle is not None:
        errors.append("type cycle detected: " + " -> ".join(cycle))

    if errors:
        raise RegistryError(
            f"{path}: semantic validation failed:\n" + "\n".join(f"  - {e}" for e in errors)
        )


def _check_field_ref(
    parsed: dict[str, Any],
    types: dict[str, Any],
    modules: dict[str, Any],
    errors: list[str],
    where: str,
) -> None:
    if parsed["kind"] == "type":
        if parsed["type_name"] not in types:
            errors.append(f"{where}: references unknown type {parsed['type_name']!r}")
    elif parsed["kind"] == "enum":
        modname = parsed["module"]
        name = parsed["name"]
        mod = modules.get(modname)
        if mod is None:
            errors.append(f"{where}: enum references unknown module {modname!r}")
            return
        errs = mod.get("errors") or {}
        # 'errors' (the literal word) means "any code in this module's errors namespace"
        if name == "errors":
            return
        if name not in errs:
            errors.append(
                f"{where}: enum {modname}.{name} not found (must be 'errors' or an error name)"
            )


def _detect_type_cycle(types: dict[str, Any]) -> list[str] | None:
    """Return a cycle path if any user type cycles via type:<name> references."""
    WHITE, GREY, BLACK = 0, 1, 2
    colour = {t: WHITE for t in types}
    parent: dict[str, str | None] = {t: None for t in types}

    def deps(tname: str) -> list[str]:
        out: list[str] = []
        for fld in types[tname]["fields"]:
            try:
                parsed = parse_field_type(fld["type"])
            except ValueError:
                continue
            if parsed["kind"] == "type":
                out.append(parsed["type_name"])
        return out

    def visit(start: str) -> list[str] | None:
        stack: list[tuple[str, list[str]]] = [(start, deps(start))]
        colour[start] = GREY
        while stack:
            node, remaining = stack[-1]
            if not remaining:
                colour[node] = BLACK
                stack.pop()
                continue
            nxt = remaining.pop()
            if nxt not in types:
                continue
            if colour.get(nxt) == GREY:
                # cycle: build path from nxt back through stack
                path = [nxt]
                for n, _ in stack:
                    path.append(n)
                    if n == nxt:
                        break
                # rotate so cycle starts and ends at the same node
                # stack is in DFS order; nxt closes the loop
                cycle_start = next(i for i, (n, _) in enumerate(stack) if n == nxt)
                return [n for n, _ in stack[cycle_start:]] + [nxt]
            if colour.get(nxt) == WHITE:
                parent[nxt] = node
                colour[nxt] = GREY
                stack.append((nxt, deps(nxt)))
        return None

    for t in types:
        if colour[t] == WHITE:
            found = visit(t)
            if found is not None:
                return found
    return None
