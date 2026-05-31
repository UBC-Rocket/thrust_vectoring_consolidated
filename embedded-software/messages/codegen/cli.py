"""CLI entry point for the phase-1 registry codegen tool.

Loads + validates the registry, runs the bus-budget linter, then emits
four files into the output directory:

    registry.h   IDs, channel/module/message #defines, error enums.
    types.h      Packed C structs for user types and Class A payloads.
    publish.h    PUB_<MODULE>_<NAME>(...) macros.
    routing.{c,h}  The runtime routing table.

Class B messages, commands, and the .proto generation are out of scope
for phase 1; they are skipped with a warning.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .emit_common import messages_sorted, modules_sorted
from .emit_publish import emit as emit_publish
from .emit_registry import emit as emit_registry
from .emit_routing import emit_header as emit_routing_h
from .emit_routing import emit_source as emit_routing_c
from .emit_types import emit as emit_types
from .linter import BudgetExceeded, check_budget, compute_budget
from .loader import RegistryError, load_registry
from .types_resolve import TypeResolver

DEFAULT_REGISTRY = Path("embedded-software/messages/registry.json")
DEFAULT_SCHEMA = Path("embedded-software/messages/registry.schema.json")
DEFAULT_OUT = Path("embedded-software/firmware/generated/messages")


def _write_if_changed(path: Path, content: str) -> bool:
    """Write content to path; return True iff the file changed.

    LF line endings, UTF-8, no BOM. Deterministic.
    """
    data = content.encode("utf-8")
    if path.is_file() and path.read_bytes() == data:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="messages.codegen",
        description="Phase-1 codegen for the UBC Rocket message registry.",
    )
    parser.add_argument(
        "--registry",
        type=Path,
        default=DEFAULT_REGISTRY,
        help=f"Path to registry.json (default: {DEFAULT_REGISTRY})",
    )
    parser.add_argument(
        "--schema",
        type=Path,
        default=None,
        help="Path to registry.schema.json (default: alongside the registry).",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_OUT,
        help=f"Output directory for generated files (default: {DEFAULT_OUT})",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress informational output.",
    )
    args = parser.parse_args(argv)

    registry_path: Path = args.registry
    schema_path: Path = args.schema or registry_path.with_name("registry.schema.json")
    out_dir: Path = args.out

    try:
        registry, crc32 = load_registry(registry_path, schema_path)
    except RegistryError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    # Warn about phase-2 entries we deliberately skip.
    _warn_phase2(registry, quiet=args.quiet)

    resolver = TypeResolver(registry.get("types") or {})

    # Bus-budget linter — must pass before we emit anything.
    summary = compute_budget(registry, resolver)
    try:
        check_budget(summary)
    except BudgetExceeded as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 3

    if not args.quiet:
        _print_budget(summary)

    # Emit.
    files: dict[str, str] = {
        "registry.h": emit_registry(registry, crc32),
        "types.h": emit_types(registry, crc32, resolver),
        "publish.h": emit_publish(registry, crc32),
        "routing.h": emit_routing_h(registry, crc32),
        "routing.c": emit_routing_c(registry, crc32, resolver),
    }

    changed: list[str] = []
    for name, content in files.items():
        if _write_if_changed(out_dir / name, content):
            changed.append(name)

    if not args.quiet:
        if changed:
            print(f"wrote {len(changed)} file(s): {', '.join(changed)}")
        else:
            print("up to date — no files changed")
        print(f"registry CRC32: 0x{crc32:08X}")

    return 0


def _warn_phase2(registry: dict, *, quiet: bool) -> None:
    class_b_count = 0
    command_count = 0
    for modname, mod in modules_sorted(registry):
        for _msgname, msg in messages_sorted(mod):
            if msg["class"] == "B":
                class_b_count += 1
        for _cmdname in (mod.get("commands") or {}):
            command_count += 1
    if quiet:
        return
    if class_b_count:
        print(
            f"WARN: skipping {class_b_count} Class B message(s) — phase 2",
            file=sys.stderr,
        )
    if command_count:
        print(
            f"WARN: skipping {command_count} command(s) — phase 2",
            file=sys.stderr,
        )


def _print_budget(summary: dict) -> None:
    print("bus-budget summary:")
    for chname in sorted(summary.keys(), key=lambda n: summary[n]["channel_id"]):
        info = summary[chname]
        pct = (100.0 * info["used_bps"] / info["max_bps"]) if info["max_bps"] else 0.0
        print(
            f"  {chname:<8} {info['used_bps']:>12} / {info['max_bps']:>12} bps "
            f"({pct:5.1f}%, {len(info['entries'])} class-A msg(s))"
        )
