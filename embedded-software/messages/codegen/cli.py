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
import subprocess
import sys
from pathlib import Path

from .emit_proto import emit as emit_proto
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
# Resolved relative to this file so it works regardless of cwd.
# codegen/ -> messages/ -> embedded-software/ -> repo root.
DEFAULT_NANOPB_GEN = (
    Path(__file__).resolve().parent.parent.parent.parent
    / "embedded-software/libs/rocket-protocol/nanopb/generator/nanopb_generator.py"
)


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
        "--nanopb-generator",
        type=Path,
        default=DEFAULT_NANOPB_GEN,
        help=(
            "Path to nanopb_generator.py (default: vendored copy under "
            f"{DEFAULT_NANOPB_GEN}). Used to compile messages.proto into "
            "messages.pb.{c,h}. Needs `protobuf` + `grpcio-tools` on PYTHONPATH."
        ),
    )
    parser.add_argument(
        "--no-nanopb",
        action="store_true",
        help="Skip the nanopb_generator step (still emit messages.proto). "
             "Useful when protobuf/grpcio-tools aren't installed.",
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
    files: dict[Path, str] = {
        out_dir / "registry.h":  emit_registry(registry, crc32),
        out_dir / "types.h":     emit_types(registry, crc32, resolver),
        out_dir / "publish.h":   emit_publish(registry, crc32),
        out_dir / "routing.h":   emit_routing_h(registry, crc32),
        out_dir / "routing.c":   emit_routing_c(registry, crc32, resolver),
        # .proto lives in a sibling dir so protoc/nanopb_generator can be
        # pointed at firmware/generated/proto/ without dragging in our C
        # headers.
        (out_dir.parent / "proto" / "messages.proto"): emit_proto(registry, crc32),
    }

    changed: list[str] = []
    for path, content in files.items():
        if _write_if_changed(path, content):
            changed.append(path.name)

    # nanopb generation: compile messages.proto -> messages.pb.{c,h}.
    proto_path = out_dir.parent / "proto" / "messages.proto"
    pb_h = out_dir.parent / "proto" / "messages.pb.h"
    pb_c = out_dir.parent / "proto" / "messages.pb.c"
    nanopb_changed = False
    if args.no_nanopb:
        if not args.quiet:
            print("nanopb: skipped (--no-nanopb)")
    elif not args.nanopb_generator.is_file():
        print(
            f"WARN: nanopb_generator.py not found at {args.nanopb_generator}; "
            "skipping .pb.{c,h} generation. Pass --nanopb-generator <path> or "
            "--no-nanopb to silence.",
            file=sys.stderr,
        )
    else:
        try:
            # Snapshot current .pb.{c,h} so we can decide if anything drifted.
            before = {
                p: p.read_bytes() if p.is_file() else None
                for p in (pb_h, pb_c)
            }
            subprocess.run(
                [sys.executable, str(args.nanopb_generator), "messages.proto"],
                cwd=proto_path.parent,
                check=True,
                capture_output=True,
            )
            for p in (pb_h, pb_c):
                if p.is_file() and p.read_bytes() != before[p]:
                    changed.append(p.name)
                    nanopb_changed = True
        except subprocess.CalledProcessError as exc:
            print(
                "ERROR: nanopb_generator failed.\n"
                f"  stderr: {exc.stderr.decode('utf-8', errors='replace')}",
                file=sys.stderr,
            )
            return 4

    if not args.quiet:
        if changed:
            print(f"wrote {len(changed)} file(s): {', '.join(changed)}")
        else:
            print("up to date — no files changed")
        print(f"registry CRC32: 0x{crc32:08X}")

    return 0


def _print_budget(summary: dict) -> None:
    print("bus-budget summary:")
    for chname in sorted(summary.keys(), key=lambda n: summary[n]["channel_id"]):
        info = summary[chname]
        pct = (100.0 * info["used_bps"] / info["max_bps"]) if info["max_bps"] else 0.0
        print(
            f"  {chname:<8} {info['used_bps']:>12} / {info['max_bps']:>12} bps "
            f"({pct:5.1f}%, {len(info['entries'])} class-A msg(s))"
        )
