"""Registry loader + optional jsonschema validation + CRC32.

The registry JSON file is the single source of truth. We deliberately
read it at runtime rather than codegen'ing Python from it.

If the ``jsonschema`` package is available the registry is validated
against its sibling ``registry.schema.json``. If it is not, we proceed
without validation (the schema is enforced upstream by codegen anyway —
this is a developer convenience, not a correctness guarantee).
"""

from __future__ import annotations

import binascii
import json
import os
from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class Registry:
    """In-memory registry view.

    ``raw`` is the parsed JSON document (untouched). ``crc32`` is the
    CRC32 of the registry file's canonical-encoded bytes (utf-8 of the
    JSON re-serialised with sorted keys and no extra whitespace) so it
    is reproducible across formatting changes that don't affect the data.
    """

    raw: Dict[str, Any]
    crc32: int
    source_path: Optional[str] = None

    @property
    def modules(self) -> Dict[str, Any]:
        return self.raw.get("modules", {})

    @property
    def types(self) -> Dict[str, Any]:
        return self.raw.get("types", {})


def registry_crc32(raw: Dict[str, Any]) -> int:
    """Compute a reproducible CRC32 over a registry document.

    Re-serialises with ``sort_keys=True`` and compact separators so the
    same registry data yields the same CRC regardless of whitespace.
    """

    canonical = json.dumps(raw, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return binascii.crc32(canonical) & 0xFFFFFFFF


def _validate(raw: Dict[str, Any], schema_path: str) -> None:
    try:
        import jsonschema  # type: ignore[import-not-found]
    except Exception:
        return
    with open(schema_path, "r", encoding="utf-8") as f:
        schema = json.load(f)
    jsonschema.validate(raw, schema)


def load_registry(path: str) -> Registry:
    """Load + (optionally) validate ``registry.json``.

    Looks for a sibling ``registry.schema.json`` in the same directory
    and validates against it if ``jsonschema`` is importable.
    """

    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)

    schema_path = os.path.join(os.path.dirname(os.path.abspath(path)), "registry.schema.json")
    if os.path.exists(schema_path):
        _validate(raw, schema_path)

    return Registry(raw=raw, crc32=registry_crc32(raw), source_path=os.path.abspath(path))
