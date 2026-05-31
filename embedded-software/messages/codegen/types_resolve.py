"""Map registry types to C types and compute packed sizes.

A user-defined type has size = sum of its field sizes (packed, no
padding). Same for Class A message payloads. Enum-refs (both
specific-error and namespace) encode as u16.
"""

from __future__ import annotations

from typing import Any

from .loader import parse_field_type

PRIM_C: dict[str, str] = {
    "u8": "uint8_t",
    "u16": "uint16_t",
    "u32": "uint32_t",
    "u64": "uint64_t",
    "i8": "int8_t",
    "i16": "int16_t",
    "i32": "int32_t",
    "i64": "int64_t",
    "f32": "float",
    "f64": "double",
    "bool": "uint8_t",  # one byte on the wire
}

PRIM_SIZE: dict[str, int] = {
    "u8": 1, "i8": 1, "bool": 1,
    "u16": 2, "i16": 2,
    "u32": 4, "i32": 4, "f32": 4,
    "u64": 8, "i64": 8, "f64": 8,
}


class TypeResolver:
    """Resolves field types into (c_type, base_size, array_len) and computes
    aggregate sizes for user types and Class A message payloads.
    """

    def __init__(self, types: dict[str, Any]):
        self.types = types
        self._size_cache: dict[str, int] = {}

    def type_size(self, type_name: str) -> int:
        if type_name in self._size_cache:
            return self._size_cache[type_name]
        tdef = self.types[type_name]
        total = 0
        for fld in tdef["fields"]:
            total += self.field_size(fld["type"])
        self._size_cache[type_name] = total
        return total

    def field_size(self, raw_type: str) -> int:
        parsed = parse_field_type(raw_type)
        arr = parsed["array_len"] or 1
        if parsed["kind"] == "prim":
            prim = parsed["prim"]
            if prim in ("string", "bytes"):
                raise ValueError(f"Class A messages cannot carry {prim!r} fields")
            return PRIM_SIZE[prim] * arr
        if parsed["kind"] == "type":
            return self.type_size(parsed["type_name"]) * arr
        if parsed["kind"] == "enum":
            return 2 * arr  # u16
        raise AssertionError(parsed)

    def message_payload_size(self, fields: list[dict[str, Any]]) -> int:
        return sum(self.field_size(fld["type"]) for fld in fields)

    def field_c_decl(self, fld: dict[str, Any]) -> str:
        """Return the C declaration for one field, e.g. 'float position[3]'."""
        parsed = parse_field_type(fld["type"])
        arr = parsed["array_len"]
        if parsed["kind"] == "prim":
            ctype = PRIM_C[parsed["prim"]]
        elif parsed["kind"] == "type":
            ctype = f"{parsed['type_name']}_t"
        elif parsed["kind"] == "enum":
            ctype = "uint16_t"
        else:
            raise AssertionError(parsed)
        if arr is not None:
            return f"{ctype} {fld['name']}[{arr}]"
        return f"{ctype} {fld['name']}"

    def user_type_order(self) -> list[str]:
        """Topologically sort user types so dependencies appear first.

        Cycles were already rejected in loader._semantic_check.
        """
        order: list[str] = []
        seen: set[str] = set()

        def visit(t: str) -> None:
            if t in seen or t not in self.types:
                return
            for fld in self.types[t]["fields"]:
                parsed = parse_field_type(fld["type"])
                if parsed["kind"] == "type":
                    visit(parsed["type_name"])
            if t not in seen:
                seen.add(t)
                order.append(t)

        for t in sorted(self.types.keys()):
            visit(t)
        return order
