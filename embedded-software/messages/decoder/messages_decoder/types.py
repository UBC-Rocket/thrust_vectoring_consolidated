"""Build dataclass types + unpackers from the registry at runtime.

This is the Python-side equivalent of what C codegen does: walk the
registry, produce one ``@dataclass`` per Class A message and per user
composite type, and pre-compute a ``struct`` format string for fast
unpacking of the payload.

Enum-typed fields decode to a dynamically-built ``IntEnum`` subclass per
``<module>.errors`` namespace. Unknown codes still decode (via
``IntEnum``'s permissive construction) — the consumer can check
``isinstance(value.name, str)`` (always true), but for unknown codes
``.name`` will return the synthetic ``"unknown_<code>"`` name we register.
We picked an IntEnum so:

  * ``int(record.error_code)`` Just Works for arithmetic / serialisation.
  * ``record.error_code.name`` gives the symbolic string.
  * ``record.error_code == SystemErrors.routes_locked`` works.
  * JSON serialisation falls back to the int via a custom encoder helper.

This is more ergonomic than carrying two parallel fields per enum.
"""

from __future__ import annotations

import logging
import re
import struct
import warnings
from dataclasses import dataclass, make_dataclass
from enum import IntEnum
from typing import Any, BinaryIO, Callable, Dict, Iterator, List, Optional, Tuple

from .registry import Registry
from .wire import (
    CLASS_A,
    CLASS_B,
    CRC_SIZE,
    ENVELOPE_SIZE,
    LENGTH_PREFIX_SIZE,
    MIN_RECORD_LENGTH,
    Record,
    UnsupportedClassBRecord,
    UnknownMsgRecord,
    DecodeError,
    crc16_ccitt_false,
    _ENVELOPE_STRUCT,
)

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Primitive struct format codes (all little-endian via the outer "<").
# ---------------------------------------------------------------------------
_PRIM_FORMATS: Dict[str, str] = {
    "u8": "B", "i8": "b",
    "u16": "H", "i16": "h",
    "u32": "I", "i32": "i",
    "u64": "Q", "i64": "q",
    "f32": "f", "f64": "d",
    "bool": "?",
}

_FIELD_RE = re.compile(
    r"^(?P<core>"
    r"(?:u8|u16|u32|u64|i8|i16|i32|i64|f32|f64|bool|string|bytes)"
    r"|(?:type:[a-z_][a-z0-9_]*)"
    r"|(?:enum:[a-z_][a-z0-9_]*\.[a-z_][a-z0-9_]*)"
    r")(?:\[(?P<count>[1-9][0-9]*)\])?$"
)


def _pascal(name: str) -> str:
    return "".join(part.capitalize() for part in name.split("_"))


def _parse_type(field_type: str) -> Tuple[str, Optional[int]]:
    """Split a registry type string into (core, optional fixed-array count)."""

    m = _FIELD_RE.match(field_type)
    if not m:
        raise ValueError(f"unrecognised field type: {field_type!r}")
    core = m.group("core")
    count = int(m.group("count")) if m.group("count") else None
    return core, count


# ---------------------------------------------------------------------------
# Enum (IntEnum) factory per module.errors namespace.
# ---------------------------------------------------------------------------
def _build_enum(module_name: str, errors: Dict[str, Any]) -> type:
    """Build an IntEnum subclass for one module's errors table.

    Includes a synthetic ``ok = 0`` member so the common
    ``error_code == 0`` success case has a friendly name. Unknown codes
    are surfaced by overriding ``_missing_`` to return a dynamically-
    registered ``unknown_<n>`` member.
    """

    members: Dict[str, int] = {"ok": 0}
    for name, entry in errors.items():
        if entry["code"] == 0:
            raise ValueError(f"{module_name}.errors.{name}: code 0 is reserved for 'ok'")
        members[name] = entry["code"]

    cls_name = f"{_pascal(module_name)}Errors"
    enum_cls: Any = IntEnum(cls_name, members)

    # IntEnum's _missing_ default raises; override so unknown wire values
    # decode to a synthetic member rather than blowing up the whole record.
    def _missing_(cls, value):  # type: ignore[no-redef]
        if not isinstance(value, int) or value < 0 or value > 0xFFFF:
            return None
        # Use enum's extension protocol to add a new member at runtime.
        new = int.__new__(cls, value)
        new._name_ = f"unknown_{value}"
        new._value_ = value
        return new

    enum_cls._missing_ = classmethod(_missing_)
    return enum_cls


# ---------------------------------------------------------------------------
# FieldSpec: a single resolved field's wire description.
# ---------------------------------------------------------------------------
@dataclass
class _FieldSpec:
    py_name: str
    # 'prim' | 'enum' | 'type' (composite). Arrays are handled via repeat.
    kind: str
    repeat: int = 1
    # primitive: struct format char (e.g. 'I'); enum: same as backing prim ('H');
    # composite: empty string (handled by sub-unpacker).
    fmt_char: str = ""
    py_type: Any = None
    # composite-only: callable taking a list of already-unpacked primitives
    # for this sub-field and returning the composite object.
    sub_unpack: Optional[Callable[[List[Any]], Any]] = None
    # composite-only: how many primitive slots the sub-unpacker consumes.
    sub_slots: int = 0
    # enum-only: factory(int) -> IntEnum member.
    enum_cls: Any = None


# ---------------------------------------------------------------------------
# Decoder: holds the message tables and offers iter_records().
# ---------------------------------------------------------------------------
class Decoder:
    def __init__(self, registry: Registry):
        self.registry = registry
        # (module_id, msg_id) -> _MsgSpec
        self._by_id: Dict[Tuple[int, int], "_MsgSpec"] = {}
        # (module_name, message_name) -> _MsgSpec (for CLI filter convenience).
        self._by_name: Dict[Tuple[str, str], "_MsgSpec"] = {}
        # composite-type name -> _CompositeSpec
        self._composites: Dict[str, "_CompositeSpec"] = {}
        # module_name -> IntEnum class (built from module.errors)
        self._enums: Dict[str, type] = {}

        self._warned_no_header = False

        self._build_enums()
        self._build_composites()
        self._build_messages()

    # ---- builders --------------------------------------------------------
    def _build_enums(self) -> None:
        for mod_name, mod in self.registry.modules.items():
            errors = mod.get("errors", {})
            self._enums[mod_name] = _build_enum(mod_name, errors)

    def _build_composites(self) -> None:
        for tname, tdef in self.registry.types.items():
            self._composites[tname] = _CompositeSpec.build(
                tname, tdef["fields"], self._composites, self._enums
            )

    def _build_messages(self) -> None:
        for mod_name, mod in self.registry.modules.items():
            module_id = mod["module_id"]
            for msg_name, msg in mod.get("messages", {}).items():
                if msg["class"] != "A":
                    continue  # phase 1: only Class A
                spec = _MsgSpec.build(
                    module_name=mod_name,
                    module_id=module_id,
                    msg_name=msg_name,
                    msg=msg,
                    composites=self._composites,
                    enums=self._enums,
                )
                self._by_id[(module_id, msg["msg_id"])] = spec
                self._by_name[(mod_name, msg_name)] = spec

    # ---- public lookup --------------------------------------------------
    def message_specs(self) -> List["_MsgSpec"]:
        return list(self._by_id.values())

    def get_spec(self, module_name: str, msg_name: str) -> "_MsgSpec":
        return self._by_name[(module_name, msg_name)]

    # ---- header / version policy ---------------------------------------
    def _check_registry_crc(self, header_crc: Optional[int]) -> Optional[DecodeError]:
        """Phase-1 fail-closed stub.

        When firmware starts emitting a 'log start' record carrying the
        baked-in ``REGISTRY_CRC32``, ``iter_records`` will pass it here.
        For now ``header_crc`` is always ``None`` and we warn once on the
        first record so the integration path is obvious.
        """
        if header_crc is None:
            if not self._warned_no_header:
                warnings.warn(
                    "SD log has no registry header yet; skipping fail-closed "
                    "registry_crc32 check (phase-1 behaviour).",
                    RuntimeWarning,
                    stacklevel=3,
                )
                self._warned_no_header = True
            return None
        if header_crc != self.registry.crc32:
            return DecodeError(
                offset=0,
                reason=(
                    f"registry CRC mismatch: log header says "
                    f"0x{header_crc:08x}, this registry is 0x{self.registry.crc32:08x}"
                ),
            )
        return None

    # ---- main iterator --------------------------------------------------
    def iter_records(self, stream: BinaryIO) -> Iterator[Any]:
        """Yield one record per SD log entry.

        Yields generated per-message dataclasses for Class A,
        ``UnsupportedClassBRecord`` for Class B,
        ``UnknownMsgRecord`` for unknown (module_id, msg_id) pairs, and
        ``DecodeError`` for CRC/struct/length failures (decoding continues).
        """

        # Stub fail-closed check (no in-log header yet).
        crc_check = self._check_registry_crc(None)
        if crc_check is not None:
            yield crc_check
            return

        while True:
            offset = stream.tell() if hasattr(stream, "tell") else -1
            len_bytes = stream.read(LENGTH_PREFIX_SIZE)
            if not len_bytes:
                return  # clean EOF
            if len(len_bytes) < LENGTH_PREFIX_SIZE:
                yield DecodeError(offset=offset, reason="truncated length prefix")
                return
            (length,) = struct.unpack("<H", len_bytes)
            if length < MIN_RECORD_LENGTH:
                yield DecodeError(offset=offset, reason=f"length={length} below minimum")
                # Best effort: try to keep going by consuming what was
                # claimed; on truncation we'll exit on the next read.
                _ = stream.read(length)
                continue
            body = stream.read(length)
            if len(body) < length:
                yield DecodeError(offset=offset, reason="truncated record body")
                return

            envelope = body[:ENVELOPE_SIZE]
            payload = body[ENVELOPE_SIZE : length - CRC_SIZE]
            (got_crc,) = struct.unpack("<H", body[length - CRC_SIZE :])
            want_crc = crc16_ccitt_false(body[: length - CRC_SIZE])
            if got_crc != want_crc:
                yield DecodeError(
                    offset=offset,
                    reason=f"crc mismatch: got 0x{got_crc:04x} want 0x{want_crc:04x}",
                    raw=body,
                )
                continue

            class_byte, module_id, msg_id, t_us_publish = _ENVELOPE_STRUCT.unpack(envelope)

            if class_byte == CLASS_B:
                warnings.warn(
                    f"skipping Class B record module_id={module_id} msg_id={msg_id} "
                    "(phase 2 will decode these)",
                    RuntimeWarning,
                    stacklevel=2,
                )
                yield UnsupportedClassBRecord(
                    module_id=module_id, msg_id=msg_id,
                    t_us_publish=t_us_publish, payload=payload,
                )
                continue

            if class_byte != CLASS_A:
                yield DecodeError(
                    offset=offset,
                    reason=f"unknown class byte 0x{class_byte:02x}",
                    raw=body,
                )
                continue

            spec = self._by_id.get((module_id, msg_id))
            if spec is None:
                yield UnknownMsgRecord(
                    class_byte=class_byte, module_id=module_id, msg_id=msg_id,
                    t_us_publish=t_us_publish, payload=payload,
                )
                continue

            try:
                record = spec.decode(payload, t_us_publish)
            except struct.error as e:
                yield DecodeError(
                    offset=offset,
                    reason=f"struct unpack failed for {spec.full_name}: {e}",
                    raw=body,
                )
                continue

            yield record


# ---------------------------------------------------------------------------
# Specs for composites + messages.
# ---------------------------------------------------------------------------
@dataclass
class _CompositeSpec:
    name: str
    py_type: Any
    fmt: str         # "<...": full struct format string (incl. byte-order).
    fields: List[_FieldSpec]
    size: int        # struct size in bytes for sanity.

    @staticmethod
    def build(
        type_name: str,
        fields_def: List[Dict[str, Any]],
        composites: Dict[str, "_CompositeSpec"],
        enums: Dict[str, type],
    ) -> "_CompositeSpec":
        py_name = _pascal(type_name)
        py_fields: List[Tuple[str, Any]] = []
        specs: List[_FieldSpec] = []
        fmt_parts: List[str] = []
        for fdef in fields_def:
            spec, sub_fmt, py_type = _resolve_field(fdef, composites, enums)
            specs.append(spec)
            fmt_parts.append(sub_fmt)
            py_fields.append((spec.py_name, py_type))
        fmt = "<" + "".join(fmt_parts)
        # All composite types must currently be pure-primitive (the
        # registry's only composites are vec3_f32 / quat_wxyz). If we
        # ever nest composites, _resolve_field handles it via sub_unpack
        # but the precomputed `fmt` would no longer match — we lift the
        # composite into a flat format then reassemble in decode().
        cls = make_dataclass(py_name, py_fields)
        return _CompositeSpec(
            name=py_name, py_type=cls,
            fmt=fmt, fields=specs, size=struct.calcsize(fmt),
        )

    def decode(self, raw: bytes) -> Any:
        values = struct.unpack(self.fmt, raw)
        return self._from_flat(values)[0]

    def _from_flat(self, values: Tuple[Any, ...], start: int = 0) -> Tuple[Any, int]:
        """Consume primitive slots from ``values`` starting at ``start`` and
        return (instance, new_start)."""

        kwargs: Dict[str, Any] = {}
        idx = start
        for fspec in self.fields:
            if fspec.kind == "prim":
                if fspec.repeat == 1:
                    kwargs[fspec.py_name] = values[idx]
                    idx += 1
                else:
                    kwargs[fspec.py_name] = list(values[idx : idx + fspec.repeat])
                    idx += fspec.repeat
            elif fspec.kind == "enum":
                if fspec.repeat == 1:
                    kwargs[fspec.py_name] = fspec.enum_cls(values[idx])
                    idx += 1
                else:
                    kwargs[fspec.py_name] = [fspec.enum_cls(v) for v in values[idx : idx + fspec.repeat]]
                    idx += fspec.repeat
            elif fspec.kind == "type":
                # Composite-of-composite: delegate to the sub-spec.
                sub_spec: _CompositeSpec = fspec.py_type  # type: ignore[assignment]
                # NOTE: requires that composite-of-composite fmt was inlined.
                if fspec.repeat == 1:
                    inst, idx = sub_spec._from_flat(values, idx)
                    kwargs[fspec.py_name] = inst
                else:
                    arr = []
                    for _ in range(fspec.repeat):
                        inst, idx = sub_spec._from_flat(values, idx)
                        arr.append(inst)
                    kwargs[fspec.py_name] = arr
            else:
                raise AssertionError(f"unknown field kind {fspec.kind}")
        return self.py_type(**kwargs), idx


@dataclass
class _MsgSpec:
    full_name: str       # "module.message" for diagnostics
    py_name: str         # PascalCase class name
    module_name: str
    module_id: int
    msg_name: str
    msg_id: int
    py_type: Any
    fmt: str
    fields: List[_FieldSpec]
    size: int

    @staticmethod
    def build(
        module_name: str, module_id: int,
        msg_name: str, msg: Dict[str, Any],
        composites: Dict[str, _CompositeSpec],
        enums: Dict[str, type],
    ) -> "_MsgSpec":
        py_name = _pascal(module_name) + _pascal(msg_name)
        py_fields: List[Tuple[str, Any]] = [("t_us_publish", int)]
        specs: List[_FieldSpec] = []
        fmt_parts: List[str] = []
        for fdef in msg["fields"]:
            spec, sub_fmt, py_type = _resolve_field(fdef, composites, enums)
            specs.append(spec)
            fmt_parts.append(sub_fmt)
            py_fields.append((spec.py_name, py_type))
        fmt = "<" + "".join(fmt_parts)
        cls = make_dataclass(py_name, py_fields, bases=(Record,))
        return _MsgSpec(
            full_name=f"{module_name}.{msg_name}",
            py_name=py_name,
            module_name=module_name, module_id=module_id,
            msg_name=msg_name, msg_id=msg["msg_id"],
            py_type=cls, fmt=fmt, fields=specs,
            size=struct.calcsize(fmt),
        )

    def decode(self, payload: bytes, t_us_publish: int) -> Any:
        if len(payload) != self.size:
            raise struct.error(
                f"{self.full_name}: payload length {len(payload)} != expected {self.size}"
            )
        values = struct.unpack(self.fmt, payload)
        kwargs: Dict[str, Any] = {"t_us_publish": t_us_publish}
        idx = 0
        for fspec in self.fields:
            if fspec.kind == "prim":
                if fspec.repeat == 1:
                    kwargs[fspec.py_name] = values[idx]
                    idx += 1
                else:
                    kwargs[fspec.py_name] = list(values[idx : idx + fspec.repeat])
                    idx += fspec.repeat
            elif fspec.kind == "enum":
                if fspec.repeat == 1:
                    kwargs[fspec.py_name] = fspec.enum_cls(values[idx])
                    idx += 1
                else:
                    kwargs[fspec.py_name] = [fspec.enum_cls(v) for v in values[idx : idx + fspec.repeat]]
                    idx += fspec.repeat
            elif fspec.kind == "type":
                sub: _CompositeSpec = fspec.py_type  # the spec, not the dataclass
                if fspec.repeat == 1:
                    inst, idx = sub._from_flat(values, idx)
                    kwargs[fspec.py_name] = inst
                else:
                    arr = []
                    for _ in range(fspec.repeat):
                        inst, idx = sub._from_flat(values, idx)
                        arr.append(inst)
                    kwargs[fspec.py_name] = arr
            else:
                raise AssertionError(f"unknown field kind {fspec.kind}")
        return self.py_type(**kwargs)


# ---------------------------------------------------------------------------
# Field resolution: takes one registry field dict and returns
#   (FieldSpec, contribution_to_struct_format, python_type_for_dataclass)
# Note: ``py_type`` is the actual dataclass for composites (so the
# generated message dataclass has the proper type annotation), but
# ``spec.py_type`` for composite fields holds the *_CompositeSpec*
# (so the decoder can recursively re-flatten primitives).
# ---------------------------------------------------------------------------
def _resolve_field(
    fdef: Dict[str, Any],
    composites: Dict[str, _CompositeSpec],
    enums: Dict[str, type],
) -> Tuple[_FieldSpec, str, Any]:
    py_name = fdef["name"]
    core, count = _parse_type(fdef["type"])
    repeat = count or 1

    if core in _PRIM_FORMATS:
        fmt_char = _PRIM_FORMATS[core] * repeat
        py_type: Any = list if count else _prim_py_type(core)
        return (
            _FieldSpec(py_name=py_name, kind="prim", repeat=repeat,
                       fmt_char=fmt_char, py_type=py_type),
            fmt_char, py_type,
        )

    if core in ("string", "bytes"):
        raise ValueError(
            f"field {py_name!r} type {core!r} is Class-B only; not legal in Class A"
        )

    if core.startswith("type:"):
        tname = core[len("type:") :]
        sub = composites.get(tname)
        if sub is None:
            raise ValueError(f"field {py_name!r} references unknown type {tname!r}")
        # Inline the sub-composite's primitive layout into our format string
        # (strip its leading byte-order marker, which is just '<').
        sub_fmt = sub.fmt.lstrip("<") * repeat
        py_type = list if count else sub.py_type
        return (
            _FieldSpec(py_name=py_name, kind="type", repeat=repeat,
                       fmt_char=sub_fmt, py_type=sub),
            sub_fmt, py_type,
        )

    if core.startswith("enum:"):
        # enum:<mod>.<name>  -- where <name> is either 'errors' (whole table)
        # or a specific error name. Either way it encodes as u16 and decodes
        # via the module's IntEnum class.
        ref = core[len("enum:") :]
        mod_name, _, _ = ref.partition(".")
        enum_cls = enums.get(mod_name)
        if enum_cls is None:
            raise ValueError(f"field {py_name!r} references unknown enum namespace {ref!r}")
        fmt_char = "H" * repeat
        py_type = list if count else enum_cls
        return (
            _FieldSpec(py_name=py_name, kind="enum", repeat=repeat,
                       fmt_char=fmt_char, py_type=enum_cls, enum_cls=enum_cls),
            fmt_char, py_type,
        )

    raise ValueError(f"field {py_name!r}: unhandled type core {core!r}")


def _prim_py_type(core: str) -> Any:
    if core in ("f32", "f64"):
        return float
    if core == "bool":
        return bool
    return int


# ---------------------------------------------------------------------------
# Public factory.
# ---------------------------------------------------------------------------
def build_decoder(registry: Registry) -> Decoder:
    """Build a decoder bound to ``registry``.

    The decoder caches dataclass types + struct unpackers, so build once
    and reuse across many log files.
    """

    return Decoder(registry)
