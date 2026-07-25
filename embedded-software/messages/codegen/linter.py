"""Bus-budget linter.

For every Class A message, sum `max_rate_hz * (envelope_size + payload_size)`
bytes per channel. Multiplied by 8 to convert to bits/s. If any channel
exceeds its declared `max_wire_bps`, raise BudgetExceeded.

Envelope size (see registry spec): u16 length + u8 class + u8 module_id +
u16 msg_id + u64 t_us + u16 crc16 = 16 bytes.
"""

from __future__ import annotations

from typing import Any

from .types_resolve import TypeResolver

ENVELOPE_SIZE = 16  # bytes


class BudgetExceeded(Exception):
    pass


def compute_budget(
    registry: dict[str, Any], resolver: TypeResolver
) -> dict[str, dict[str, Any]]:
    """Return per-channel budget summary.

    {channel_name: {'used_bps': int, 'max_bps': int, 'entries': [...]}}
    """
    channels = registry["channels"]
    summary: dict[str, dict[str, Any]] = {
        chname: {
            "used_bps": 0,
            "max_bps": int(ch["max_wire_bps"]),
            "channel_id": ch["channel_id"],
            "entries": [],
        }
        for chname, ch in channels.items()
    }

    for modname, mod in registry["modules"].items():
        for msgname, msg in (mod.get("messages") or {}).items():
            if msg["class"] != "A":
                continue
            payload_size = resolver.message_payload_size(msg["fields"])
            wire_size = ENVELOPE_SIZE + payload_size
            for chname, route in (msg.get("routing") or {}).items():
                if not route.get("enabled"):
                    continue
                rate = route.get("max_rate_hz")
                if rate is None or rate <= 0:
                    # 0 / unlimited contributes nothing to the budget;
                    # the registry author has explicitly opted out of
                    # rate limiting and accepted unbounded usage.
                    continue
                bps = int(rate * wire_size * 8)
                summary[chname]["used_bps"] += bps
                summary[chname]["entries"].append(
                    {
                        "module": modname,
                        "message": msgname,
                        "rate_hz": rate,
                        "wire_size": wire_size,
                        "bps": bps,
                    }
                )

    return summary


def check_budget(summary: dict[str, dict[str, Any]]) -> None:
    for chname, info in summary.items():
        if info["used_bps"] > info["max_bps"]:
            raise BudgetExceeded(
                f"channel {chname!r} bus budget exceeded: "
                f"{info['used_bps']} bps required, {info['max_bps']} bps available"
            )
