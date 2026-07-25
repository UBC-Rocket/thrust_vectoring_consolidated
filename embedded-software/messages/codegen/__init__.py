"""Phase-1 codegen for the UBC Rocket message registry.

Consumes embedded-software/messages/registry.json (the single source of
truth) and emits packed C structs + a routing table + publish macros for
the firmware to memcpy onto the wire. Class B messages, commands, and
nanopb .proto generation are out of scope here — they belong to phase 2.
"""
