"""Storage codegen — JSON registry → C headers + linker fragment.

Mirrors the structure of embedded-software/messages/codegen/ since the
two systems are conceptually parallel:
  - messages/ registry → wire-format C structs + nanopb + Python decoder
  - storage/  registry → flash-resident C structs + linker fragment +
                          runtime helpers consumed by lib/storage/.

The output drops into firmware/generated/storage/ and is built by
lib/storage which provides the load/save runtime.
"""
