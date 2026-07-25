#!/usr/bin/env bash
# Flash STM32H745 dual-core firmware (CM7 @ 0x08000000, CM4 @ 0x08100000) via ST OpenOCD.
#
# For single-core debug, disable DUAL_CORE_BOOT_SYNC_SEQUENCE in both CM7 and CM4
# main.c before flashing only one core.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FC_DIR="${SCRIPT_DIR}/flight_controller"

usage() {
  cat <<'EOF'
Usage: flash.sh [Debug|Release] [both|cm7|cm4]

Flash STM32H745 dual-core firmware using ST OpenOCD.

Requires environment variables:
  STM32_OPENOCD_PATH           Path to OpenOCD binary (from STM32CubeIDE)
  STM32_OPENOCD_SCRIPTS_PATH   Path to OpenOCD st_scripts directory

  Debug|Release   ELF build type to flash (default: Debug)
  both|cm7|cm4    Cores to flash (default: both; CM7 is programmed before CM4)

Examples:
  ./flash.sh
  ./flash.sh Release both
  ./flash.sh Debug cm4
EOF
}

to_lower() {
  printf '%s' "$1" | tr '[:upper:]' '[:lower:]'
}

normalize_preset() {
  local arg
  arg="$(to_lower "$1")"
  case "$arg" in
    debug) echo "Debug" ;;
    release) echo "Release" ;;
    "") echo "Debug" ;;
    *)
      if [[ "$1" == "Debug" || "$1" == "Release" ]]; then
        echo "$1"
      else
        echo "error: unknown preset '$1' (expected Debug or Release)" >&2
        exit 1
      fi
      ;;
  esac
}

normalize_core() {
  local arg
  arg="$(to_lower "$1")"
  case "$arg" in
    both|"") echo "both" ;;
    cm7) echo "cm7" ;;
    cm4) echo "cm4" ;;
    *)
      echo "error: unknown core '$1' (expected both, cm7, or cm4)" >&2
      exit 1
      ;;
  esac
}

resolve_elf() {
  local core="$1"
  local preset="$2"
  local primary secondary

  if [[ "$core" == "cm7" ]]; then
    primary="${FC_DIR}/CM7/build/ulysses-fc_CM7.elf"
    secondary="${FC_DIR}/CM7/build/${preset}/ulysses-fc_CM7.elf"
  else
    primary="${FC_DIR}/CM4/build/ulysses-fc_CM4.elf"
    secondary="${FC_DIR}/CM4/build/${preset}/ulysses-fc_CM4.elf"
  fi

  if [[ -f "$primary" ]]; then
    echo "$primary"
  elif [[ -f "$secondary" ]]; then
    echo "$secondary"
  else
    echo "$primary"
  fi
}

require_elf() {
  local elf="$1"
  local core="$2"
  if [[ ! -f "$elf" ]]; then
    echo "error: ${core} ELF not found: $elf" >&2
    echo "Run ./build.sh first." >&2
    exit 1
  fi
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ -z "${STM32_OPENOCD_PATH:-}" ]]; then
  echo "error: STM32_OPENOCD_PATH is not set" >&2
  echo "Set it to the OpenOCD binary bundled with STM32CubeIDE." >&2
  exit 1
fi
if [[ -z "${STM32_OPENOCD_SCRIPTS_PATH:-}" ]]; then
  echo "error: STM32_OPENOCD_SCRIPTS_PATH is not set" >&2
  echo "Set it to the OpenOCD st_scripts directory from STM32CubeIDE." >&2
  exit 1
fi
if [[ ! -x "$STM32_OPENOCD_PATH" ]]; then
  echo "error: STM32_OPENOCD_PATH is not executable: $STM32_OPENOCD_PATH" >&2
  exit 1
fi
if [[ ! -d "$STM32_OPENOCD_SCRIPTS_PATH" ]]; then
  echo "error: STM32_OPENOCD_SCRIPTS_PATH is not a directory: $STM32_OPENOCD_SCRIPTS_PATH" >&2
  exit 1
fi

PRESET="$(normalize_preset "${1:-Debug}")"
CORE="$(normalize_core "${2:-both}")"

# STM32CubeIDE sets these before sourcing stm32h7x.cfg for H745 (see st_scripts/board/stm32_st_board.vm).
# Without them, OpenOCD fails with: can't read "AP_NUM": no such variable
OPENOCD_INIT=(
  -c "set DUAL_CORE 1"
  -c "set DUAL_BANK 1"
  # Full-chip reset via SRST (CubeIDE default for SWD). Core-only reset leaves
  # CM4 in a bad state and CM7 can fault during dual-core boot sync.
  -c "reset_config srst_only srst_nogate"
  -c "set CORE_RESET 0"
)
case "$CORE" in
  cm4)
    OPENOCD_INIT+=(-c "set AP_NUM 3")
    OPENOCD_TARGET="stm32h7x.cm4"
    ;;
  *)
    # AP_NUM 0 exposes CM7 only, but DUAL_BANK gives CM7 access to bank 2 @ 0x08100000.
    OPENOCD_INIT+=(-c "set AP_NUM 0")
    OPENOCD_TARGET="stm32h7x.cm7"
    ;;
esac

OPENOCD_CMDS=(-c "targets ${OPENOCD_TARGET}")
if [[ "$CORE" == "both" || "$CORE" == "cm7" ]]; then
  CM7_ELF="$(resolve_elf cm7 "$PRESET")"
  require_elf "$CM7_ELF" "CM7"
  OPENOCD_CMDS+=(-c "program ${CM7_ELF} verify")
  echo "==> Flashing CM7: $CM7_ELF"
fi
if [[ "$CORE" == "both" || "$CORE" == "cm4" ]]; then
  CM4_ELF="$(resolve_elf cm4 "$PRESET")"
  require_elf "$CM4_ELF" "CM4"
  OPENOCD_CMDS+=(-c "program ${CM4_ELF} verify")
  echo "==> Flashing CM4: $CM4_ELF"
fi

OPENOCD_CMDS+=(-c "reset run" -c "exit")

"$STM32_OPENOCD_PATH" \
  -s "$STM32_OPENOCD_SCRIPTS_PATH" \
  -f interface/stlink-dap.cfg \
  "${OPENOCD_INIT[@]}" \
  -f target/stm32h7x.cfg \
  "${OPENOCD_CMDS[@]}"

echo "Flash complete."
