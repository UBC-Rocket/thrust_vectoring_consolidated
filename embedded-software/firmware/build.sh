#!/usr/bin/env bash
# Build STM32H745 dual-core firmware (CM7 + CM4).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FC_DIR="${SCRIPT_DIR}/flight_controller"

usage() {
  cat <<'EOF'
Usage: build.sh [Debug|Release] [both|cm7|cm4]

Build the STM32H745 dual-core flight controller firmware.

  Debug|Release   CMake preset (default: Debug)
  both|cm7|cm4    Cores to build (default: both)

CM4 servo driver defaults to Dynamixel (USE_DYNAMIXEL_SERVO=ON).
To build with Feetech instead: USE_FEETECH_SERVO=1 ./build.sh

Examples:
  ./build.sh
  ./build.sh Release
  ./build.sh Debug cm7
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

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "error: required command not found: $1" >&2
    exit 1
  fi
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

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

PRESET="$(normalize_preset "${1:-Debug}")"
CORE="$(normalize_core "${2:-both}")"

require_cmd cmake
require_cmd ninja
require_cmd arm-none-eabi-gcc
require_cmd arm-none-eabi-size

BUILD_CONTEXT="CM4_CM7"
case "$CORE" in
  cm7) BUILD_CONTEXT="CM7" ;;
  cm4) BUILD_CONTEXT="CM4" ;;
esac

echo "==> Building STM32H745 firmware (preset=${PRESET}, cores=${CORE})"
cd "$FC_DIR"

USE_DYNAMIXEL_SERVO=ON
if [[ "${USE_FEETECH_SERVO:-}" == "1" ]]; then
  USE_DYNAMIXEL_SERVO=OFF
fi

CMAKE_ARGS=(
  --preset "$PRESET"
  -DUSE_DYNAMIXEL_SERVO="${USE_DYNAMIXEL_SERVO}"
  -DBUILD_CONTEXT="$BUILD_CONTEXT"
)

if [[ "$USE_DYNAMIXEL_SERVO" == "ON" ]]; then
  echo "    CM4 servo: Dynamixel (set USE_FEETECH_SERVO=1 for Feetech)"
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build --preset "$PRESET"

echo
echo "Build complete."
if [[ "$CORE" == "both" || "$CORE" == "cm7" ]]; then
  CM7_ELF="$(resolve_elf cm7 "$PRESET")"
  if [[ -f "$CM7_ELF" ]]; then
    echo "CM7: $CM7_ELF"
    arm-none-eabi-size "$CM7_ELF"
  else
    echo "warning: CM7 ELF not found at $CM7_ELF" >&2
  fi
fi
if [[ "$CORE" == "both" || "$CORE" == "cm4" ]]; then
  CM4_ELF="$(resolve_elf cm4 "$PRESET")"
  if [[ -f "$CM4_ELF" ]]; then
    echo "CM4: $CM4_ELF"
    arm-none-eabi-size "$CM4_ELF"
  else
    echo "warning: CM4 ELF not found at $CM4_ELF" >&2
  fi
fi
