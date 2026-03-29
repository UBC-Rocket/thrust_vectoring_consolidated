#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build-linux}"
DESKTOP_FILE="ulysses-ground-control.desktop"
APPDIR="${APPDIR:-${BUILD_DIR}/AppDir}"

if [[ "$(uname -s)" != "Linux" ]]; then
    echo "This script must be run on Linux."
    exit 1
fi

if ! command -v linuxdeployqt >/dev/null 2>&1; then
    echo "linuxdeployqt is required to produce the AppImage."
    exit 1
fi

if ! command -v appimagetool >/dev/null 2>&1; then
    echo "appimagetool is required to produce the AppImage."
    exit 1
fi

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" --parallel

rm -rf "${APPDIR}"
cmake --install "${BUILD_DIR}" --prefix "${APPDIR}/usr"

pushd "${BUILD_DIR}" >/dev/null
linuxdeployqt "${APPDIR}/usr/share/applications/${DESKTOP_FILE}" -appimage -bundle-non-qt-libs
popd >/dev/null
