#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build-linux}"
DESKTOP_FILE="ulysses-ground-control.desktop"
APPDIR="${APPDIR:-${BUILD_DIR}/AppDir}"
BUILD_JOBS="${BUILD_JOBS:-1}"
QMAKE_BIN="${QMAKE_BIN:-$(command -v qmake6 || command -v qmake)}"
APPIMAGE_PATH="${APPIMAGE_PATH:-${BUILD_DIR}/ulysses-ground-control-x86_64.AppImage}"
QT_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}"

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

if [[ -z "${QT_PREFIX_PATH}" ]]; then
    if [[ -n "${QMAKE_BIN}" && -x "${QMAKE_BIN}" ]]; then
        QT_PREFIX_PATH="$("${QMAKE_BIN}" -query QT_INSTALL_PREFIX 2>/dev/null || true)"
    fi
fi

if [[ -z "${QT_PREFIX_PATH}" ]]; then
    echo "Could not determine the Qt installation prefix."
    echo "Set CMAKE_PREFIX_PATH or make qmake6/qmake available."
    exit 1
fi

rm -rf "${BUILD_DIR}"

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="${QT_PREFIX_PATH}"
cmake --build "${BUILD_DIR}" --parallel "${BUILD_JOBS}"

rm -rf "${APPDIR}"
cmake --install "${BUILD_DIR}" --prefix "${APPDIR}/usr"

pushd "${BUILD_DIR}" >/dev/null
echo "Running linuxdeployqt..."
APPIMAGETOOL_APP_NAME="${APPIMAGETOOL_APP_NAME:-ulysses-ground-control}" \
ARCH="${ARCH:-x86_64}" \
linuxdeployqt "${APPDIR}/usr/share/applications/${DESKTOP_FILE}" -bundle-non-qt-libs -qmake="${QMAKE_BIN}"
echo "linuxdeployqt finished."

echo "Running appimagetool..."
ARCH="${ARCH:-x86_64}" appimagetool "${APPDIR}" "${APPIMAGE_PATH}"
echo "appimagetool finished."
popd >/dev/null

echo "AppImage packaging complete."
