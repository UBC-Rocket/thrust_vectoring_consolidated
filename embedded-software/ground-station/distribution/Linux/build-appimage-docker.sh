#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-ulysses-ground-control-linux-builder}"
BUILD_JOBS="${BUILD_JOBS:-1}"
DOCKER_PLATFORM="${DOCKER_PLATFORM:-linux/amd64}"

if ! command -v docker >/dev/null 2>&1; then
    echo "Docker is required to build the Linux AppImage container."
    exit 1
fi

if docker buildx version >/dev/null 2>&1; then
    docker buildx build --platform "${DOCKER_PLATFORM}" --load -t "${IMAGE_NAME}" -f "${SCRIPT_DIR}/Dockerfile" "${SCRIPT_DIR}"
else
    docker build --platform "${DOCKER_PLATFORM}" -t "${IMAGE_NAME}" -f "${SCRIPT_DIR}/Dockerfile" "${SCRIPT_DIR}"
fi

docker run --rm \
    --platform "${DOCKER_PLATFORM}" \
    --user "$(id -u):$(id -g)" \
    -e BUILD_JOBS="${BUILD_JOBS}" \
    -v "${WORKSPACE_ROOT}:/workspace/embedded-software" \
    -w /workspace/embedded-software/ground-station \
    "${IMAGE_NAME}"

echo "AppImage build complete."
echo "Look in: ${PROJECT_ROOT}/build-linux"
