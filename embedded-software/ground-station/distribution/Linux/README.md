# Ulysses Ground Control Linux Packaging

This directory contains the files used to package the ground control station for Linux distribution.

## Recommended path

Build an AppImage from a Linux machine or Linux CI runner:

```bash
./distribution/Linux/build-appimage.sh
```

## Requirements

- Qt 6 with the same modules used by the app
- CMake 3.21+
- `linuxdeployqt`

## Containerized build

If you want to build without installing the Linux toolchain on your host, use Docker:

```bash
./distribution/Linux/build-appimage-docker.sh
```

The resulting AppImage will appear under `build-linux/` in the project root.
The container defaults to `linux/amd64` so the AppImage tooling can run reliably.

If the container runs out of memory, lower the compile parallelism:

```bash
BUILD_JOBS=1 ./distribution/Linux/build-appimage-docker.sh
```

## Output

The script stages the app into an `AppDir` and then generates an AppImage from that tree.

## GitHub Actions

The repository also includes a non-Docker AppImage workflow at
`.github/workflows/ground-station-appimage.yml`. It runs on an Ubuntu runner,
installs Qt plus the AppImage tooling, calls `build-appimage.sh`, and uploads
the generated AppImage as a workflow artifact.
