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
- `appimagetool`

## Output

The script stages the app into an `AppDir` and then generates an AppImage from that tree.
