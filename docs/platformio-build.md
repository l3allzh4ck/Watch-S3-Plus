# Building the factory firmware with PlatformIO

This repository is both the LilyGoLib library (`src/`) and a PlatformIO
project that builds the factory touchscreen firmware (`examples/factory/`)
against those local sources.

Target: LilyGO T-Watch S3 Plus (ESP32-S3, 16MB flash, 8MB OPI PSRAM, SX1262).

## One-time setup (Windows)

The project uses the [pioarduino](https://github.com/pioarduino/platform-espressif32)
platform, which needs the **pioarduino IDE** VS Code extension rather than
PlatformIO IDE (both use `%USERPROFILE%\.platformio\penv`).

1. Disable or uninstall **PlatformIO IDE**, install **pioarduino IDE**, restart VS Code.
2. Enable Windows long paths (administrator PowerShell, then reboot). The
   framework package contains paths long enough to stall unpacking otherwise:
   ```powershell
   New-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" -Name "LongPathsEnabled" -Value 1 -PropertyType DWORD -Force
   ```
3. Pin pioarduino Core to 6.2.0 (with VS Code closed). Core `6.2.0.post1`
   rejects platform 55.03.312 as requiring Core `>=6.2.0`:
   ```powershell
   & "$env:USERPROFILE\.platformio\penv\Scripts\uv.exe" pip install --python "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" "pioarduino==6.2.0"
   ```

## Build

1. Open the repository folder in VS Code with the pioarduino IDE extension.
2. Select the `twatchs3` environment (the default).
3. Click **Build** (the tick in the status bar).

The first build downloads the platform and libraries and compiles everything,
which takes a while. Later builds only recompile what changed.

Output: `.pio/build/twatchs3/firmware.bin` (application, flashed at `0x10000`),
plus `bootloader.bin` (`0x0`) and `partitions.bin` (`0x8000`).

## Layout notes

- `platformio.ini` sets `src_dir = examples/factory` and pulls in the
  repository root as the LilyGoLib library with `symlink://.`.
- Downloaded libraries are kept outside the repository
  (`libdeps_dir = ../.pio-libdeps/Watch-S3-Plus`). PlatformIO attributes a
  header to whichever library folder contains it, so libraries stored under
  `<repo>/.pio/libdeps` would be claimed by LilyGoLib itself and LilyGoLib
  would miss their include paths (`lvgl.h: No such file or directory`).
- `examples/factory/factory.ino` is named `factory.cpp` so PlatformIO compiles
  it directly instead of converting a sketch.
- `boards/` and `variants/` come from
  [LilyGoLib-PlatformIO](https://github.com/Xinyuan-LilyGO/LilyGoLib-PlatformIO).
- Library versions follow [third_party.md](third_party.md). Optional apps
  (Walkie, INA219, ...) are only compiled in when their libraries are present.

## Verified baseline

Built with the pioarduino platform 55.03.312 (Arduino ESP32 core 3.3.12, GCC 14.2.0):

```
RAM:   25.1% (82176 / 327680 bytes)
Flash: 69.9% (2930561 / 4194304 bytes)
```

The menu matches the `twatchs3-sx1262-factory-20260919` recovery image.
