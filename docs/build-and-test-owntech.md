# Build and Test Guide for OwnTech/Twist MMC Firmware

## Purpose

This document explains how to test the C/C++ side of the repository before using real hardware.

## 1. Update your local repository

```bash
cd ~/MMC_phase_hackathon_circuit4_HAAS_Consensus1
git pull origin main
```

## 2. Check the repository structure

```bash
ls src
ls owntech
cat platformio.ini
```

Expected files:

```text
src/main.cpp
src/app.ini
src/mmc_config.hpp
src/mmc_frame.hpp
src/mmc_local_rank.hpp
owntech/pio_extra.ini
platformio.ini
```

## 3. Native build smoke test

The repository includes a `native` environment in `src/app.ini`. This is useful for checking basic C++ syntax without hardware.

```bash
pio run -e native
```

If this fails because PlatformIO is not installed:

```bash
pip install platformio
```

or install the PlatformIO extension in VS Code.

## 4. OwnTech/Twist build

For hardware-oriented build:

```bash
pio run -e USB
```

For STLink:

```bash
pio run -e STLink
```

## 5. Upload to board

USB serial upload:

```bash
pio run -e USB -t upload
```

STLink upload:

```bash
pio run -e STLink -t upload
```

## 6. Serial menu

When the firmware runs, the serial interface supports:

```text
h : help menu
i : idle mode
p : power mode
r : record scope data
a : toggle scope acquisition trigger
```

## 7. Expected first errors

The first build may fail if the full OwnTech Core / Zephyr environment is not installed. Typical missing items:

- `SpinAPI.h`
- `TaskAPI.h`
- `ShieldAPI.h`
- `CommunicationAPI.h`
- `filters.h`
- `trigo.h`
- `ScopeMimicry.h`

These errors mean the OwnTech environment is incomplete, not that the MMC algorithm is wrong.

## 8. Hardware safety warning

Do not connect power hardware before confirming:

- duty cycle limits are correct,
- sensors are calibrated,
- capacitor voltage thresholds are correct,
- emergency idle transition works,
- RS485 board order is correct,
- PWM output is verified on oscilloscope at low voltage.
