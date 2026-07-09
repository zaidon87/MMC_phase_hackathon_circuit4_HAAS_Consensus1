# Integration Notes from `analuhaas/MMC`

## Purpose

This note records what was learned from the reference OwnTech/MMC repository and how it was applied to this repository.

## Reference concepts applied

The reference project uses an OwnTech Power API layout:

- `platformio.ini` at repository root.
- `extra_configs` pointing to `owntech/pio_extra.ini` and `src/app.ini`.
- all application code starts from `src/main.cpp`.
- OwnTech APIs are used for `SpinAPI`, `TaskAPI`, `ShieldAPI`, and `CommunicationAPI`.
- the firmware uses RS485 frames between one lead board and several submodule boards.
- capacitor voltage and arm current are compactly encoded into 12-bit fields.
- a serial menu controls idle, power, and data acquisition modes.
- the critical task runs at a fixed microsecond-scale period.

## Adaptation in this repository

`src/main.cpp` was rewritten as a bridge between the Simulink Local Consensus controller and the OwnTech/Twist hardware workflow.

Implemented blocks:

1. OwnTech-style includes and task layout.
2. MMC board IDs from `MMC_LEAD` to `MMC_SM10`.
3. MMC status states: `IDLE`, `POWER`, `LEAD_ERROR`, `OVER_VOLTAGE`, `UNDER_VOLTAGE`, `OVER_CURRENT`.
4. Compact `MMC_frame_t` with:
   - insertion bitfield,
   - capacitor-voltage raw value,
   - arm-current raw value,
   - status field,
   - submodule ID.
5. RS485 reception/relay logic.
6. Measurement update from Twist sensors.
7. Lead-board NLM reference generation.
8. Local Consensus balancing priority based on neighbor capacitor voltages.
9. Follower-board command application.
10. ScopeMimicry acquisition placeholders.

## Technical distinction

The reference firmware uses capacitor-voltage sorting for CVB. This repository replaces that decision rule with Local Consensus / neighbor-consensus priority to match the Simulink control research direction.

## Validation still required

This code has been prepared structurally but must still be compiled and tested with the full OwnTech/Zephyr/PlatformIO environment. The first hardware validation should check:

- board UID detection,
- RS485 ring ordering,
- voltage/current scaling,
- safety transitions to idle mode,
- correct insertion bits per submodule,
- switching behavior on Twist hardware.
