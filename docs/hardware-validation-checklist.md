# Hardware Validation Checklist

## Goal

Validate the MMC Local Consensus neighbor-control firmware safely before applying significant power.

## Phase 0 — Repository and build

- [ ] `git pull origin main` completed.
- [ ] `src/main.cpp` exists.
- [ ] `src/mmc_config.hpp` exists.
- [ ] `src/mmc_frame.hpp` exists.
- [ ] `src/mmc_local_consensus.hpp` exists.
- [ ] `pio run -e native` attempted.
- [ ] `pio run -e USB` attempted.

## Phase 1 — No power, board-only test

- [ ] Board detected by PlatformIO.
- [ ] Serial monitor opens.
- [ ] Help menu responds to `h`.
- [ ] Idle command `i` works.
- [ ] Power command `p` changes mode.
- [ ] LED behavior confirms mode transition.

## Phase 2 — Sensor sanity

- [ ] V_HIGH reads reasonable values.
- [ ] I1_LOW reads reasonable values.
- [ ] Zero-current offset checked.
- [ ] Voltage scaling checked.
- [ ] Current scaling checked.

## Phase 3 — RS485 ring

- [ ] Lead board UID detected.
- [ ] Follower board UIDs detected.
- [ ] Sender ID is correctly stored in `MMC_frame_t`.
- [ ] Lead receives capacitor voltage from followers.
- [ ] Followers relay frame only after the previous board.
- [ ] `counter_receive` increases correctly.

## Phase 4 — Gate command verification without power

- [ ] `gate_upper[0..4]` changes with modulation.
- [ ] Inserted bits match expected module IDs.
- [ ] Followers receive their own insertion command.
- [ ] PWM duty is zero in idle mode.
- [ ] PWM duty becomes active only in power mode.

## Phase 5 — Low-voltage power test

- [ ] DC supply current limit configured.
- [ ] Oscilloscope connected.
- [ ] PWM frequency verified.
- [ ] Duty bounds verified.
- [ ] Capacitor voltage stays below safety limit.
- [ ] Emergency idle transition tested.

## Phase 6 — Control validation

- [ ] Capacitor voltage spread decreases with Local Consensus control.
- [ ] Local Consensus gate selection matches neighbor-voltage logic.
- [ ] Switching frequency does not become excessive.
- [ ] Compare against centralized sorting baseline.
- [ ] Export measurement traces for thesis/report.
