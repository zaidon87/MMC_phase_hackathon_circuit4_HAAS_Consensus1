# Changelog

## 0.3.0 - 2026-07-09

### Changed

- Renamed the embedded control terminology from `local-rank` to **Local Consensus**.
- Replaced `src/mmc_local_rank.hpp` with `src/mmc_local_consensus.hpp`.
- Renamed C/C++ build flag to `MMC_LOCAL_CONSENSUS_CONTROL`.
- Updated README, build guide, hardware checklist, and integration notes.
- Updated MATLAB candidate controller name to `hb_sm_local_consensus_correction_v1.m`.

## 0.2.0 - 2026-07-09

### Added

- Reorganized repository according to the requested GitHub structure.
- Added `.github/ISSUE_TEMPLATE` with control-focused issue templates.
- Added `.vscode` workspace settings and extension recommendations.
- Added `MMC_documentation` for thesis-style documentation.
- Added `MMC_models` structure for Simulink model artifacts.
- Added `owntech` folder for OwnTech Twist / embedded mapping notes.
- Added `zephyr` placeholder configuration.
- Added `platformio.ini` placeholder for future embedded testing.
- Added parameterized neighbor lookup candidate and smoke tests.

### Known limitations

- The uploaded Simulink model still contains a hardcoded `N = 5` neighbor lookup.
- Simulation was not executed in this environment because MATLAB/Simulink is not available.

## 0.1.0 - Initial packaging

- Packaged uploaded SLX model with extracted controller functions and static review notes.
