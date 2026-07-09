# Project Status

## Current state

This repository contains a professional structure for the MMC neighbor-consensus Simulink research model.

## Completed

- Repository initialized on GitHub.
- Professional README added.
- MATLAB / Simulink support files added.
- Documentation folders prepared.
- Extracted controller source prepared for version control.
- Embedded and OwnTech placeholder structure prepared.

## Important technical finding

The uploaded Simulink model contains the local neighbor controller `hb_sm_local_ctrl_neighbor(...)` and a neighbor lookup function that is hardcoded for `N = 5` submodules. This should be generalized before scaling to larger MMC arms.

## Not completed in this environment

- MATLAB/Simulink simulation was not executed here.
- The binary `.slx` file should be uploaded from the local computer if it is not already present, because the available GitHub connector is optimized for UTF-8 text files.

## Recommended next branch

```text
refactor/local-rank-correction-v1
```

## Immediate next steps

1. Upload or confirm the original `.slx` model under `MMC_models/original/`.
2. Replace hardcoded neighbor indexing with a parameterized version.
3. Move controller constants to `src/matlab/config/` or a Simulink data dictionary.
4. Run nominal, imbalance, and load-step experiments.
5. Export results to `results/`.
