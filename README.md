# MMC Local Consensus Control

OwnTech/PlatformIO-style research repository for a **Modular Multilevel Converter (MMC)** arm controller with **neighbor-consensus capacitor-voltage balancing** and a **Local Consensus correction** layer.

This repository is organized to work like an OwnTech Power API project: the embedded entry point is `src/main.cpp`, board/shield configuration is in `platformio.ini`, application-specific configuration is in `src/app.ini`, and advanced board/Zephyr support is kept under `owntech/` and `zephyr/`.

## Project objective

The goal is to move from a Simulink-only MMC prototype toward a professional repository that can support:

- MATLAB/Simulink model review,
- extracted controller source-code review,
- OwnTech/Twist embedded migration,

## Repository layout

```text
MMC_phase_hackathon_circuit4_HAAS_Consensus1
├── .github/ISSUE_TEMPLATE/      GitHub issue templates for control, bugs, and features
├── .vscode/                     VS Code settings for OwnTech + MATLAB work
├── MMC_documentation/           Academic documentation and control notes
├── MMC_models/                  Original and derived Simulink models
├── docs/                        Repository, reproducibility, build, and validation notes
├── models/                      Optional simplified / derived model folder
├── owntech/                     OwnTech PlatformIO support placeholders
├── src/
│   ├── main.cpp                 OwnTech-style embedded entry point
│   ├── app.ini                  Application configuration included by PlatformIO
│   ├── mmc_config.hpp           MMC constants, module IDs, gains, safety limits
│   ├── mmc_frame.hpp            RS485 frame packing/unpacking helpers
│   ├── mmc_local_consensus.hpp  Local Consensus / neighbor-consensus helper functions
│   └── matlab/                  Extracted MATLAB controller source and tests
├── zephyr/                      Zephyr configuration placeholder
├── LICENSE
├── platformio.ini
└── README.md
```

## Downloading the repository

```bash
git clone https://github.com/zaidon87/MMC_phase_hackathon_circuit4_HAAS_Consensus1.git
cd MMC_phase_hackathon_circuit4_HAAS_Consensus1
```

Open the folder in VS Code and install the PlatformIO extension.

## Working with PlatformIO

The repository follows the OwnTech-style workflow:

```bash
pio run
pio run -e native
pio run -e USB
pio run -e STLink
```

The default environment is `USB`. The board/shield target is configured as:

```ini
board = spin
board_version = 1_2_0
board_shield = twist
board_shield_version = 1_4_2
```

Detailed build steps are documented in:

```text
docs/build-and-test-owntech.md
```

## Working with MATLAB / Simulink

The original model should be placed here:

```text
MMC_models/original/MMC_phase_hackathon_circuit4_HAAS_Consensus1.slx
```

Then in MATLAB:

```matlab
run('src/matlab/scripts/setup_path.m')
run('src/matlab/tests/run_smoke_checks.m')
```

## Main control idea

For submodule `i`, the Local Consensus controller uses only neighbor capacitor voltages:

```matlab
Vlocal_ref = 0.5 * (Vc_prev + Vc_next);
err = Vlocal_ref - Vc_i;
dir = tanh(i_arm / Iscale);
dm = k_v * err * dir;
m_i = min(max(m_arm + dm, 0.0), 1.0);
```

This avoids centralized global sorting and prepares the control law for distributed submodule implementation.

## Embedded direction

`src/main.cpp` is the OwnTech-style firmware prototype. It contains:

- OwnTech API includes,
- board/module identification constants,
- MMC status definitions,
- compact communication frame structure,
- capacitor-voltage/current encode-decode helpers,
- RS485 reception and relay logic,
- setup/background/critical tasks,
- NLM reference generation,
- Local Consensus gate selection for the upper arm,
- safety transition logic.

Reusable C/C++ pieces are also split into headers:

```text
src/mmc_config.hpp
src/mmc_frame.hpp
src/mmc_local_consensus.hpp
```

## Important technical note

The uploaded Simulink model was detected as MATLAB/Simulink R2024a and contains a neighbor lookup currently hardcoded around `N = 5`. The repository includes a parameterized replacement under:

```text
src/matlab/utils/get_neighbors_parameterized.m
```




