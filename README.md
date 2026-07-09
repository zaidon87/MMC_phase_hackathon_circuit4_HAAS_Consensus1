# MMC Neighbor-Consensus Control

Professional research repository for a **Modular Multilevel Converter (MMC)** Simulink model implementing **neighbor-only capacitor-voltage balancing** for decentralized submodule control.

The repository follows the requested structure:

```text
.github/ISSUE_TEMPLATE/      GitHub issue templates
.vscode/                     VS Code / MATLAB-oriented workspace settings
MMC_documentation/           Thesis-style model and control documentation
MMC_models/                  Simulink model artifacts
models/                      Model folder note and conventions
owntech/                     OwnTech Twist / embedded hardware notes
src/                         MATLAB controller source and embedded candidates
zephyr/                      Zephyr placeholder configuration
platformio.ini               PlatformIO placeholder for embedded experiments
```

## Repository objective

This project is intended to support PhD-level development and publication-quality review of a decentralized MMC control architecture. The current model uses a local neighbor-consensus principle: each submodule computes its balancing action from its own capacitor voltage, the previous neighbor voltage, and the next neighbor voltage.

## Model included

| Item | Value |
|---|---|
| Main Simulink file | `MMC_models/original/MMC_phase_hackathon_circuit4_HAAS_Consensus1.slx` |
| MATLAB / Simulink release detected | R2024a |
| Main control function detected | `hb_sm_local_ctrl_neighbor(...)` |
| Neighbor lookup detected | `get_neighbors(...)` |
| Current limitation | `get_neighbors` is hardcoded for `N = 5` in the uploaded model |

## Control principle

For each submodule `i`, the controller estimates the local capacitor-voltage error from the immediate neighbors:

```matlab
err = 0.5 * (Vc_prev + Vc_next) - Vc_i;
dir = tanh(i_arm / Iscale);
dm  = k_v * err * dir;
m_i = min(max(m_arm + dm, 0.0), 1.0);
```

This keeps the balancing law local and avoids a centralized sorting stage. The corrected continuous duty/modulation output can then be used by a downstream phase-shifted PWM block.

## Quick start in MATLAB

```matlab
cd path/to/mmc-neighbor-consensus-control
run('src/matlab/scripts/setup_path.m')
run('src/matlab/scripts/open_model.m')
run('src/matlab/tests/run_smoke_checks.m')
```

To run a short simulation with fallback workspace variables:

```matlab
run('src/matlab/scripts/run_simulation.m')
```

## Development roadmap

The next professional development step is not to add more diagrams; it is to make the controller scalable and experimentally reproducible:

1. Replace hardcoded `N = 5` with a parameterized neighbor function.
2. Move controller constants `k_v`, `deadband`, and `Iscale` to a parameter file or Simulink data dictionary.
3. Add repeatable experiment scenarios: nominal, initial imbalance, load step, and switching-frequency comparison.
4. Export capacitor-voltage traces, switching counters, THD, and loss metrics into `results/`.
5. Prepare a short branch for publication: `refactor/local-rank-correction-v1`.

## Important note

The `.slx` file is binary. For professional GitHub review, the controller functions were extracted into source files under `src/matlab/` so that future changes can be reviewed line by line.

## License and citation

Use `LICENSE` and `CITATION.cff` as starting points. Update author, supervisor, institution, and paper/thesis title before public release.
