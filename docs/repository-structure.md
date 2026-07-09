# Repository Structure

This repository follows the structure requested for a professional MMC / OwnTech development repository.

```text
.github/ISSUE_TEMPLATE/      GitHub issue templates
.vscode/                     VS Code workspace support
MMC_documentation/           Academic and thesis-style documentation
MMC_models/                  Simulink model artifacts
models/                      Compatibility folder for alternate model organization
owntech/                     OwnTech Twist and embedded hardware notes
src/                         Source code extracted from the Simulink model
zephyr/                      Zephyr RTOS placeholder project
results/                     Simulation result folders
platformio.ini               PlatformIO placeholder
```

## Design principle

The binary Simulink model is kept separate from the reviewable controller source. MATLAB Function block logic is extracted into `src/matlab/` so controller changes can be reviewed line by line in GitHub.

## Main source folders

| Folder | Purpose |
|---|---|
| `src/matlab/controllers` | Neighbor-consensus and local-rank controller functions. |
| `src/matlab/utils` | Utility functions such as neighbor lookup. |
| `src/matlab/config` | Nominal control and simulation parameters. |
| `src/matlab/scripts` | MATLAB scripts to open, check, and run the model. |
| `src/embedded` | C candidate implementation for embedded migration. |
