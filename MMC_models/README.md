# MMC Models

This folder stores Simulink model artifacts.

## Expected layout

```text
MMC_models/
├── original/
│   └── MMC_phase_hackathon_circuit4_HAAS_Consensus1.slx
├── working/
└── exports/
```

## Important note

The original `.slx` file is a binary Simulink file. It should be kept under `MMC_models/original/` and treated as a binary artifact by Git.

The controller logic extracted from the `.slx` model is stored in `src/matlab/` for professional code review.
