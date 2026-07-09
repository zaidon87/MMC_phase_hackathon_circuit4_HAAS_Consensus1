# OwnTech-Style Repository Port

## Reference style

The repository was reorganized to follow the practical OwnTech Power API style:

- `README.md` explains how to clone, open in VS Code, and build with PlatformIO.
- `platformio.ini` defines the board/shield selection and includes additional configuration files.
- `src/app.ini` contains application-specific PlatformIO environments.
- `src/main.cpp` is the embedded entry point.
- `owntech/` and `zephyr/` hold advanced board and RTOS support placeholders.

## What was adapted

| Area | Adaptation |
|---|---|
| README | Rewritten around OwnTech + PlatformIO workflow. |
| PlatformIO | Default `USB` environment, `STLink` option, Spin/Twist board parameters. |
| Source entry | Added `src/main.cpp` with MMC IDs, status codes, frame helpers, and Local Consensus controller. |
| OwnTech support | Added `owntech/pio_extra.ini` placeholder. |
| App config | Added `src/app.ini` with `USB`, `STLink`, and `native` environments. |

## Important distinction

This is not a full OwnTech Core fork. It is a research repository styled to be compatible with the OwnTech workflow. To deploy on real hardware, it must be tested inside a complete OwnTech-supported toolchain with the correct board support package.

## Next engineering step

Split `src/main.cpp` into smaller modules after the first hardware test:

```text
src/
├── main.cpp
├── mmc_config.hpp
├── mmc_frame.hpp
├── mmc_local_consensus.hpp
├── mmc_measurements.hpp
└── mmc_tasks.cpp
```
