# Static Model Review

## Source model

`MMC_phase_hackathon_circuit4_HAAS_Consensus1.slx`

Detected MATLAB/Simulink release: **R2024a**.

## Main finding

The model contains MATLAB Function logic consistent with decentralized MMC submodule control using local neighbor voltage information. The most important detected function is:

```matlab
hb_sm_local_ctrl_neighbor(sm_id, m_arm, i_arm, Vc_arm, Vc_prev, Vc_next, N)
```

## Current limitation

The neighbor lookup used in the uploaded model is hardcoded for `N = 5`:

```matlab
N = 5;
idx_prev = mod(i - 2, N) + 1;
idx_next = mod(i, N) + 1;
```

This should be replaced by a parameterized implementation before testing larger MMC arms.

## Recommended validation signals

- Individual capacitor voltages `Vc_i`.
- Arm current `i_arm`.
- Corrected submodule duty `m_i`.
- Gate signals per submodule.
- Output voltage and current.
- Switching frequency / switching counter per submodule.
- Conduction, switching, gate, and dead-time loss estimates.

## Simulation note

The model was statically inspected. It was not executed in this environment because MATLAB/Simulink is not available here.
