# Local- Correction v1

## Purpose

This document proposes the next professional development step for the neighbor-consensus MMC controller.

The current model uses immediate neighbor capacitor voltages. The next step is to generalize this into a scalable local-rank correction that works for arbitrary `N` and avoids hardcoded submodule counts.

## Local voltage target

For submodule `i`:

```matlab
Vlocal_ref = 0.5 * (Vc_prev + Vc_next);
err = Vlocal_ref - Vc_i;
```

## Current-direction weighting

The duty correction should depend on arm-current direction:

```matlab
dir = tanh(i_arm / Iscale);
dm = k_v * err * dir;
```

## Deadband

A voltage deadband prevents unnecessary switching when voltages are already close:

```matlab
if abs(err) < deadband
    dm = 0;
end
```

## Saturation

The corrected modulation command must remain inside `[0, 1]`:

```matlab
m_i = min(max(m_arm + dm, 0), 1);
```

## Acceptance criteria

- No hardcoded `N = 5`.
- Works for `N = 4`, `N = 5`, `N = 10`, and larger arms.
- Keeps capacitor voltage spread lower than the open-loop/reference case.
- Does not create excessive high-frequency switching.
- Can be mapped later to OwnTech/Twist embedded implementation.
