# Control Architecture

## Architecture level

The model follows a submodule-oriented architecture:

```text
Arm modulation reference m_arm
        |
        v
Submodule i local controller
        |
        |-- own capacitor voltage Vc_i
        |-- previous neighbor voltage Vc_prev
        |-- next neighbor voltage Vc_next
        |-- arm current i_arm
        v
Corrected duty m_i
        |
        v
Phase-shifted PWM / gate generation
```

## Neighbor-consensus balancing

The balancing target for submodule `i` is the average voltage of its two immediate neighbors:

```matlab
Vref_local = 0.5 * (Vc_prev + Vc_next);
err = Vref_local - Vc_i;
```

The correction is current-direction-aware:

```matlab
dir = tanh(i_arm / Iscale);
dm = k_v * err * dir;
```

## Why this is decentralized

The submodule does not need the complete capacitor-voltage vector for global sorting. It only requires immediate neighbor information and a shared modulation reference.

## Main engineering risk

The controller can reduce capacitor-voltage dispersion, but it must not create excessive high-frequency duty perturbations. This is why deadband, gain tuning, and switching-frequency metrics are required in the next validation step.
