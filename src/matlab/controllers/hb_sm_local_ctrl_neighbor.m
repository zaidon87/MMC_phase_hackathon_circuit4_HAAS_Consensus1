% Extracted from MMC_phase_hackathon_circuit4_HAAS_Consensus1.slx for review and version control.
function gate_signal = hb_sm_local_ctrl_neighbor(sm_id, m_arm, i_arm, Vc_arm, Vc_prev, Vc_next, N)
%#codegen
% =========================================================
% LOCAL CONSENSUS BALANCING (v9) - matched to existing PSC-PWM hardware
%
% Implements the user's hb_sm_local_ctrl_consensus algorithm:
%   - Consensus balancing with neighbour-only Vref (distributed)
%   - Voltage deadband suppresses correction near steady state
%   - tanh(i_arm/Iscale) smooth sign avoids zero-current discontinuity
%
% Output is m_i as a continuous duty in [0, 1]. The downstream PWM
% Generator block carries the phase-shifted carrier (Phase = sps.Pc,
% configured per-SM in the parameter file) and produces the actual
% gate signal -- so we get phase-shifted PWM switching at the arm
% level without needing to generate a carrier in software.
%
% Inputs N and Vc_arm beyond v_current are unused but kept in the
% signature so the parent subsystem wiring is unchanged.
% =========================================================

% --- 1. PARAMETERS (verbatim from user spec) ---
k_v      = double(0.2);    % Balancing gain (1/V)
deadband = double(0.1);    % Voltage-error deadband (V)
Iscale   = double(1.0);    % Current scaling for tanh smoothing (A)

% --- 2. CONSENSUS BALANCING WITH LOCAL NEIGHBOUR REFERENCE ---
Vc_i = double(Vc_arm(sm_id));

% err: deviation of this SM from its immediate neighbours.
%   err > 0  -> this SM is BELOW neighbour average -> needs more charge
%   err < 0  -> this SM is ABOVE neighbour average -> needs less charge
err = double(0.5 * (Vc_prev + Vc_next)) - Vc_i;

% Deadband: ignore micro-imbalance near steady state.
if abs(err) < deadband
    err = 0.0;
end

% dir: smooth current-direction signal via tanh.
% Avoids the discontinuity of sign() near zero current crossings.
dir = tanh(double(i_arm) / Iscale);

% dm: balancing correction on the modulation index
dm = k_v * err * dir;

% m_i: per-SM corrected modulation index, clamped to [0, 1]
m_i = min(max(double(m_arm) + dm, 0.0), 1.0);

% --- 3. OUTPUT: duty cycle for the downstream phase-shifted PWM block ---
gate_signal = double(m_i);

end
