function duty = hb_sm_local_consensus_correction_v1(sm_id, m_arm, i_arm, Vc_arm, Vc_prev, Vc_next, N, params)
%HB_SM_LOCAL_CONSENSUS_CORRECTION_V1 Candidate Local Consensus correction controller.
%#codegen
%
% This function is a development candidate and is not yet wired into the
% uploaded Simulink model. It extends neighbor consensus by adding a local
% neighbor-order score while keeping the control law local.

if nargin < 8 || isempty(params)
    params = mmc_default_control_params();
end

Vc_i = double(Vc_arm(sm_id));

% Local Consensus error.
err_consensus = 0.5 * (double(Vc_prev) + double(Vc_next)) - Vc_i;

if abs(err_consensus) < params.deadband_V
    err_consensus = 0.0;
end

% Local neighbor-order score: 0 = lowest, 1 = middle, 2 = highest.
neighbor_order_score = double(Vc_i > double(Vc_prev)) + double(Vc_i > double(Vc_next));
neighbor_order_centered = neighbor_order_score - 1.0;

% Smooth current direction.
dir = tanh(double(i_arm) / params.current_scale_A);

% Local Consensus correction plus neighbor-order correction.
dm_consensus = params.k_v * err_consensus * dir;
dm_neighbor_order = -params.k_neighbor_order * neighbor_order_centered * dir;

m_i = double(m_arm) + dm_consensus + dm_neighbor_order;
duty = min(max(m_i, 0.0), 1.0);
end
