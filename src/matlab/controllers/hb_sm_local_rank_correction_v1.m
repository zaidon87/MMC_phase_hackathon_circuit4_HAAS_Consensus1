function duty = hb_sm_local_rank_correction_v1(sm_id, m_arm, i_arm, Vc_arm, Vc_prev, Vc_next, N, params)
%HB_SM_LOCAL_RANK_CORRECTION_V1 Candidate local-rank correction controller.
%#codegen
%
% This function is a development candidate and is not yet wired into the
% uploaded Simulink model. It extends neighbor consensus by adding a local
% 3-node rank score.

if nargin < 8 || isempty(params)
    params = mmc_default_control_params();
end

Vc_i = double(Vc_arm(sm_id));

% Local consensus error.
err_consensus = 0.5 * (double(Vc_prev) + double(Vc_next)) - Vc_i;

if abs(err_consensus) < params.deadband_V
    err_consensus = 0.0;
end

% Local rank score: 0 = lowest, 1 = middle, 2 = highest.
rank_score = double(Vc_i > double(Vc_prev)) + double(Vc_i > double(Vc_next));
rank_centered = rank_score - 1.0;

% Smooth current direction.
dir = tanh(double(i_arm) / params.current_scale_A);

% Consensus correction plus rank correction.
dm_consensus = params.k_v * err_consensus * dir;
dm_rank = -params.k_rank * rank_centered * dir;

m_i = double(m_arm) + dm_consensus + dm_rank;
duty = min(max(m_i, 0.0), 1.0);
end
