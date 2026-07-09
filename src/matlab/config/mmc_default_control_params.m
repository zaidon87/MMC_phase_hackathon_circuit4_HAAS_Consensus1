function params = mmc_default_control_params()
%MMC_DEFAULT_CONTROL_PARAMS Default parameters for Local Consensus MMC controllers.

params = struct();
params.k_v = 0.2;                     % neighbor-consensus balancing gain
params.k_neighbor_order = 0.02;       % Local Consensus neighbor-order correction gain
params.deadband_V = 0.1;              % voltage error deadband
params.current_scale_A = 1.0;         % tanh current scaling
params.N = 5;                         % default number of submodules per arm
end
