function params = mmc_default_control_params()
%MMC_DEFAULT_CONTROL_PARAMS Default parameters for local MMC controllers.

params = struct();
params.k_v = 0.2;              % neighbor-consensus balancing gain
params.k_rank = 0.02;          % local-rank correction gain candidate
params.deadband_V = 0.1;       % voltage error deadband
params.current_scale_A = 1.0;  % tanh current scaling
params.N = 5;                  % default number of submodules per arm
end
