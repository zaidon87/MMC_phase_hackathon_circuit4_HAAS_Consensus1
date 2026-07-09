%RUN_SMOKE_CHECKS Lightweight smoke checks for extracted MATLAB functions.

run(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'scripts', 'setup_path.m'));

fprintf('Running smoke checks...\n');

Vc = [10 11 12 13 14];
[vp, vn] = get_neighbors_parameterized(Vc, 1, 5);
assert(vp == 14 && vn == 11, 'Boundary neighbor check failed for i=1, N=5.');

[vp, vn] = get_neighbors_parameterized(Vc, 5, 5);
assert(vp == 13 && vn == 10, 'Boundary neighbor check failed for i=5, N=5.');

for N = [4 5 10]
    V = 1:N;
    for i = 1:N
        [vp, vn] = get_neighbors_parameterized(V, i, N);
        assert(~isempty(vp) && ~isempty(vn));
    end
end

m = hb_sm_local_ctrl_neighbor(1, 0.5, 1.0, Vc, 14, 11, 5);
assert(m >= 0 && m <= 1, 'Controller output must be clamped in [0,1].');

params = mmc_default_control_params();
m2 = hb_sm_local_consensus_correction_v1(1, 0.5, 1.0, Vc, 14, 11, 5, params);
assert(m2 >= 0 && m2 <= 1, 'Local Consensus controller output must be clamped in [0,1].');

fprintf('Smoke checks passed.\n');
