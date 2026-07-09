function [Vc_prev, Vc_next] = get_neighbors_parameterized(Vc_arm, i, N)
%GET_NEIGHBORS_PARAMETERIZED Return previous and next neighbor voltages.
%#codegen
%
% This scalable version removes the hardcoded N = 5 assumption used in the
% original uploaded Simulink model.

if nargin < 3 || isempty(N)
    N = numel(Vc_arm);
end

assert(N >= 2, 'N must be at least 2.');
assert(i >= 1 && i <= N, 'Index i must be inside 1..N.');
assert(numel(Vc_arm) >= N, 'Vc_arm must contain at least N elements.');

idx_prev = mod(i - 2, N) + 1;
idx_next = mod(i, N) + 1;

Vc_prev = Vc_arm(idx_prev);
Vc_next = Vc_arm(idx_next);
end
