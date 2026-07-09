%SETUP_PATH Add repository MATLAB folders to path.

repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
addpath(genpath(fullfile(repoRoot, 'MMC_models')));

disp(['Repository path configured: ', repoRoot]);
