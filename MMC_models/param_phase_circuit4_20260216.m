%% Parameters settings
% clear all, clc

%% MMC parameters (exp config)

VDC = 48; % [V] DC voltage
f0 = 50; % [Hz] Electrical frequency
L_arm = 20e-3; % [H] Arm test inductance
C_bus = 4400e-6; % [H] Arm test inductance
R = 15; % [Ω] Arm test resistance
R_dec = R; % [Ω] Arm test resistance

% C_m = 188.4e-6; % [F] Module capacitor capacitance
C_m = 1868.4e-6; % [F] Module capacitor capacitance

C_M_extra = C_m-twist_high_side_C;
N=5; % [-] Total number of modules per arm

R_cablage = 1e-6;

%% Modulation parameters in ULabc (simple model)
m = 1;034; % Modulation for simple model -> Vac = m * VDC/2
a = 1; % Modulation for simple model -> Changes the state of charge of voltage module Vm = a * VDC/2 + m * VDC/2 sin(wt)

%% Parameters SW
f_control = 10e3; % [Hz] Control task frequency