%% Clean everything
clear all; clc; 
% close all;

%% Sequence and Twist configuration

data_save = false; % true : save in a .mat file the following data : duty / VHigh / V1Low / V2Low / IHigh / I1Low / I2Low
power_losses = false; % true : takes into account power losses from the MOSFETs / false : no power losses
isInterleaved = false; % true : 180° phase shift between leg 1 and leg 2 / false : 0° phase shift
HaveChargingPhase = true;
isC1low1Active = false;
isC2low1Active = false;
is6VExternallySupplied = true;
isFeederJumperOpen = true;
isDutyCycleRamp = false;
C1low_variable = false;

% Initialize times and frequencies
twist_freq_data_sampling = 20e3; %data sampling frequency
twist_data_sampling_period = 1/twist_freq_data_sampling; %data sampling delay
twist_data_acquisition_delay = 50e-9; %data acquisition at the peak of the carry

%% Simulation configuration


% latest_sim_name = 'MMC_phase_hackathon_circuit1_HAAS_20260216.slx';
% latest_sim_name = 'MMC_phase_hackathon_circuit2_HAAS_20260216.slx';
% latest_sim_name = 'MMC_phase_hackathon_circuit3_HAAS_20260216.slx';
latest_sim_name = 'MMC_phase_hackathon_circuit4_HAAS_20260216.slx';


latest_MMC_parameters = "param_phase_circuit4_20260216.m";
latest_twist_parameters = "twist_parameters.m";

sim_time = 1.5; %overall simulation time
t_init_sim = 0; % [s] Simulation start time
Ts = 5e-7; % [s] Simulation sample period

time_charge = 0.1; % [s] Charging time - used only if HaveChargingPhase = true
time_decharge = 1.5; % [s] Decharging time

duty_cycle = 0.95;
%% Initialize twist parameters

run(latest_twist_parameters);

if(is6VExternallySupplied == false)
    feeder_current_lower_rate_limit = -300;
else
    feeder_current_lower_rate_limit = -10;

end

%% Initialize MMC parameters

run(latest_MMC_parameters);
%% Setup depending on user configuration

if(~HaveChargingPhase)
    source_V = VDC/N;  %source voltage [V] - inicial module capacitor voltage
else
    source_V = 0;  %source voltage [V] - inicial module capacitor voltage
end

if(~isInterleaved)
    twist_phase_shift_leg_2_deg = 0; %no phase shift
else
    twist_phase_shift_leg_2_deg = 180; %interleaved phase shift
end

if(~isC1low1Active)
    C1low1_active = 0;
else
    C1low1_active = 1;
end

if(~isC2low1Active)
    C2low1_active = 0;
else
    C2low1_active = 1;
end

%% Start simulink simulation

tic

% open(latest_sim_name);
sim(latest_sim_name);

if(data_save == true)
% Data output save
results = ans;
VDC_name = string(VDC);
    if(is6VExternallySupplied == false)
        if(isC1low1Active == true)
            save('sim_results/20260216/results_phase_circuit4_VDC'+VDC_name+'V_freq'+round(f0)+'Hz_Larm'+round(L_arm*1e6)+'uH_Cm'+round(C_m*1e6)+'uF_feeder_avecC1low_detailedmeasure','results');
        else
            save('sim_results/20260216/results_phase_circuit4_VDC'+VDC_name+'V_freq'+round(f0)+'Hz_Larm'+round(L_arm*1e6)+'uH_Cm'+round(C_m*1e6)+'uF_feeder_sansC1lowelec_detailedmeasure','results');
        end
    else
        if(isFeederJumperOpen == true)
            if(isC1low1Active == true)
                save('sim_results/20260216/results_phase_circuit4_VDC'+VDC_name+'V_freq'+round(f0)+'Hz_Larm'+round(L_arm*1e6)+'uH_Cm'+round(C_m*1e6)+'uF_6Vext_avecC1low_jumperopen_detailedmeasure','results');
            else
                save('sim_results/20260216/results_phase_circuit4_VDC'+VDC_name+'V_freq'+round(f0)+'Hz_Larm'+round(L_arm*1e6)+'uH_Cm'+round(C_m*1e6)+'uF_R'+round(R)+'Ohms_6Vext_sansC1lowelec_jumperopen_CVB_detailedmeasure','results');
            end
        else
            if(isC1low1Active == true)
                save('sim_results/20260216/results_phase_circuit4_VDC'+VDC_name+'V_freq'+round(f0)+'Hz_Larm'+round(L_arm*1e6)+'uH_Cm'+round(C_m*1e6)+'uF_6Vext_avecC1low_detailedmeasure','results');
            else
                save('sim_results/20260216/results_phase_circuit4_VDC'+VDC_name+'V_freq'+round(f0)+'Hz_Larm'+round(L_arm*1e6)+'uH_Cm'+round(C_m*1e6)+'uF_6Vext_sansC1lowelec_detailedmeasure','results');
            end
        end
    end
end


toc


