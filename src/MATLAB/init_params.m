clear; clc; close all;

% Initial state
x0 = [85;0;0;0;0;0;0;0.1;0];

% Static inputs (can be replaced by joystick inputs)
u = [0; -0.1; 0; 0.08; 0.08];

% Simulation parameters
dt = 1/50;
t_final = 15;

% Run Simulink model
sim('fbw_simulink.slx');
