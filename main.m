%% metadata
% Title:            Model Predictive Motion Planning
% Subject:          16.32 Optimal Control
% Author:           Se Hwan Jeon
% Description:      Model predictive motion planner formulation
% Dependencies:     Casadi + IPOPT (included)

clear; clc; close all;

%% dependencies
% addpath(genpath('casadi/casadi_linux'));   % choose version of casadi depending on OS
addpath(genpath('casadi/casadi_windows'));
addpath(genpath('models'));
addpath(genpath('simulation'));
addpath(genpath('controllers'));
addpath(genpath('utilities'));
addpath(genpath('visualization'));
import casadi.*

%% testing/extra code
% GET RID OF MPC CONTROLLER 
% CLEAN UP CODE

%% flags
options.simType = 'noisy';  % OPTIONS: {'deterministic', 'noisy', 'obstacles'}
saveAni = true;                     % save animation

%% model parameters
mass = 50;
inertia = [10 0 0; 0 25 0; 0 0 30];
shipLength = 5;
maxThrust = 25;
maxTorque = 10;

%% model initialization
enterprise = Spaceship(mass, inertia, shipLength, maxThrust, maxTorque);
state_init = zeros(12,1);
% state_target = [0 0 0 10 1.5 3 0 0 0 0 0 0]';
state_target = [-pi/8 pi/4 0 15 0.2 1.5 0.2 .5 .7 0 0 0]';
obstacles = [3 7; .5 0; 0 0; 1 0.5]; 
% obstacles = [];

%% simulation
output = simulation(enterprise, state_init, state_target, obstacles, options);

%% animation and plots
plotData.target = state_target;
plotData.state = output.state;
plotData.input = output.input;
plotData.t_sim = output.t_sim;
plotData.replannedPaths = output.replannedPaths;
plotData.replan_idx = output.replan_idx;
plotData.obstacles = obstacles;

animation(plotData, saveAni);                  % generate and save animation
plots(plotData, enterprise);                   % produce plots


