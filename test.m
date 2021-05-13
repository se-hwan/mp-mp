%% metadata
% Title:            Model Predictive Motion Planning
% Subject:          16.32 Optimal Control
% Author:           Se Hwan Jeon
% Description:      Model predictive planner formulation
% Dependencies:     Casadi

clear; clc; close all;

%% dependencies
%addpath(genpath('casadi/casadi_linux'));   % choose version of casadi depending on OS
addpath(genpath('casadi/casadi_windows'));
addpath(genpath('models'));
addpath(genpath('simulation'));
addpath(genpath('controllers'));
addpath(genpath('utilities'));
addpath(genpath('visualization'));
import casadi.*

%% test stuff
n = 100;
test_path = [linspace(0,20,n); zeros(1,n); zeros(1,n)];
obstacles = [5; .3; 0; 1];
pathIn.X = [zeros(3,n); test_path];

%% test plot
figure; grid on; hold on;
plot3(test_path(1,:), test_path(2,:), test_path(3,:), 'r--')

newPath = checkCollisions(pathIn, obstacles);
[X,Y,Z] = sphere;
surf(obstacles(4)*X + obstacles(1), ...
     obstacles(4)*Y + obstacles(2), ...
     obstacles(4)*Z + obstacles(3))


plot3(newPath(1,:), newPath(2,:), newPath(3,:), 'bo-')

hold off;
