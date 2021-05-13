% Car race along a track
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP

clear; clc; close all;

addpath(genpath('casadi/casadi_windows'));

N = 100; % number of control intervals

opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
X = opti.variable(2,N+1); % state trajectory
pos   = X(1,:);
speed = X(2,:);
U = opti.variable(1,N);   % control trajectory (throttle)
U_max = opti.variable(1,N);   % control trajectory (throttle)
T = opti.variable();      % final time


% ---- dynamic constraints --------
f = @(x,u) [x(2);u]; % dx/dt = f(x,u)

dt = T/N; % length of a control interval
cost = 0;
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),         U(:,k));
   k2 = f(X(:,k)+dt/2*k1, U(:,k));
   k3 = f(X(:,k)+dt/2*k2, U(:,k));
   k4 = f(X(:,k)+dt*k3,   U(:,k));
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4);
   x_next = X(:,k) + dt * f(X(:,k),U(:,k));
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
   cost = cost + dt + .5*U_max(:,k);
end

% ---- objective          ---------
opti.minimize(cost); % race in minimal time

% ---- path constraints -----------
%limit = @(pos) 1-sin(2*pi*pos)/2;
%opti.subject_to(speed<=limit(pos)); % track speed limit
opti.subject_to(-1<=U);
opti.subject_to(U<=1);           % control is limited

opti.subject_to(U(:) <= U_max(:))
opti.subject_to(-U(:) <= U_max(:))

% ---- boundary conditions --------
opti.subject_to(pos(1)==1);  
opti.subject_to(speed(1)==1);
opti.subject_to(pos(N+1)==0); % finish line at position 1
opti.subject_to(speed(N+1)==0); % finish line at position 1

% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive

% ---- initial values for solver ---
%opti.set_initial(speed, 1);
%opti.set_initial(T, 1);

% ---- solve NLP              ------
opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

% ---- post-processing        ------
time = sol.value(T);
time = 0:time/N:time;

figure
hold on
plot(time,sol.value(pos))
plot(time,sol.value(speed))
xlabel('Time')
ylabel('Posn and vel')
legend('Posn', 'Vel')
hold off

figure
plot(sol.value(pos), sol.value(speed))
xlabel('x')
ylabel('x_dot')
