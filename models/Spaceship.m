classdef Spaceship
    properties
        N_states{}      % number of states
        N_inputs{}      % number of inputs
        Mass{}          % mass
        Inertia{}       % inertia along principal axes
        I_inv{}         % inverse of inertia
        Length{}        % length of spaceship
        MaxThrust{}    % maximum power for thrust
        MaxTorque{}     % maximum torque
        r_thrust{}      % vector to thrusters from CoM, body frame
    end
    methods
        function obj = Spaceship(m, I, l, maxThrust, maxTorque) % constructor
            if (nargin > 0)
                obj.N_states = 12; 
                obj.N_inputs = 6;
                obj.Mass = m;
                obj.Inertia = I;
                obj.I_inv = inv(I);
                obj.Length = l;
                obj.r_thrust = [-obj.Length/2 0 0]';
                obj.MaxThrust = maxThrust;
                obj.MaxTorque = maxTorque;
            end
        end
        function [f_world, f_local] = f(obj, x, u)
            rpy = x(1:3); R = rpyToR(rpy);
            F = u(1:3); tau = u(4:6);
            omega = x(7:9);
            vel = x(10:12);
            alpha = obj.I_inv*cross(obj.r_thrust, F) + obj.I_inv*tau;
            accel = [(F(1))/obj.Mass F(2)/obj.Mass F(3)/obj.Mass]';
            
            f_world = [R*omega; R*vel; R*alpha; R*accel];
            f_local = [omega; vel; alpha; accel];
        end
        function [f_world, f_local] = f_gravity(obj, x, u)
            rpy = x(1:3); R = rpyToR(rpy);
            F = u(1:3); tau = u(4:6);
            omega = x(7:9);
            vel = x(10:12);
            alpha = obj.I_inv*cross(obj.r_thrust, F) + obj.I_inv*tau;
            accel = [(F(1))/obj.Mass F(2)/obj.Mass F(3)/obj.Mass]';
            
            f_world = [R*omega; R*vel; R*alpha; R*accel - [0 0 5]'/obj.Mass];
            f_local = [omega; vel; alpha; accel];
        end
    end
end