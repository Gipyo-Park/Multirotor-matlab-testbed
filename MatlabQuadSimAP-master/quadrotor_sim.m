%-----------------------------------------------------------------------%
%                                                                       %
%   This script simulates quadrotor dynamics and implements a control   %                                %
%   algrotihm.                                                          %
%   Developed by: Wil Selby                                             %
%                                                                       %
%                                                                       %
%-----------------------------------------------------------------------%

% Add Paths
addpath utilities

%% Initialize Workspace
clear all;
close all;
clc;

global Quad;

%% Initialize the plot
init_plot;
plot_quad_model;

%% Initialize Variables
% full_model_dynamics;

quad_variables;
model_dynamics;
quad_ACS;
quad_dynamics_nonlinear;


%% Run The Simulation Loop
while Quad.t_plot(Quad.counter-1)< max(Quad.t_plot);    
    
    % Measure Parameters (for simulating sensor errors)
      sensor_meas;

    % Filter Measurements
%     Kalman_phi2;
%     Kalman_theta2;
%     Kalman_psi2;
%     Kalman_Z2;
%     Kalman_X2;
%     Kalman_Y2;


    % if (Quad.counter < 100)
    %     Quad.phi_des = pi/6;          % desired value of phi (radians)
    %     Quad.theta_des = 0;        % desired value of theta (radians)
    %     Quad.psi_des = 0;          % desired value of psi (radians)
    % elseif (Quad.counter < 300)
    %     Quad.phi_des = 0;          % desired value of phi (radians)
    %     Quad.theta_des = pi/6;        % desired value of theta (radians)
    %     Quad.psi_des = 0;          % desired value of psi (radians)
    % elseif (Quad.counter < 600)
    %     Quad.phi_des = -pi/6;          % desired value of phi (radians)
    %     Quad.theta_des = 0;        % desired value of theta (radians)
    %     Quad.psi_des = 0;          % desired value of psi (radians)
    % elseif (Quad.counter < 900)
    %     Quad.phi_des = 0;          % desired value of phi (radians)
    %     Quad.theta_des = -pi/6;        % desired value of theta (radians)
    %     Quad.psi_des = 0;          % desired value of psi (radians)
    % end

    % if (Quad.counter < 1000)
    %     Quad.X_des_GF = 0;         % desired value of X in Global frame
    %     Quad.Y_des_GF = 0;         % desired value of Y in Global frame
    %     Quad.Z_des_GF = 1;         % desired value of Z in Global frame
    % elseif (Quad.counter < 3000)
    %     Quad.X_des_GF = 1;         % desired value of X in Global frame
    %     Quad.Y_des_GF = 1;         % desired value of Y in Global frame
    %     Quad.Z_des_GF = 1;         % desired value of Z in Global frame
    % elseif (Quad.counter < 6000)
    %     Quad.X_des_GF = 1;         % desired value of X in Global frame
    %     Quad.Y_des_GF = -1;         % desired value of Y in Global frame
    %     Quad.Z_des_GF = 1;         % desired value of Z in Global frame
    % elseif (Quad.counter < 9000)
    %     Quad.X_des_GF = -1;         % desired value of X in Global frame
    %     Quad.Y_des_GF = -1;         % desired value of Y in Global frame
    %     Quad.Z_des_GF = 1;         % desired value of Z in Global frame
    % end

    
    % yaw 안 맞추고 Lemniscate 경로 생성
    Trajectory_Lemniscate;



    
    % Position Controller
    position_PID;


    % Attitude Controller
    % attitude_PID;
    % attitude_LQR;
    % attitude_LQI;
    % attitude_MPC;
    attitude_NMPC;


    % Attitude / Altitude Controller
    % AttAlt_MPC;
    
    
    % Rate Controller
    % rate_PID;
    
    % Combined Controller
    % full_LQR;



    % Calculate Desired Motor Speeds
    quad_motor_speed;
    
    % Update Position With The Equations of Motion
    quad_dynamics_nonlinear;    
    
    % Plot the Quadrotor's Position
    if(mod(Quad.counter,3)==0)
        plot_quad 
        
        % campos([A.X+2 A.Y+2 A.Z+2])
        % camtarget([A.X A.Y A.Z])
        % camroll(0);
        Quad.counter;
        drawnow
    end
    
    Quad.init = 1;  %Ends initialization after first simulation iteration
end

%% Plot Data
plot_data

% Performance Metrics Calculation
calculate_performance;

% 시뮬 후
figure();
plot3(Quad.X_log, Quad.Y_log, Quad.Z_log,'--');
xlabel('X'), ylabel('Y'), zlabel('Z');
title('Lemniscate Trajectory');
grid on;
hold on;
plot3(Quad.X_des_log, Quad.Y_des_log, Quad.Z_des_log);
legend('simulated trajectory', 'reference');
