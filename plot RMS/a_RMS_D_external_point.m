%% 논문 식 (7)~(19) 간단 구현 예시
% "Required Moment Sets" 개념에 따라 멀티로터(혹은 eVTOL)에서 
% 에어스피드 경계, 자세 overshoot 등을 고려한 RMS(Disturbance) 산출

close all; clear; clc;

%% 1) 기본 파라미터 설정
param.m    = 20;                 % 질량 (kg)
param.g    = 9.81;               % 중력가속도 (m/s^2)
param.I    = diag([0.2, 0.02, 0.04]);  % 관성행렬 (kg·m^2)
param.rho  = 1.225;              % 공기 밀도 (kg/m^3)
param.Sref = 0.3;                % (단순) 기체 레퍼런스 면적 (m^2)
param.Cd   = 1.0;                % 드래그 계수 (가정)
param.kM   = 0.05;               % 에어로 모멘트 스케일(임의 예시)

% 논문 예시처럼 멀티로터의 사각형 형태 Airspeed Envelope
% 식 (7)~(9):
uC_min = -8;  uC_max = 8;
vC_min = -8;  vC_max = 8;
wC_min = -3;  wC_max = 3;
n_u = 3;  n_v = 3;  n_w = 3;  % 해상도 (테스트용으로 작게 잡음)

%% 2) Airspeed Envelope Boundary Points 구하기
u_vals = linspace(uC_min, uC_max, n_u);
v_vals = linspace(vC_min, vC_max, n_v);
w_vals = linspace(wC_min, wC_max, n_w);

[U, V, W] = ndgrid(u_vals, v_vals, w_vals);
mask = ( abs(U - uC_min) < 1e-6 | abs(U - uC_max) < 1e-6 | ...
         abs(V - vC_min) < 1e-6 | abs(V - vC_max) < 1e-6 | ...
         abs(W - wC_min) < 1e-6 | abs(W - wC_max) < 1e-6 );
U_b = U(mask);
V_b = V(mask);
W_b = W(mask);

%% 3) 자세 overshoot 범위 설정 (식 (11)~(14))
phi_trim   = 0;   % 본 예시: trim은 (phi=0, theta=0) 근처라 가정
theta_trim = 0;

phi_min = deg2rad(-20);
phi_max = deg2rad(20);
theta_min = deg2rad(-20);
theta_max = deg2rad(20);

phi_overshoot = deg2rad(5);
theta_overshoot = deg2rad(5);

phi_lower   = min(phi_min,   phi_trim - phi_overshoot);
phi_upper   = max(phi_max,   phi_trim + phi_overshoot);
theta_lower = min(theta_min, theta_trim - theta_overshoot);
theta_upper = max(theta_max, theta_trim + theta_overshoot);

n_phi   = 5;
n_theta = 5;
phi_range   = linspace(phi_lower,   phi_upper,   n_phi);
theta_range = linspace(theta_lower, theta_upper, n_theta);

%% 4) RMS 포인트 (Disturbance) 계산을 위한 배열
% 논문 식 (15)~(19) 결과를 저장할 공간: 
% [n_z, p_dot, q_dot, r_dot] or [T, L_RMS,D, M_RMS,D, N_RMS,D] 등
RMS_points = [];

%% 5) 각 Airspeed 경계점(U_b, V_b, W_b) + (phi, theta) 범위에서:
%    - 식 (10)으로 T 계산 (혹은 직접 T 수식)
%    - 에어로 모멘트 (식 (15)) = - M_A
%    - (16)~(17) 추력, (18) 각가속도, (19) load factor...
for iV = 1:length(U_b)
    % current airspeed (Body-C frame 차이는 여기서 단순화)
    V_C_body = [U_b(iV); V_b(iV); W_b(iV)];
    V_mag = norm(V_C_body);

    % Aerodynamic Forces(Trim기준)에서 X_A, Y_A, Z_A
    %  => 식 (10)을 풀어도 되지만 여기서는 phi,theta 주면 T를 바로 구하는 방식 사용
    for phi_dev = phi_range
        for theta_dev = theta_range
            
            % === (A) 에어로다이내믹 힘 ===
            [X_A, Y_A, Z_A] = AeroForces(V_C_body, phi_dev, theta_dev, param);
            
            % (10)에서 phi, theta를 안다고 치면, T는 z축 평형으로 결정 가능
            % (16)~(17) => T_req 계산
            term_ = - sin(theta_dev)*X_A ...
                     + cos(theta_dev)*sin(phi_dev)*Y_A ...
                     + cos(theta_dev)*cos(phi_dev)*Z_A ...
                     + param.m*param.g;  % + m*g
            den_  = cos(theta_dev)*cos(phi_dev);
            
            if abs(den_) < 1e-9
                % 수학적으로 cos(theta)*cos(phi)=0 이면 T_req 무한대 등등 비물리
                % => 스킵
                continue;
            end
            
            T_req = term_ / den_;
            
            % (19) load factor
            nz_ = T_req / (param.m * param.g);
            
            % === (B) 에어로다이내믹 모멘트 ===
            [L_A, M_A, N_A] = AeroMoments(V_C_body, phi_dev, theta_dev, param);
            
            % 식 (15): Disturbance 모멘트 = - M_A
            L_RMS_D = -L_A;
            M_RMS_D = -M_A;
            N_RMS_D = -N_A;
            
            % (18) 각가속도
            ang_acc = param.I \ [L_RMS_D; M_RMS_D; N_RMS_D];
            p_dot = ang_acc(1);
            q_dot = ang_acc(2);
            r_dot = ang_acc(3);
            
            % 결과 저장 ([nz, p_dot, q_dot, r_dot] 형태)
            RMS_points = [RMS_points; nz_, p_dot, q_dot, r_dot];
        end
    end
end

%% 6) 결과 시각화
figure;
plot(RMS_points(:,1), RMS_points(:,2), 'ro'); 
xlabel('n_z'); ylabel('p_{dot} [rad/s^2]');
title('RMS disturbance (n_z vs p_{dot})'); grid on;

figure;
plot(RMS_points(:,1), RMS_points(:,3), 'bx'); 
xlabel('n_z'); ylabel('q_{dot} [rad/s^2]');
title('RMS disturbance (n_z vs q_{dot})'); grid on;

figure;
plot(RMS_points(:,1), RMS_points(:,4), 'k^'); 
xlabel('n_z'); ylabel('r_{dot} [rad/s^2]');
title('RMS disturbance (n_z vs r_{dot})'); grid on;

disp('=== DONE ===');
disp(['RMS_points size: ', num2str(size(RMS_points,1)), ' x 4']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 에어로다이내믹 함수들

% 간단한 드래그 중심의 에어로 힘 (논문식 (3)를 "단순화" 가정)
function [X_A, Y_A, Z_A] = AeroForces(V_body, phi, theta, param)
    % V_body : [u; v; w] (m/s)
    % phi, theta : 자세 (rad)
    % param : 구조체 (rho, Sref, Cd 등)
    V_mag = norm(V_body);
    if V_mag < 1e-9
        X_A=0; Y_A=0; Z_A=0; return;
    end
    
    % 단순 드래그 크기
    D = 0.5 * param.rho * V_mag^2 * param.Sref * param.Cd;
    
    % 자세에 따라 효과가 달라진다고 단순 가정 (cos, sin 보정)
    X_A = -D * cos(phi)   * cos(theta);
    Y_A = -D * sin(phi);
    Z_A = -D * sin(theta) * cos(phi);
end

% 간단 선형 근사 에어로 모멘트 (논문식 (4) "유사"하게만 구성)
function [L_A, M_A, N_A] = AeroMoments(V_body, phi, theta, param)
    % param.kM = 0.05 (간단 스케일)
    % 실제로는 alpha, beta, lookup table 등에 따라 복잡
    V_mag = norm(V_body);
    
    % 예시: L_A = kM * V_mag * phi
    L_A = param.kM * V_mag * phi;
    M_A = param.kM * V_mag * theta;
    N_A = param.kM * V_mag * 0.5*(phi+theta);
end
