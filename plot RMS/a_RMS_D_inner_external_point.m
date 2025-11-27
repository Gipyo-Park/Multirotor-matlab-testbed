%% 전체 Airspeed Envelope (내부+경계) 모든 점을 고려하여 RMS 계산
close all; clear; clc;

%% 1) 기본 파라미터 설정
param.m    = 20;                 % 질량 (kg)
param.g    = 9.81;               % 중력가속도 (m/s^2)
param.I    = diag([0.2, 0.02, 0.04]);  % 관성행렬 (kg·m^2)
param.rho  = 1.225;              % 공기 밀도 (kg/m^3)
param.Sref = 0.3;                % 참조 면적 (m^2)
param.Cd   = 1.0;                % 단순 드래그 계수
param.kM   = 0.05;               % 에어로 모멘트 스케일링(임의 예시)

% Airspeed Envelope: (u, v, w) 범위 미션중에 내야할 속도 범위
uC_min = -8;  uC_max = 8;  n_u = 3;
vC_min = -8;  vC_max = 8;  n_v = 3;
wC_min = -3;  wC_max = 3;  n_w = 3;

u_vals = linspace(uC_min, uC_max, n_u);
v_vals = linspace(vC_min, vC_max, n_v);
w_vals = linspace(wC_min, wC_max, n_w);

%% 2) 전체 (u, v, w) 3D 격자를 생성
[U, V, W] = ndgrid(u_vals, v_vals, w_vals);

%% 3) 자세 overshoot 범위 설정 (논문 식 (11)~(14))
% 미션중에 필요한 자세 설정
phi_trim   = 0;  % 예: roll trim
theta_trim = 0;  % 예: pitch trim

phi_min = deg2rad(-20);
phi_max = deg2rad(20);
theta_min = deg2rad(-20);
theta_max = deg2rad(20);

phi_overshoot   = deg2rad(5);
theta_overshoot = deg2rad(5);

phi_lower   = min(phi_min,   phi_trim - phi_overshoot);
phi_upper   = max(phi_max,   phi_trim + phi_overshoot);
theta_lower = min(theta_min, theta_trim - theta_overshoot);
theta_upper = max(theta_max, theta_trim + theta_overshoot);

n_phi   = 5;
n_theta = 5;

phi_range   = linspace(phi_lower,   phi_upper,   n_phi);
theta_range = linspace(theta_lower, theta_upper, n_theta);

%% 4) RMS 포인트 저장용
% [n_z, p_dot, q_dot, r_dot]
RMS_points = [];

%% 5) 전체 영역 (u, v, w)와 overshoot (phi, theta)에 대해 반복
for i = 1:numel(U)
    % 현재 속도 벡터
    V_C_body = [U(i); V(i); W(i)];
    V_mag = norm(V_C_body);
    
    % phi, theta 변화를 순회
    for phi_dev = phi_range
        for theta_dev = theta_range
            
            % === (A) 에어로힘 계산
            [X_A, Y_A, Z_A] = AeroForces(V_C_body, phi_dev, theta_dev, param);
            
            % (16)~(17) Z축 평형으로부터 T_req 계산
            term_ = -sin(theta_dev)*X_A ...
                     + cos(theta_dev)*sin(phi_dev)*Y_A ...
                     + cos(theta_dev)*cos(phi_dev)*Z_A ...
                     + param.m * param.g;
            den_ = cos(theta_dev)*cos(phi_dev);
            
            if abs(den_)<1e-9
                % cos(theta)*cos(phi) == 0 => 물리적으로 추력 무한 등 비현실
                continue;
            end
            T_req = term_ / den_;
            
            % (19) load factor
            n_z = T_req / (param.m * param.g);
            
            % === (B) 에어로모멘트 계산
            [L_A, M_A, N_A] = AeroMoments(V_C_body, phi_dev, theta_dev, param);
            
            % (15) disturbance moment = - M_A
            L_dist = -L_A;
            M_dist = -M_A;
            N_dist = -N_A;
            
            % (18) 각가속도
            ang_acc = param.I \ [L_dist; M_dist; N_dist];
            p_dot = ang_acc(1);
            q_dot = ang_acc(2);
            r_dot = ang_acc(3);
            
            % 한 포인트 저장
            RMS_points = [RMS_points; n_z, p_dot, q_dot, r_dot];
        end
    end
end

%% 6) 결과 시각화
figure;
plot(RMS_points(:,1), RMS_points(:,2), 'ro');
xlabel('n_z'); ylabel('p_{dot} [rad/s^2]');
title('RMS: n_z vs p_{dot} (전체 내부 포함)'); grid on;

figure;
plot(RMS_points(:,1), RMS_points(:,3), 'bx');
xlabel('n_z'); ylabel('q_{dot} [rad/s^2]');
title('RMS: n_z vs q_{dot}'); grid on;

figure;
plot(RMS_points(:,1), RMS_points(:,4), 'k^');
xlabel('n_z'); ylabel('r_{dot} [rad/s^2]');
title('RMS: n_z vs r_{dot}'); grid on;

disp('=== DONE ===');
disp(['RMS_points.size = ', num2str(size(RMS_points,1)), ' x 4']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 간단한 에어로힘 모델 (논문 (3) 단순화)
function [X_A, Y_A, Z_A] = AeroForces(V_body, phi, theta, param)
    V_mag = norm(V_body);
    if V_mag<1e-9
        X_A=0; Y_A=0; Z_A=0; return;
    end
    D = 0.5 * param.rho * V_mag^2 * param.Sref * param.Cd;
    
    % 자세 보정 (예시)
    X_A = -D * cos(phi)*cos(theta);
    Y_A = -D * sin(phi);
    Z_A = -D * sin(theta)*cos(phi);
end

%% 간단한 에어로모멘트 모델 (논문 (4) 단순화)
function [L_A, M_A, N_A] = AeroMoments(V_body, phi, theta, param)
    V_mag = norm(V_body);
    % 임의 선형 스케일링
    L_A = param.kM * V_mag * phi;
    M_A = param.kM * V_mag * theta;
    N_A = param.kM * V_mag * 0.5*(phi+theta);
end
