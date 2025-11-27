% clear; clc;close all;

number_of_rotors = 4; % quad: 4 / octo: 8

base_link_mass = 19.0939;
base_link_Ixx = 7.46;
base_link_Iyy = 1.52;
base_link_Izz = 8.88;

imu_link_mass = 0.015;
imu_link_Ixx = 0.00001;
imu_link_Iyy = 0.00001;
imu_link_Izz = 0.00001;

% 각 rotor의 값들을 배열에 저장
masses = 0.085251;
Ixx_values = 0.00001545;
Iyy_values = 0.00073574;
Izz_values = 0.00074861;

% 구조체 배열을 생성하여 변수들을 저장
rotors = struct('mass', cell(1, 8), 'Ixx', cell(1, 8), 'Iyy', cell(1, 8), 'Izz', cell(1, 8));

for i = 1:number_of_rotors
    rotors(i).mass = masses;
    rotors(i).Ixx = Ixx_values;
    rotors(i).Iyy = Iyy_values;
    rotors(i).Izz = Izz_values;
end

% Sum rotor mass
total_rotor_masses = [rotors.mass];
total_rotor_masses = sum(total_rotor_masses);
% fprintf('total_rotor_masses = %f \n', total_rotor_masses);
% fprintf('rotor_1_mass = %f',rotors(1).mass);


% Sum rotor Ixx
total_rotor_Ixx = [rotors.Ixx];
total_rotor_Ixx = sum(total_rotor_Ixx);

% Sum rotor Iyy
total_rotor_Iyy = [rotors.Iyy];
total_rotor_Iyy = sum(total_rotor_Iyy);

% Sum rotor Izz
total_rotor_Izz = [rotors.Izz];
total_rotor_Izz = sum(total_rotor_Izz);


gps0_mass = 0.01;
gps0_Ixx = 0.0000021733;
gps0_Iyy = 0.0000021733;
gps0_Izz = 0.00000018;

% Sum drone mass
total_mass = base_link_mass + imu_link_mass + total_rotor_masses + gps0_mass;
total_mass = round(total_mass);
fprintf('total_mass = %f \n', total_mass);

% Sum drone Ixx
total_Ixx = base_link_Ixx + imu_link_Ixx + total_rotor_Ixx + gps0_Ixx;
total_Ixx = round(total_Ixx, 2);
fprintf('total_Ixx = %f \n', total_Ixx);

% Sum drone Iyy
total_Iyy = base_link_Iyy + imu_link_Iyy + total_rotor_Iyy + gps0_Iyy;
total_Iyy = round(total_Iyy, 2);
fprintf('total_Iyy = %f \n', total_Iyy);

% Sum drone Izz
total_Izz = base_link_Izz + imu_link_Izz + total_rotor_Izz + gps0_Izz;
total_Izz = round(total_Izz, 2);
fprintf('total_Izz = %f \n', total_Izz);


%% Define parameters

% 변수 정의
syms x y z u v w phi theta psi p q r
syms x_dot y_dot z_dot u_dot v_dot w_dot phi_dot theta_dot psi_dot p_dot q_dot r_dot
syms fx fy fz mx my mz U Ixx Iyy Izz
syms m g fw fwx fwy fwz ft tau_x tau_y tau_z tau_wx tau_wy tau_wz


% 간단히 하기 위해 cos, sin을 줄여서 씁니다.
% c = @(x) cos(x);
% s = @(x) sin(x);
% t = @(x) tan(x);

% m = total_mass; % mass of the octocopter (kg)
m = 1.4;
g = 9.81; % gravitational acceleration (m/s^2)
I_v = diag([Ixx, Iyy, Izz]); % inertia matrix (kg*m^2) % 수식확인 할때 사용
% I_v = diag([total_Ixx, total_Iyy, total_Izz]); % inertia matrix (kg*m^2)
% Ixx = total_Ixx;
% Iyy = total_Iyy;
% Izz = total_Izz;
Ixx = 0.05;
Iyy = 0.05;
Izz = 0.24;

%{
octocopter figure

1cw        2ccw
3ccw       4cw
      ㅁ
5cw        6ccw
7ccw       8cw

%}


%% Six degree of the freedom model 
% Kinematics 

% Translation
% Pos_dot = [x_dot;y_dot;z_dot]; % translational velocity in earth frame
VB = [u;v;w]; % translational velocity in body frame
Pos_dot = RotMat(phi, theta, psi, 5) * VB; %translational velocity in body frame rotates velocity in earth frame % Inertial frame

% Rotational
% OMEGA = [phi_dot;theta_dot;psi_dot]; % rotational(angular) velocity in earth frame
OMEGAB = [p;q;r];  % rotational(angular) velocity in body frame
OMEGA = RotMat(phi, theta, psi, 7) * OMEGAB; % rotational(angular) velocity in body frame rotates angular velocity in earth frame % Diff Vehicle frames




% % 방정식 정의 % kinematic model은 쿼드나 옥토나 같다.
% x_dot = w*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - v*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + u*cos(psi)*cos(theta);
% y_dot = v*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - w*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + u*cos(theta)*sin(psi);
% z_dot = w*cos(phi)*cos(theta) - u*sin(theta) + v*cos(theta)*sin(phi);
% phi_dot = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta);
% theta_dot = q*cos(phi) - r*sin(phi);
% psi_dot = r*cos(phi)/cos(theta) + q*sin(phi)/cos(theta);

% VB = [u_dot;v_dot;w_dot]; % linear acceleration in body frame
% OMEGAB = [p_dot;q_dot;r_dot]; % angular acceleration in body frame



gravity_force = RotMat(phi, theta, psi, 8) * [0;0;m*g];
fw = [fwx;fwy;fwz];
thrust_force = ft * [0;0;1];

% 총 force (body frame)
FB = gravity_force - thrust_force + fw;


% Rotor + Wind torque
tau_rotor  = [tau_x; tau_y; tau_z];
tau_wind = [tau_wx; tau_wy; tau_wz];
gyroscopic = [0;0;0];

% 총 torque (body frame)
MB = tau_rotor - gyroscopic + tau_wind;


% Translation Dynamics
VB = - cross(OMEGAB, VB) + (1/m) * FB ; % body frame

% Rotation Dynamics
OMEGAB = inv(I_v) * ( MB - cross(OMEGAB, I_v * OMEGAB) ); % body frame



X_dot = [Pos_dot; VB; OMEGA; OMEGAB];
disp('X_dot = ')
disp(X_dot)

% 위치 가속도
x_ddot = -(ft/m) * ( cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi) );
y_ddot = -(ft/m) * ( cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi) );
z_ddot = g - (ft/m)*cos(phi)*cos(theta);

% 회전 가속도
phi_ddot   = ((Iyy - Izz)/Ixx)*r*q + tau_x/Ixx;
theta_ddot = ((Izz - Ixx)/Iyy)*p*r + tau_y/Iyy;
psi_ddot   = ((Ixx - Iyy)/Izz)*p*q + tau_z/Izz;

% 상태 미분 벡터
df_dx = jacobian(X_dot,[x;y;z;u;v;w;phi;theta;psi;p;q;r]);
df_du = jacobian(X_dot,[tau_x;tau_y;tau_z;ft]);
%% 여기서 부터는 LQR 가정 으로 인해 phi_dot ≈ p , theta_dot ≈ q , psi_dot ≈ r

phi_ddot   = ((Iyy - Izz)/Ixx)*psi_dot*theta_dot + tau_x/Ixx;
theta_ddot = ((Izz - Ixx)/Iyy)*phi_dot*psi_dot + tau_y/Iyy;
psi_ddot   = ((Ixx - Iyy)/Izz)*phi_dot*theta_dot + tau_z/Izz;


% LQR에 활용 할 상태 미분 벡터
global Quad;

X_dot = [phi_dot; phi_ddot; theta_dot; theta_ddot; psi_dot; psi_ddot];
LQR_df_dx = jacobian(X_dot,[phi; phi_dot; theta; theta_dot; psi; psi_dot]); % phi_dot ≈ p , theta_dot ≈ q , psi_dot ≈ r 
LQR_df_du = jacobian(X_dot,[tau_x; tau_y; tau_z]);

% 평형점에서 평가 (phi=0, phi_dot=0, theta=0, theta_dot=0, psi=0, psi_dot=0)
A_eq = double(subs(LQR_df_dx, {phi,phi_dot,theta,theta_dot,psi,psi_dot}, {0, 0, 0, 0, 0, 0}));
B_eq = double(subs(LQR_df_du, {phi,phi_dot,theta,theta_dot,psi,psi_dot}, {0, 0, 0, 0, 0, 0}));

% LQR 가중치 설정
% 상태 순서: [φ, φ̇, θ, θ̇, ψ, ψ̇]
Q = diag([1000,  1,   1000,  1,   10,  1]);  
R = diag([10,   10,    10]);  

% LQR 이득 계산
% K_LQR_attitude = lqr(A_eq, B_eq, Q, R);       % LQR 이득
K_LQR_attitude = lqrd(A_eq, B_eq, Q, R, 0.01);       % LQR 이득
Quad.K_LQR_attitude = K_LQR_attitude;

%% LQI
C = [1 0 0 0 0 0;    % phi
     0 0 1 0 0 0;    % theta
     0 0 0 0 1 0];   % psi

A_LQI = [A_eq, zeros(6,3);
         C, zeros(3,3)];
B_LQI = [B_eq;
         zeros(3,3)];
Q_LQI = diag([1000,  1,   1000,  1,   10,  1,  0.1, 0.1, 0.1]);  % 상태 + 적분 오차
R_LQI = diag([10, 10, 10]);

% LQI 이득 계산
K_LQI_attitude = lqrd(A_LQI, B_LQI, Q_LQI, R_LQI, 0.01);
Quad.K_LQI_attitude = K_LQI_attitude;

disp('df_dx = ')
disp(df_dx)
disp('df_du = ')
disp(df_du)

disp('size of df_dx = ')
disp(size(df_dx))
disp('size of df_du = ')
disp(size(df_du))

%%%%%%%%%%%%%%%%
disp('delft_df_dx = ')
disp(LQR_df_dx)
disp('delft_df_du = ')
disp(LQR_df_du)

disp('size of LQR_df_dx = ')
disp(size(LQR_df_dx))
disp('size of LQR_df_du = ')
disp(size(LQR_df_du))




%% 확인 방법 (LQR일 경우)
A_cl = A_eq - B_eq * K_LQR_attitude;
eigvals = eig(A_cl);
disp('Eigenvalues of closed-loop A matrix (LQR):');
disp(eigvals);

if all(real(eigvals) < 0)
    disp('✅ LQR system is stable.');
else
    disp('❌ LQR system is unstable.');
end
[p, wn, zeta] = damp(ss(A_cl, B_eq, eye(size(A_cl)), 0));  % LQR 기준
T = table(p, zeta, wn);
disp(T);

figure();
sys_cl = ss(A_cl, B_eq, eye(6), 0);  % 상태 출력
step(sys_cl);
title('Closed-loop step Response (LQR)');

figure();
x0 = [0.1; 1; 0.1; 1; 0.1; 1];  % 초기 각도 오차 예시
initial(sys_cl, x0);
title('Closed-loop initial Response (LQR)');


%% 확인 방법 (LQI일 경우)
A_LQI_cl = A_LQI - B_LQI * K_LQI_attitude;
eigvals_LQI = eig(A_LQI_cl);
disp('Eigenvalues of closed-loop A matrix (LQI):');
disp(eigvals_LQI);

if all(real(eigvals_LQI) < 0)
    disp('✅ LQI system is stable.');
else
    disp('❌ LQI system is unstable.');
end


[p, wn, zeta] = damp(ss(A_LQI_cl, B_LQI, eye(size(A_LQI_cl)), 0));  % LQR 기준
T = table(p, zeta, wn);
disp(T);

figure();
sys_LQI_cl = ss(A_LQI_cl, B_LQI, eye(9), 0);  % 상태 출력
step(sys_LQI_cl);
title('Closed-loop step Response (LQI)');

figure();
x0 = [0.1; 0; 0.1; 0; 0.1; 0];  % 초기 각도 오차 예시
initial(sys_cl, x0);
title('Closed-loop initial Response (LQI)');

%% MPC /  선형화하기위해 가정으로 인해 phi_dot ≈ p , theta_dot ≈ q , psi_dot ≈ r

% X = [phi; phi_dot; theta; theta_dot; psi; psi_dot]
% u = [tau_x; tau_y; tau_z]
% y = [phi; theta; psi]

X_dot = [phi_dot; phi_ddot; theta_dot; theta_ddot; psi_dot; psi_ddot];
Am = jacobian(X_dot,[phi; phi_dot; theta; theta_dot; psi; psi_dot]); % phi_dot ≈ p , theta_dot ≈ q , psi_dot ≈ r 
Bm = jacobian(X_dot,[tau_x; tau_y; tau_z]);

% 평형점에서 평가 (phi=0, phi_dot=0, theta=0, theta_dot=0, psi=0, psi_dot=0)
Quad.Am = double(subs(Am, {phi,phi_dot,theta,theta_dot,psi,psi_dot}, {0, 0, 0, 0, 0, 0}));
Quad.Bm = double(subs(Bm, {phi,phi_dot,theta,theta_dot,psi,psi_dot}, {0, 0, 0, 0, 0, 0}));

% Quad.Cm = [1 0 0 0 0 0;
%       0 0 1 0 0 0;
%       0 0 0 0 1 0];
Quad.Cm = eye(6);
% Quad.Dm = zeros(3,3);
Quad.Dm = zeros(6, 3);

% [Ad,Bd,Cd,Dd] = c2dm(Am,Bm,Cm,Dm,0.01); % Converting from Continuous to Discrete Time

% [A_aug,B_aug,C_aug] = augment_mimo(Ad, Bd, Cd, num_of_states, num_of_inputs, num_of_outputs);
% [P, H] = calculate_prediction_matrices(A_aug, B_aug, C_aug, Np, Nc); % Y = P*X(k) + H*U(k)
% 
% 
% umax = [Quad.U2_max ; Quad.U3_max ; Quad.U4_max];
% umin = [Quad.U2_min ; Quad.U3_min ; Quad.U4_min];
% Delta_umax = 0.6*umax;
% 
% [CC, dd, dupast] = constraints_mimo(Delta_umax, umax, umin, num_of_inputs, Nc);