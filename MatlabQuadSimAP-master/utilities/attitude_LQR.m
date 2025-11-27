% Wil Selby
% Washington, DC
% May 30, 2015

% This function implements a Proportional Integral Derivative Controller
% (LQR) for the quadrotor. A lower level controller takes those inputs and 
% controls the error between the deisred and actual Euler angles. 

function attitude_LQR

persistent z_error_sum;
persistent phi_error_sum;
persistent theta_error_sum;
persistent psi_error_sum;

global Quad

% initialize persistent variables at beginning of simulation
if Quad.init==0
    z_error_sum = 0;
    phi_error_sum = 0;
    theta_error_sum = 0;
    psi_error_sum = 0;
end

% % Measurement Model
% if(Quad.ground_truth)
%     phi = Quad.phi;
%     theta = Quad.theta;
%     psi = Quad.psi;
% end
% 
% if(Quad.sensor_unfiltered)
%     phi = Quad.phi_meas;
%     theta = Quad.theta_meas;
%     psi = Quad.psi_meas;
% end
% 
% if(Quad.sensor_kf)
%     phi = Quad.phi;
%     theta = Quad.theta;
%     psi = Quad.psi;
% end

phi = Quad.phi;
theta = Quad.theta;
psi = Quad.psi;

p = Quad.p;
q = Quad.q;
r = Quad.r;

x = Quad.X;
y = Quad.Y;
z = Quad.Z;


%% Z Position PID Controller/Altitude Controller

% Rotate Desired Position from GF to BF (Z axis rotation only)
[Quad.X_des,Quad.Y_des,Quad.Z_des] = rotateGFtoBF(Quad.X_des_GF,Quad.Y_des_GF,Quad.Z_des_GF,0*phi,0*theta,psi);

% Rotate Current Position from GF to BF
[Quad.X_BF,Quad.Y_BF,Quad.Z_BF] = rotateGFtoBF(x,y,z,phi,theta,psi);

% Rotate Current Velocity from GF to BF
[Quad.X_BF_dot,Quad.Y_BF_dot,Quad.Z_BF_dot] = rotateGFtoBF(Quad.X_dot,Quad.Y_dot,Quad.Z_dot,phi,theta,psi);

z_error = Quad.Z_des_GF-Quad.Z_BF;
if(abs(z_error) < Quad.Z_KI_lim)
    z_error_sum = z_error_sum + z_error;
end
cp = Quad.Z_KP*z_error;         %Proportional term
ci = Quad.Z_KI*Quad.Ts*z_error_sum; %Integral term
ci = min(Quad.U1_max, max(Quad.U1_min, ci));    %Saturate ci
cd = Quad.Z_KD*Quad.Z_dot;                  %Derivative term
Quad.U1 = -(cp + ci + cd)/(cos(theta)*cos(phi)) + (Quad.m * Quad.g)/(cos(theta)*cos(phi));   %Negative since Thurst and Z inversely related
Quad.U1 = min(Quad.U1_max, max(Quad.U1_min, Quad.U1));


%% Attitude Controller

desired_attitude = [Quad.phi_des; Quad.theta_des; Quad.psi_des];
% phi error
% phi_error = Quad.phi_des - phi;
phi_error = phi - Quad.phi_des;
% phi_error = phi;
% phi_error = -Quad.phi_des;
% phi_error = 0;
if(abs(phi_error) < Quad.phi_KI_lim)
    phi_error_sum = phi_error_sum + phi_error;
end
% phi_dot error
% phi_dot_error = Quad.phi_dot; % phi_dot ≈ p (근사) 또는 0 으로 두기
% phi_dot_error = 0 - Quad.p;  % or desired_phi_dot - actual_p
phi_dot_error = p - 0;
% phi_dot_error = 0;
% phi_dot_error = Quad.phi_dot;

% theta error
% theta_error = Quad.theta_des - theta;
theta_error = theta - Quad.theta_des;
% theta_error = theta;
% theta_error = -Quad.theta_des;
% theta_error = 0;
if(abs(theta_error) < Quad.theta_KI_lim)
    theta_error_sum = theta_error_sum + theta_error;
end
% theta_dot error
% theta_dot_error = Quad.theta_dot; % theta_dot ≈ q (근사) 또는 0 으로 두기
% theta_dot_error = 0 - Quad.q;  % or desired_theta_dot - actual_q
theta_dot_error = q - 0;
% theta_dot_error = 0;
% theta_dot_error = Quad.theta_dot;

% psi error
% psi_error = Quad.psi_des - psi;
psi_error = psi - Quad.psi_des;
% psi_error = psi;
% psi_error = -Quad.psi_des;
% psi_error = 0;
if(abs(psi_error) < Quad.psi_KI_lim)
    psi_error_sum = psi_error_sum + psi_error;
end
% psi_dot error
% psi_dot_error = Quad.psi_dot; % psi_dot ≈ r (근사) 또는 0 으로 두기 
% psi_dot_error = 0 - Quad.r;  % or desired_psi_dot - actual_r
psi_dot_error = r - 0;
% psi_dot_error = 0;
% psi_dot_error = Quad.psi_dot;

x_state = [phi_error; phi_dot_error; theta_error; theta_dot_error; psi_error; psi_dot_error];

% LQR 이득 적용 (3x6 * 6x1 = 3x1)
tau_LQR = -Quad.K_LQR_attitude * x_state;

% 각 토크를 원하는 각속도로 매핑 (모터 믹싱을 따로 두는 구조를 유지할 경우)
Quad.U2 = tau_LQR(1);
Quad.U3 = tau_LQR(2);
Quad.U4 = tau_LQR(3);

% Saturation 처리
Quad.U2 = min(Quad.U2_max, max(Quad.U2_min, Quad.U2));
Quad.U3 = min(Quad.U3_max, max(Quad.U3_min, Quad.U3));
Quad.U4 = min(Quad.U4_max, max(Quad.U4_min, Quad.U4));




end
















