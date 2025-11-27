% Wil Selby
% Washington, DC
% May 30, 2015

% This function implements a Proportional Integral Derivative Controller
% (LQR) for the quadrotor. A lower level controller takes those inputs and 
% controls the error between the deisred and actual Euler angles. 

function full_LQR

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

phi = Quad.phi;
theta = Quad.theta;
psi = Quad.psi;

%% Attitude Controller



% Rotate Desired Position from GF to BF (Z axis rotation only)
[Quad.X_des,Quad.Y_des,Quad.Z_des] = rotateGFtoBF(Quad.X_des_GF,Quad.Y_des_GF,Quad.Z_des_GF,0*phi,0*theta,psi);

% Rotate Current Position from GF to BF
[Quad.X_BF,Quad.Y_BF,Quad.Z_BF] = rotateGFtoBF(x,y,z,phi,theta,psi);

% Rotate Current Velocity from GF to BF
[Quad.X_BF_dot,Quad.Y_BF_dot,Quad.Z_BF_dot] = rotateGFtoBF(Quad.X_dot,Quad.Y_dot,Quad.Z_dot,phi,theta,psi);



phi_error = phi;
phi_dot_error = p - 0;
theta_error = theta;
theta_dot_error = q - 0;
psi_error = psi;
psi_dot_error = r - 0;
x_error = Quad.X_des - Quad.X_BF;
x_dot_error = Quad.X_dot;
y_error = Quad.Y_des - Quad.Y_BF;
y_dot_error = Quad.Y_dot;
z_error = Quad.Z_des_GF-Quad.Z_BF;
z_dot_error = Quad.Z_dot;




x_state = [phi_error; phi_dot_error; theta_error; theta_dot_error; psi_error; psi_dot_error; x_error; x_dot_error; y_error; y_dot_error; z_error; z_dot_error];

% LQR 이득 적용 
tau_LQR = -Quad.K_LQR_attitude_full * x_state;

% 각 토크를 원하는 각속도로 매핑 (모터 믹싱을 따로 두는 구조를 유지할 경우)
Quad.U1 = tau_LQR(1);
Quad.U2 = tau_LQR(2);
Quad.U3 = tau_LQR(3);
Quad.U4 = tau_LQR(4);

% Saturation 처리
Quad.U1 = min(Quad.U1_max, max(Quad.U1_min, Quad.U1));
Quad.U2 = min(Quad.U2_max, max(Quad.U2_min, Quad.U2));
Quad.U3 = min(Quad.U3_max, max(Quad.U3_min, Quad.U3));
Quad.U4 = min(Quad.U4_max, max(Quad.U4_min, Quad.U4));



end
















