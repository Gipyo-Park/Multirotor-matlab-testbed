function attitude_NMPC
% =========================================================================
% NMPC 기반 자세 제어기 (attitude_MPC과 구조 완벽히 통일)
%
% 1. 초기화: 시뮬레이션 시작 시 한 번만 실행하여 NMPC에 필요한 모든
%    영구 변수(가중치, 옵션, 과거 상태/입력 등)를 계산하고 저장합니다.
% 2. 제어 루프: 매 제어 주기마다 다음을 반복합니다.
%    a. 현재 상태와 목표 각도를 읽어옵니다.
%    b. NLP(Nonlinear Programming) 문제를 구성합니다.
%    c. fmincon을 사용하여 최적의 제어 입력 변화량(DeltaU)을 풉니다.
%    d. 첫 번째 제어 입력을 실제 시스템에 적용합니다.
%    e. 다음 스텝을 위해 현재 상태와 입력을 저장합니다.
% =========================================================================

global Quad;

phi = Quad.phi;
theta = Quad.theta;
psi = Quad.psi;

p = Quad.p;
q = Quad.q;
r = Quad.r;

x = Quad.X;
y = Quad.Y;
z = Quad.Z;

persistent z_error_sum;
persistent phi_error_sum;
persistent theta_error_sum;
persistent psi_error_sum;


% --- NMPC 파라미터 및 계산된 행렬을 저장하기 위한 영구 변수 ---
persistent Qx Ru opts Delta_umax umin umax
persistent x_m_past u_past
persistent DeltaU_past

% NMPC 파라미터 정의
num_of_states = 6;
num_of_inputs = 3;
num_of_outputs = 6;
Nc = 15;
Np = 15;

%% 1. 초기화 (시뮬레이션 시작 시 한 번만 실행)
if Quad.init == 0
    % ---------------------------------------------------------------------
    % NMPC에 필요한 파라미터들을 한 번만 계산
    % (주: NMPC는 P, H와 같은 정적 예측 행렬을 사용하지 않음)
    % ---------------------------------------------------------------------
    
    % --- 비용 함수 가중치 정의 (튜닝 필요!) ---
    % 상태 순서: [φ, p, θ, q, ψ, r]
    Qx = diag([1000, 1, 1000, 1, 10, 1]); 
    % Ru: 입력(토크) '변화율'에 대한 가중치. 클수록 제어가 부드러워짐.
    Ru = diag([100, 100, 100]);

    % --- 제약 조건 정의 ---
    umax = [Quad.U2_max ; Quad.U3_max ; Quad.U4_max];
    umin = [Quad.U2_min ; Quad.U3_min ; Quad.U4_min];
    Delta_umax = 0.8 * umax;

    % fmincon 옵션
    opts = optimoptions('fmincon','Algorithm','active-set','Display','off', 'EnableFeasibilityMode', true);

    % 초기값 설정
    x_m_past = [Quad.phi; Quad.p; Quad.theta; Quad.q; Quad.psi; Quad.r];
    u_past = [Quad.U2; Quad.U3; Quad.U4];
    DeltaU_past = zeros(num_of_inputs * Nc, 1);

    z_error_sum = 0;
    phi_error_sum = 0;
    theta_error_sum = 0;
    psi_error_sum = 0;
end

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


%% 2. NMPC 제어 루프 (매 제어 주기마다 실행)

% a. 현재 상태 및 목표값 읽기
x_m_current = [Quad.phi; Quad.p; Quad.theta; Quad.q; Quad.psi; Quad.r];
r_current = [Quad.phi_des; 0; Quad.theta_des; 0; Quad.psi_des; 0];

% b. NLP 문제 구성
% 제약 조건 구성
lb = repmat(-Delta_umax, Nc, 1); % DeltaU의 하한
ub = repmat(Delta_umax, Nc, 1);  % DeltaU의 상한
constraint_fun = @(DeltaU) nonlinear_constraints(DeltaU, u_past, umin, umax, Nc);

% c. fmincon을 사용하여 최적의 DeltaU 계산
x0 = DeltaU_past; 
[DeltaU,fval,exitflag] = fmincon(@(du) objective_function(du, x_m_current, u_past, Np, Nc, r_current, Qx, Ru), x0, [], [], [], [], lb, ub, constraint_fun, opts);

% d. 첫 번째 제어 입력 적용
if exitflag > 0 % fmincon의 exitflag 대신 fval로 성공 여부 판단 (더 견고한 방법은 exitflag > 0)
    delta_u = DeltaU(1:num_of_inputs);
    u_current = u_past + delta_u;
    % DeltaU_past = [DeltaU(num_of_inputs+1:end); zeros(num_of_inputs,1)];  % <-- 시프트
    DeltaU_past = DeltaU;
    
    disp('NMPC solver success!');
else % 해를 찾지 못한 경우 (비상)
    u_current = u_past; % 이전 값 유지
    DeltaU_past = zeros(num_of_inputs * Nc, 1); % 실패 시 초기 추정치 초기화
    disp('NMPC solver failed!');
end

% 계산된 토크를 Quad 구조체에 할당
Quad.U2 = u_current(1);
Quad.U3 = u_current(2);
Quad.U4 = u_current(3);

% Saturation 처리
Quad.U2 = min(Quad.U2_max, max(Quad.U2_min, Quad.U2));
Quad.U3 = min(Quad.U3_max, max(Quad.U3_min, Quad.U3));
Quad.U4 = min(Quad.U4_max, max(Quad.U4_min, Quad.U4));

% e. 다음 스텝을 위해 현재 값을 과거 값으로 업데이트
x_m_past = x_m_current;
u_past = u_current;

end

% =========================================================================
% ## NMPC 핵심 로직: 목적함수, 제약조건함수, 예측 모델 ##
% =========================================================================

function J = objective_function(delta_u_sequence, x_current, u_past, Np, Nc, ref, Qx, Ru)
    global Quad;
    J = 0;
    x_pred = x_current;
    u_k = u_past;
    delta_u_sequence = reshape(delta_u_sequence, 3, Nc);
    for k = 1:Np
        if k <= Nc
            delta_u_k = delta_u_sequence(:, k);
            u_k = u_k + delta_u_k;
        end
        x_pred = prediction_model(x_pred, u_k, Quad.Ts);
        state_error = x_pred - ref;
        state_error(5) = atan2(sin(state_error(5)), cos(state_error(5)));
        if k <= Nc
            J = J + state_error'*Qx*state_error + delta_u_k'*Ru*delta_u_k;
        else
            J = J + state_error'*Qx*state_error;
        end
    end
end

function [c, ceq] = nonlinear_constraints(DeltaU, u_past, umin, umax, Nc)
    u_sequence = cumsum([u_past, reshape(DeltaU, 3, Nc)], 2);
    u_sequence = u_sequence(:, 2:end);
    c_max = u_sequence - repmat(umax, 1, Nc);
    c_min = repmat(umin, 1, Nc) - u_sequence;
    c = [c_max(:); c_min(:)];
    ceq = [];
end

function x_next = prediction_model(x_k, u_k, Ts)
    k1 = dynamics_model(x_k, u_k);
    k2 = dynamics_model(x_k + 0.5 * Ts * k1, u_k);
    k3 = dynamics_model(x_k + 0.5 * Ts * k2, u_k);
    k4 = dynamics_model(x_k + Ts * k3, u_k);
    x_next = x_k + (Ts / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
end

function x_dot = dynamics_model(x_k, u_k)
    global Quad;
    phi=x_k(1); p=x_k(2); theta=x_k(3); q=x_k(4); psi=x_k(5); r=x_k(6);
    U2=u_k(1); U3=u_k(2); U4=u_k(3);
    p_dot=(q*r*(Quad.Jy-Quad.Jz)+U2)/Quad.Jx;
    q_dot=(p*r*(Quad.Jz-Quad.Jx)+U3)/Quad.Jy;
    r_dot=(p*q*(Quad.Jx-Quad.Jy)+U4)/Quad.Jz;
    phi_dot=p+sin(phi)*tan(theta)*q+cos(phi)*tan(theta)*r;
    theta_dot=cos(phi)*q-sin(phi)*r;
    psi_dot=sin(phi)/cos(theta)*q+cos(phi)/cos(theta)*r;
    x_dot=[phi_dot;p_dot;theta_dot;q_dot;psi_dot;r_dot];
end