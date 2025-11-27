function attitude_MPC
% =========================================================================
% MPC 기반 자세 제어기 (attitude_LQR 대체용)
%
% 1. 초기화: 시뮬레이션 시작 시 한 번만 실행하여 MPC에 필요한 모든 행렬
%    (증강 모델, 예측 행렬, 제약 조건 행렬 등)을 계산하고 저장합니다.
% 2. 제어 루프: 매 제어 주기마다 다음을 반복합니다.
%    a. 현재 상태와 목표 각도를 읽어옵니다.
%    b. QP(Quadratic Programming) 문제를 구성합니다.
%    c. quadprog를 사용하여 최적의 제어 입력 변화량(DeltaU)을 풉니다.
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


% --- MPC 파라미터 및 계산된 행렬을 저장하기 위한 영구 변수 ---
persistent P H CC dd dupast A_aug C_aug
persistent Qx Ru opts
persistent x_m_past u_past
persistent DeltaU_past

% MPC 파라미터 정의
num_of_states = 6; % Number of states
num_of_inputs = 3; % Number of inputs
num_of_outputs = 6; % Number of outputs
Nc = 15; % control horizon
Np = 15; % prediction horizon

%% 1. 초기화 (시뮬레이션 시작 시 한 번만 실행)
if Quad.init == 0
    % ---------------------------------------------------------------------
    % 제공된 코드를 사용하여 MPC 행렬을 한 번만 계산
    % ---------------------------------------------------------------------
    

 
    % 비선형 동역학을 여기에 정의해야 합니다. 
    % 예시: phi_ddot = (1/Quad.Jx) * tau_x; ...
    % 실제로는 더 복잡한 비선형 식이 필요합니다. 
    % 여기서는 개념적인 선형 모델을 바로 사용하겠습니다.
    % Quad.Am = [0 1 0 0 0 0;
    %       0 0 0 0 0 0;
    %       0 0 0 1 0 0;
    %       0 0 0 0 0 0;
    %       0 0 0 0 0 1;
    %       0 0 0 0 0 0];
    % Quad.Bm = [0 0 0;
    %       1/Quad.Jx 0 0;
    %       0 0 0;
    %       0 1/Quad.Jy 0;
    %       0 0 0;
    %       0 0 1/Quad.Jz];
    % 
    % Quad.Cm = [0 1 0 0 0 0;
    %       0 0 0 1 0 0;
    %       0 0 0 0 0 1];
    % Quad.Dm = zeros(3,3);
    
    % model_dynamics 에서 Quad.Am Quad.Bm Quad.Cm Quad.Dm 지정해준다
    
    
    % 연속->이산 시간 변환 및 증강 모델 생성
    [Ad,Bd,Cd,~] = c2dm(Quad.Am, Quad.Bm, Quad.Cm, Quad.Dm,Quad.Ts);
    [A_aug,B_aug,C_aug] = augment_mimo(Ad, Bd, Cd, num_of_states, num_of_outputs);

    % 예측 행렬 계산
    [P, H] = calculate_prediction_matrices(A_aug, B_aug, C_aug, Np, Nc); % Y = P*x_aug + H*DeltaU 관계식을 만듭니다

    % 제약 조건 행렬 계산
    umax = [Quad.U2_max ; Quad.U3_max ; Quad.U4_max];
    umin = [Quad.U2_min ; Quad.U3_min ; Quad.U4_min];
    Delta_umax = 0.8 * umax;
    [CC, dd, dupast] = constraints_mimo(Delta_umax, umax, umin, num_of_inputs, Nc); 
    % 제어 입력(토크)의 크기와 변화율에 대한 물리적 제약 조건을 quadprog가 이해할 수 있는 CC*DeltaU <= dd + dupast*u_past 형태의 행렬로 변환

    % --- 비용 함수 가중치 정의 (튜닝 필요!) ---
    % Qx: 출력(각도) 오차에 대한 가중치. 클수록 목표 각도를 더 정확히 추종.
    % 상태 순서: [φ, φ̇, θ, θ̇, ψ, ψ̇]
    Qx = diag([1000, 1, 1000, 1, 10, 1]); 
    % Ru: 입력(토크) '변화율'에 대한 가중치. 클수록 제어가 부드러워짐.
    Ru = diag([10, 10, 10]);

    % quadprog 옵션
    opts = optimoptions('quadprog','Algorithm','active-set','Display','off');
    % opts = optimoptions('quadprog','Display','off');


    % 초기값 설정
    x_m_past = [Quad.phi; Quad.phi_dot; Quad.theta; Quad.theta_dot; Quad.psi; Quad.psi_dot];
    % x_m_past = [Quad.phi; Quad.p; Quad.theta; Quad.q; Quad.psi; Quad.r];
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

% z_error = Quad.Z_des_GF-Quad.Z_BF;
z_error = Quad.Z_des_GF-z;
if(abs(z_error) < Quad.Z_KI_lim)
    z_error_sum = z_error_sum + z_error;
end
cp = Quad.Z_KP*z_error;         %Proportional term
ci = Quad.Z_KI*Quad.Ts*z_error_sum; %Integral term
ci = min(Quad.U1_max, max(Quad.U1_min, ci));    %Saturate ci
cd = Quad.Z_KD*Quad.Z_dot;                  %Derivative term
Quad.U1 = -(cp + ci + cd)/(cos(theta)*cos(phi)) + (Quad.m * Quad.g)/(cos(theta)*cos(phi));   %Negative since Thurst and Z inversely related
Quad.U1 = min(Quad.U1_max, max(Quad.U1_min, Quad.U1));


%% 2. MPC 제어 루프 (매 제어 주기마다 실행)

% a. 현재 상태 및 목표값 읽기
x_m_current = [Quad.phi; Quad.phi_dot; Quad.theta; Quad.theta_dot; Quad.psi; Quad.psi_dot];
% x_m_current = [Quad.phi; Quad.p; Quad.theta; Quad.q; Quad.psi; Quad.r];
r_current = [Quad.phi_des; 0; Quad.theta_des; 0; Quad.psi_des; 0];

% r_current = [phi - Quad.phi_des; Quad.phi_dot - 0; theta - Quad.theta_des; Quad.theta_dot - 0; psi - Quad.psi_des; Quad.psi_dot - 0];


% b. QP 문제 구성
% 증강 상태 벡터 구성
delta_xm = x_m_current - x_m_past;
y_current = Quad.Cm * x_m_current;
x_aug = [delta_xm; y_current]; % 상태 변화량과 현재 출력을 합쳐 현재의 증강 상태 벡터 x_aug를 만듭니다.

% 비용 함수 구성: min 0.5*x'*H*x + f'*x
Q_bar = kron(eye(Np), Qx);
R_bar = kron(eye(Nc), Ru);

H_qp = 2 * (H' *Q_bar* H + R_bar); % E (논문) ↔ H_qp (코드)

% 참조 궤적 벡터 생성
R_s = repmat(r_current, Np, 1);
f_qp = -2 * H' *Q_bar* (R_s - P * x_aug); % F (논문) ↔ f_qp (코드)

% 제약 조건 구성: A*x <= b
A_ineq = CC;
b_ineq = dd + dupast * u_past;

% c. quadprog를 사용하여 최적의 DeltaU 계산
Aeq = [];
beq = [];
lb = [];
ub = [];
% x0 = [x_m_past; zeros(num_of_inputs, 1)];
% x0 = zeros(9,1);
x0 = DeltaU_past; 
% x0 = [];
% [DeltaU,uncons]=Qphild(H_qp,f_qp,A_ineq,b_ineq);
[DeltaU, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, Aeq, beq, lb, ub, x0, opts);

% d. 첫 번째 제어 입력 적용
if exitflag == 1 % 해가 존재할 경우
    delta_u = DeltaU(1:num_of_inputs);
    u_current = u_past + delta_u;
    DeltaU_past = DeltaU;
    
    disp('MPC solver success!');
else % 해를 찾지 못한 경우 (비상)
    u_current = u_past; % 이전 값 유지
    DeltaU_past = zeros(num_of_inputs * Nc, 1); % warm start 초기화
    disp('MPC solver failed!');
    disp(exitflag); % 실패 원인 확인
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

