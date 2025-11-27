function AttAlt_MPC % Renamed function for clarity
% =========================================================================
% 선형 MPC 기반 자세 + 고도 통합 제어기 (AttAlt_MPC)
% 외부 PD 위치 제어기에서 phi_des, theta_des, psi_des를 받고,
% 외부에서 Z_des_GF를 받아 자세와 고도를 동시에 제어합니다.
%
% 1. 초기화: 시뮬레이션 시작 시 한 번만 실행하여 MPC에 필요한 모든 행렬 계산.
% 2. 제어 루프: 매 제어 주기마다 QP 문제를 풀고 첫 번째 제어 입력 적용.
% =========================================================================

global Quad;

% --- MPC 파라미터 및 계산된 행렬을 저장하기 위한 영구 변수 ---
persistent P H CC dd dupast A_aug C_aug % Prediction and Constraint matrices
persistent Qx Ru % Weight matrices
persistent opts % quadprog options
persistent x_m_past u_past % Previous state and input
persistent DeltaU_past % Previous optimal DeltaU solution (for warm start)

% MPC 파라미터 정의 (수정됨)
num_of_states = 8;  % 상태: [phi, phi_dot, theta, theta_dot, psi, psi_dot, z, z_dot]
num_of_inputs = 4;  % 입력: [Fz, tau_x, tau_y, tau_z]
num_of_outputs = 8; % 출력: 상태 8개 모두 (Cm = eye(8))
Nc = 15;            % 제어 구간 (Control Horizon) - 필요시 튜닝
Np = 15;            % 예측 구간 (Prediction Horizon) - 필요시 튜닝

%% 1. 초기화 (시뮬레이션 시작 시 한 번만 실행)
if Quad.init == 0
    % ---------------------------------------------------------------------
    % MPC 행렬 계산 (한 번만 수행)
    % ---------------------------------------------------------------------

    % --- 시스템 모델 로딩 ---
    % model_dynamics.m에서 계산된 8x8 Am, 8x4 Bm, 8x8 Cm, 8x4 Dm 이
    % Quad 구조체에 미리 저장되어 있다고 가정합니다.
    % 예시: Quad.Am, Quad.Bm, Quad.Cm, Quad.Dm

    % --- 연속->이산 시간 변환 및 증강 모델 생성 ---
    [Ad,Bd,Cd,Dd] = c2dm(Quad.Am_AA, Quad.Bm_AA, Quad.Cm_AA, Quad.Dm_AA, Quad.Ts);
    [A_aug,B_aug,C_aug] = augment_mimo(Ad, Bd, Cd, num_of_states, num_of_outputs);

    % --- 예측 행렬 계산 ---
    % Y = P*x_aug + H*DeltaU 관계식을 만듭니다
    [P, H] = calculate_prediction_matrices(A_aug, B_aug, C_aug, Np, Nc);

    % --- 비용 함수 가중치 정의 (수정됨, 튜닝 필요!) ---
    % Qx: 상태 오차 가중치 [phi, phi_dot, theta, theta_dot, psi, psi_dot, z, z_dot]
    Qx = diag([1000, 1, 1000, 1, 10, 1, 500, 1]); % 8x8, 예시 값

    % Ru: 입력 '변화율' 가중치 [Delta_Fz, Delta_tau_x, Delta_tau_y, Delta_tau_z]
    Ru = diag([1, 10, 10, 10]); % 4x4, 예시 값


    % --- 제약 조건 행렬 계산 (수정됨) ---
    % 입력 크기 제약
    umax = [Quad.U1_max; Quad.U2_max ; Quad.U3_max ; Quad.U4_max]; % 4x1 벡터
    umin = [Quad.U1_min; Quad.U2_min ; Quad.U3_min ; Quad.U4_min]; % 4x1 벡터
    % 입력 변화율 제약 (예시: 최대 토크의 80%까지만 변화 허용)
    Delta_umax = 0.8 * umax; % 4x1 벡터
    % [CC, dd, dupast] = constraints_mimo(Delta_umax, umax, umin, num_of_inputs, Nc);
    
    % quad_ACS_v2.m 에서 계산된 A_all_TLMN, b_all_TLMN 이 Quad 구조체에 있다고 가정
    [CC, dd, dupast] = ACS_constraints_mimo(Quad.A_all_TLMN, Quad.b_all_TLMN, num_of_inputs, Nc);

    % CC*DeltaU <= dd + dupast*u_past 형태로 변환

    % --- quadprog 옵션 ---
    opts = optimoptions('quadprog','Algorithm','active-set','Display','off');

    % --- 초기값 설정 (수정됨) ---
    % 현재 z(Global), z_dot(Global) 상태를 읽어와야 함
    x_m_past = [Quad.phi; Quad.phi_dot; Quad.theta; Quad.theta_dot; Quad.psi; Quad.psi_dot; Quad.Z; Quad.Z_dot]; % 8x1 벡터
    % 이전 입력 초기값
    u_past = [Quad.U1; Quad.U2; Quad.U3; Quad.U4]; % 4x1 벡터 (주의: u_past는 U1~U4)
    
    DeltaU_past = zeros(num_of_inputs * Nc, 1); % (4*Nc) x 1 벡터

    Quad.init = 1; % 초기화 완료 플래그 설정
end



%% 2. MPC 제어 루프 (매 제어 주기마다 실행)

% a. 현재 상태 및 목표값 읽기 (수정됨 - Global Z 사용)
x_m_current = [Quad.phi; Quad.phi_dot; Quad.theta; Quad.theta_dot; Quad.psi; Quad.psi_dot; Quad.Z; Quad.Z_dot]; % 8x1 벡터

% 목표 값: 외부 PD에서 phi_des, theta_des, psi_des를 받고, 외부에서 Z_des_GF를 받음.
% 각속도/수직속도 목표는 0으로 가정.
r_current = [Quad.phi_des; 0; Quad.theta_des; 0; Quad.psi_des; 0; Quad.Z_des_GF; 0]; % 8x1 벡터

% b. QP 문제 구성
% 증강 상태 벡터 구성
delta_xm = x_m_current - x_m_past;
y_current = Quad.Cm_AA * x_m_current; % Cm이 eye(8)이면 y_current = x_m_current
x_aug = [delta_xm; y_current]; % 증강 상태 벡터

% QP 비용 함수 행렬 계산 (H_qp, f_qp의 상수 부분)
Q_bar = kron(eye(Np), Qx); % Np*ny x Np*ny 블록 대각 행렬
R_bar = kron(eye(Nc), Ru); % Nc*nu x Nc*nu 블록 대각 행렬
H_qp = 2 * (H' * Q_bar * H + R_bar); % QP의 Hessian 행렬 (상수)

% 비용 함수 구성: min 0.5*DeltaU'*H_qp*DeltaU + f_qp'*DeltaU
R_s = repmat(r_current, Np, 1); % 목표 궤적 벡터
f_qp = -2 * H' * Q_bar * (R_s - P * x_aug); % QP의 선형 항 (매 스텝 계산)

% 제약 조건 구성: A_ineq * DeltaU <= b_ineq
A_ineq = CC; % 상수 행렬
b_ineq = dd + dupast * u_past; % 매 스텝 계산

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
[DeltaU, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, Aeq, beq, lb, ub, x0, opts); % Warm start 사용

% d. 첫 번째 제어 입력 적용 (수정됨)
if exitflag == 1 % 해를 성공적으로 찾은 경우
    delta_u = DeltaU(1:num_of_inputs); % 첫 4개 요소 추출
    u_current = u_past + delta_u;      % 4x1 벡터 덧셈
    DeltaU_past = DeltaU;              % 다음 스텝 warm start 위해 저장
    disp('MPC solver success!'); % 디버깅용
else % 해를 찾지 못한 경우 (제약조건 위반, 계산 시간 초과 등)
    u_current = u_past; % 이전 값 유지 (안전 조치)
    DeltaU_past = zeros(num_of_inputs * Nc, 1); % warm start 초기화
    disp('MPC solver failed! Flag:');
    disp(exitflag); % 실패 원인 확인
end

% 계산된 추력/토크를 Quad 구조체에 할당 (수정됨)
Quad.U1 = u_current(1); % Fz
Quad.U2 = u_current(2); % tau_x
Quad.U3 = u_current(3); % tau_y
Quad.U4 = u_current(4); % tau_z

% Saturation 처리 (4개 입력 모두)
Quad.U1 = min(Quad.U1_max, max(Quad.U1_min, Quad.U1));
Quad.U2 = min(Quad.U2_max, max(Quad.U2_min, Quad.U2));
Quad.U3 = min(Quad.U3_max, max(Quad.U3_min, Quad.U3));
Quad.U4 = min(Quad.U4_max, max(Quad.U4_min, Quad.U4));

% e. 다음 스텝을 위해 현재 값을 과거 값으로 업데이트 (수정됨)
x_m_past = x_m_current; % 8x1 벡터 저장
u_past = u_current;     % 4x1 벡터 저장

end