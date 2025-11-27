function [CC_ACS, dd_ACS, dupast_ACS] = ACS_constraints_mimo(A_ACS, b_ACS, num_of_inputs, Nc)
% =========================================================================
% 이 함수는 ACS(Attainable Control Set)에서 파생된 H-representation
% (A_ACS * u <= b_ACS)을 MPC의 제약조건 형태로 변환합니다.
% 변환된 제약 조건 형식: CC_ACS * DeltaU <= dd_ACS - dupast_ACS * u_past
%
% 입력:
%   A_ACS: ACS 부등식 제약 조건의 A 행렬 (quad_ACS_v2의 A_all_TLMN 등)
%   b_ACS: ACS 부등식 제약 조건의 b 벡터 (quad_ACS_v2의 b_all_TLMN 등)
%   num_of_inputs: 입력 변수의 개수 (e.g., 4 for [Fz, Tx, Ty, Tz])
%   Nc:    제어 구간 (Control Horizon)
%
% 출력:
%   CC_ACS:     DeltaU에 곱해지는 행렬 (A_ineq의 ACS 해당 부분)
%   dd_ACS:     제약 조건 우변의 상수 벡터 (dd의 ACS 해당 부분)
%   dupast_ACS: u_past에 곱해지는 행렬 (dupast의 ACS 해당 부분, 부호 반대 주의)
%               (참고: 최종 b_ineq 계산 시 b_ineq = dd + dupast*u_past 이므로,
%                여기서 계산된 dupast_ACS는 부호가 반대임. 즉, dupast = -dupast_ACS)
% =========================================================================

    % 입력 차원 확인
    num_ACS_ineqs = size(A_ACS, 1); % ACS 부등식의 개수

    % 전체 최적화 변수 벡터 DeltaU의 크기
    dim = Nc * num_of_inputs;

    %% 1. 블록 행렬 생성 (수식 전개의 bar_A_ACS, b_ACS 와 동일)
    bar_A_ACS = kron(eye(Nc), A_ACS);       % Nc*num_ACS_ineqs x dim 크기
    mathbf_b_ACS = kron(ones(Nc, 1), b_ACS); % Nc*num_ACS_ineqs x 1 크기

    %% 2. 하삼각 누적합 행렬 E 생성 (u(k+j) = sum(delta u) + u(k-1) 관계 표현)
    E = zeros(dim, dim);
    for j = 1:Nc % 블록 행렬의 열 인덱스
        for i = j:Nc % 블록 행렬의 행 인덱스 (하삼각)
            row_idx = (i-1)*num_of_inputs + 1 : i*num_of_inputs;
            col_idx = (j-1)*num_of_inputs + 1 : j*num_of_inputs;
            E(row_idx, col_idx) = eye(num_of_inputs);
        end
    end

    %% 3. u(k-1) 계수 행렬 L 생성
    L = kron(ones(Nc, 1), eye(num_of_inputs)); % dim x num_of_inputs 크기

    %% 4. 최종 제약 행렬 계산 ( A_ineq * DeltaU <= dd + dupast*u_past 형태 기준 )
    % (bar_A_ACS * E) * DeltaU <= mathbf_b_ACS - (bar_A_ACS * L) * u_past

    CC_ACS = bar_A_ACS * E;                 % A_ineq 부분
    dd_ACS = mathbf_b_ACS;                  % dd 부분
    dupast_ACS = -(bar_A_ACS * L);          % dupast 부분 (부호 확인!)

end