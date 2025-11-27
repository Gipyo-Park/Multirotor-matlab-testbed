function [CC, dd, dupast] = constraints_mimo(Dumax, umax, umin, no_of_inputs, Nc)
% =========================================================================
% 이 함수는 MPC의 제약조건을 위한 행렬을 생성합니다.
% 제약 조건 형식: CC * DeltaU <= dd - dupast * u_past
%
% 입력:
%   Dumax: 입력 '변화율'의 상한 벡터 (e.g., [max_d_tau_x; ...])
%   umax:  입력의 상한 벡터 (e.g., [max_tau_x; ...])
%   umin:  입력의 하한 벡터 (e.g., [min_tau_x; ...])
%   no_of_inputs: 입력 변수의 개수 (e.g., 3 for torques)
%   Nc:    제어 구간 (Control Horizon)
%
% 출력:
%   CC, dd, dupast: 제약조건 수식을 구성하는 행렬 및 벡터
% =========================================================================

    % 전체 최적화 변수 벡터 DeltaU의 크기
    dim = Nc * no_of_inputs;

    % 효율성을 위해 행렬 크기 미리 할당
    CC = zeros(4 * dim, dim);
    dd = zeros(4 * dim, 1);
    
    %% CC 행렬 구성: DeltaU의 계수
    
    % 1. 입력 변화율(DeltaU)에 대한 제약
    I_dim = eye(dim);
    CC(1:2*dim, 1:dim) = [I_dim; -I_dim];
    
    % 2. 입력 크기(u)에 대한 제약
    % 누적 합계를 위한 하삼각행렬 E를 정확하게 생성 (버그 수정된 부분)
    E = zeros(dim, dim);
    for j = 1:Nc % 블록 행렬의 열(column) 인덱스
        for i = j:Nc % 블록 행렬의 행(row) 인덱스 (하삼각 부분을 위해 i는 j부터 시작)
            row_idx = (i-1)*no_of_inputs + 1 : i*no_of_inputs;
            col_idx = (j-1)*no_of_inputs + 1 : j*no_of_inputs;
            E(row_idx, col_idx) = eye(no_of_inputs);
        end
    end
    
    Cu = [eye(dim); -eye(dim)];
    CuE = Cu * E;
    
    CC(2*dim+1 : 4*dim, 1:dim) = CuE;
    
    %% dd 벡터 구성: 제약 조건의 우변 (상수항)
    
    % 1. 입력 변화율에 대한 상/하한 값
    dd_delta_u_upper = repmat(Dumax, Nc, 1);
    dd_delta_u_lower = repmat(Dumax, Nc, 1); % -Dumin, Dumin = -Dumax 가정
    
    % 2. 입력 크기에 대한 상/하한 값
    dd_u_upper = repmat(umax, Nc, 1);
    dd_u_lower = repmat(-umin, Nc, 1);
    
    dd = [dd_delta_u_upper; 
          dd_delta_u_lower; 
          dd_u_upper; 
          dd_u_lower];
          
    %% dupast 행렬 구성: u(k-1)의 계수
    
    % L 행렬: u(k-1)이 미래의 모든 u(k+j)에 영향을 주는 것을 표현
    L = repmat(eye(no_of_inputs), Nc, 1);
    
    % -Cu*L
    dupast_u = -Cu * L;
    
    dupast = [zeros(2*dim, no_of_inputs); % 변화율 제약은 u(k-1)과 무관
              dupast_u];

end