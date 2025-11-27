function [P, H] = calculate_prediction_matrices(A_aug, B_aug, C_aug, Np, Nc)
% =========================================================================
% 증강 모델과 예측/제어 구간을 기반으로 MPC 예측 행렬 P, H를 계산합니다.
%
% 입력:
%   A_aug: 증강 상태 행렬
%   B_aug: 증강 입력 행렬
%   C_aug: 증강 출력 행렬
%   Np:    예측 구간 (Prediction Horizon)
%   Nc:    제어 구간 (Control Horizon)
%
% 출력:
%   P:     현재 상태에 대한 미래 출력 예측 행렬
%   H:     미래 제어 입력에 대한 미래 출력 예측 행렬
% =========================================================================

    % 증강 모델의 상태 및 입력 차원 확인
    [nx_aug, ~] = size(A_aug);
    [~, nu] = size(B_aug);
    [ny, ~] = size(C_aug);

    % 효율성을 위해 P, H 행렬 크기 미리 할당
    P = zeros(Np * ny, nx_aug);
    H = zeros(Np * ny, Nc * nu);

    % P 행렬 계산
    % P = [C_aug*A_aug; C_aug*A_aug^2; ...; C_aug*A_aug^Np]
    for i = 1:Np
        P_row_start = (i-1)*ny + 1;
        P_row_end = i*ny;
        P(P_row_start:P_row_end, :) = C_aug * (A_aug^i);
    end

    % H 행렬 계산 (Block Lower Triangular Matrix)
    % H = [ C*B,      0,        ..., 0      ;
    %       C*A*B,    C*B,      ..., 0      ;
    %       ...       ...       ...  ...     ;
    %       C*A^(Np-1)*B, ..., C*A^(Np-Nc)*B ]
    
    % 첫 번째 열 블록 계산
    H_col_block = zeros(Np*ny, nu);
    for i = 1:Np
        row_start = (i-1)*ny + 1;
        row_end = i*ny;
        H_col_block(row_start:row_end, :) = C_aug * (A_aug^(i-1)) * B_aug;
    end
    
    % 계산된 첫 열 블록을 사용하여 전체 H 행렬 구성
    for j = 1:Nc
        col_start = (j-1)*nu + 1;
        col_end = j*nu;
        
        row_start = (j-1)*ny + 1;
        H(row_start:end, col_start:col_end) = H_col_block(1 : (Np-j+1)*ny, :);
    end

end