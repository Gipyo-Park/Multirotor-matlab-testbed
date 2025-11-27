close all; clc; clear all;
https://cookierobotics.com/066/
% --- 물리 상수 정의 ---
% 이 값들은 실제 드론의 프로펠러와 모터, 기체 사양에 따라 결정됩니다.
l = 0.15;      % 기체 중심에서 로터까지의 거리 (lever arm, 단위: m)
ct = 3.0e-6;  % 추력 계수 (Thrust Coefficient)
cq = 8.0e-8;  % 모멘트(항력) 계수 (Moment/Drag Coefficient)

% --- 로터 설정 ---
% {x_norm, y_norm, '방향'}
% x, y 위치는 팔 길이(l)에 대한 비율로 정규화된 값입니다.

% XYZ are in front-right-down coordinates
% rotors = [
%     Position Prop-Direction
%    [ x1, y1, 'CW'/'CCW'],
%    [ x2, y2, 'CW'/'CCW'],
%    [ x3, y3, 'CW'/'CCW'],
%    ...
% ]

% QuadrotorX
%   cw  3   1 ccw
%         ^ x
%         |
%         B-->y
%    
%   ccw 2   4 cw

% rotors = {
%      1,  1, 'CCW';  % 1번 로터 (오른쪽 앞)
%     -1, -1, 'CCW';  % 2번 로터 (왼쪽 뒤)
%      1, -1, 'CW';   % 3번 로터 (오른쪽 뒤)
%     -1,  1, 'CW'    % 4번 로터 (왼쪽 앞)
% };

% Quadrotor+
%            2 cw 
%            ^ x
%            |
%   ccw 3    B-->y    1 ccw
% 
%            4 cw 

% rotors = {
%      0,  1, 'CCW';  % 1번 로터 
%      1,  0, 'CW';  % 2번 로터 
%      0, -1, 'CCW';   % 3번 로터 
%     -1,  0, 'CW'    % 4번 로터 
% };

% Quadrotor+
%            1 ccw 
%            ^ x
%            |
%   cw 4     B-->y    2 cw
% 
%            3 ccw 

rotors = {
     1,  0, 'CCW';  % 1번 로터 
     0,  1, 'CW';  % 2번 로터 
    -1,  0, 'CCW';   % 3번 로터 
     0, -1, 'CW'    % 4번 로터 
};

num_rotors = size(rotors, 1);

% --- Control allocation 행렬 A ---
% 이 행렬은 각 로터의 각속도 제곱(w^2)이
% 기체의 총 추력과 각 축의 토크에 어떻게 기여하는지를 나타냅니다.
% [총 추력; 롤 토크; 피치 토크; 요 토크] = A * [w1^2; w2^2; w3^2; w4^2]
A = zeros(4, num_rotors);

for i = 1:num_rotors
    x_norm = rotors{i, 1}; % 정규화된 x 위치 (-1 또는 1)
    y_norm = rotors{i, 2}; % 정규화된 y 위치 (-1 또는 1)
    direction = rotors{i, 3};
    
    % 1행: 총 추력 (Sum of c_t * w_i^2)
    A(1, i) = ct;
    
    % 2행: 롤 토크 (Sum of -y_pos * F_i = Sum of -(y_norm*l) * (c_t*w_i^2))
    A(2, i) = -y_norm * l * ct;
    
    % 3행: 피치 토크 (Sum of x_pos * F_i = Sum of (x_norm*l) * (c_t*w_i^2))
    A(3, i) = x_norm * l * ct;
    
    % 4행: 요 토크 (Sum of direction * M_i = Sum of direction * (c_q*w_i^2))
    if strcmp(direction, 'CW')
        A(4, i) = -cq;
    else % CCW
        A(4, i) = cq;
    end
end


% 결과 출력
disp('Control allocation Matrix A:');
disp(A);

% --- Pseudo-inverse 행렬 B ---
% [w1^2; w2^2; w3^2; w4^2] = B * [원하는 총 추력; 원하는 롤 토크; ...]
B = pinv(A);

disp('Pseudo-inverse B (pinv(A)):');
disp(B);