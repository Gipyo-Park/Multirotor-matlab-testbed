
function Trajectory_Lemniscate


global Quad;

% Quad.counter는 0부터 증가하는 정수라고 가정
t = Quad.counter * Quad.Ts;  % Ts는 샘플링 타임 (예: 0.01초)

% 경로 파라미터
a = 0.5;    % x/y 크기
b = 0.5;    % z 높이 변화량
w = 0.1;    % 주기 조절

% 무한대 모양 경로 (lemniscate of Gerono)
Quad.X_des_GF = a * sin(w * t);
Quad.Y_des_GF = a * sin(w * t) .* cos(w * t);
Quad.Z_des_GF = 1.0 + b * sin(2 * w * t);   % 높이도 변화 가능

Quad.X_des_log(Quad.counter) = Quad.X_des_GF;
Quad.Y_des_log(Quad.counter) = Quad.Y_des_GF;
Quad.Z_des_log(Quad.counter) = Quad.Z_des_GF;

Quad.X_log(Quad.counter) = Quad.X;
Quad.Y_log(Quad.counter) = Quad.Y;
Quad.Z_log(Quad.counter) = Quad.Z;