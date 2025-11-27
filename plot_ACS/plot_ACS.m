clear all; clc; close all;

%% 1. 모터 제한 및 극점 생성
nMotors = 8;
w_min = 0;
w_max = 463;

b  = 0.00053;
d  = 0.0000169;
L1 = 0.545;  % 롤 모멘트암(일부 모터들에 적용)
L2 = 0.36;   % 피치 모멘트암(일부 모터들에 적용)
L3 = 1.08;   % 다른 모터 배치용 모멘트암

% 각 모터 입력 범위를 저장
grids = cell(1, nMotors);
for i = 1:nMotors
    grids{i} = [w_min, w_max];
end

% ndgrid로 모든 극점(2^8=256개) 생성
[A{1:nMotors}] = ndgrid(grids{:});
vertices = [];
for i = 1:nMotors
    vertices = [vertices, A{i}(:)]; % (256 x 8): 모든 제어입력 경우의 수 (최소, 최대)
end

%% 2. 제어 효과 행렬 (T[N], L[Nm], M[Nm], N[Nm] 순)
%  - 각 행: 출력 물리량 (추력, 롤모멘트, 피치모멘트, 요모멘트)
%  - 각 열: 모터별 회전수에 대한 계수
K = [
    -b,    -b,    -b,    -b,    -b,    -b,    -b,    -b;       % T (추력, N)
    -b*L1,  b*L1, -b*L1, -b*L1,  b*L1,  b*L1,  b*L1, -b*L1;     % L (롤, Nm)
     b*L3, -b*L3,  b*L2, -b*L3,  b*L3, -b*L2,  b*L2, -b*L2;     % M (피치, Nm)
    -d,    -d,     d,     d,     d,     d,    -d,    -d        % N (요, Nm)
];

% 각 극점에 대해 조종력 y = K * v (결과를 전치하여 256x4 행렬 생성)
ACS_points = (K * vertices')';

% ACS_points의 열 순서:
% 1: Thrust (T, unit = N)
% 2: Roll Moment (L, unit = Nm)
% 3: Pitch Moment (M, unit = Nm)
% 4: Yaw Moment (N, unit = Nm)

%% 3. (x=Roll(L), y=Pitch(M), z=Thrust(T)) 시각화
%    -> L=2열, M=3열, T=1열 => [2,3,1]
pts_LMT = ACS_points(:, [2, 3, 1]);
hull_LMT = convhulln(pts_LMT);

figure('Name','ACS: Roll, Pitch, Thrust','NumberTitle','off');
trisurf(hull_LMT, ...
    pts_LMT(:,1), pts_LMT(:,2), pts_LMT(:,3), ...
    'FaceColor','cyan','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
hold on;
plot3(pts_LMT(:,1), pts_LMT(:,2), pts_LMT(:,3), 'bo', 'MarkerSize', 3);

xlabel('Roll Moment L (Nm)');
ylabel('Pitch Moment M (Nm)');
zlabel('Thrust T (N)');
title('ACS: (Roll L, Pitch M, Thrust T)');
grid on; view(3);
camlight; lighting gouraud;

%% 4. (x=Roll(L), y=Pitch(M), z=Yaw(N)) 시각화
%    -> L=2열, M=3열, N=4열 => [2,3,4]
pts_LMN = ACS_points(:, [2, 3, 4]);
hull_LMN = convhulln(pts_LMN);

figure('Name','ACS: Roll, Pitch, Yaw','NumberTitle','off');
trisurf(hull_LMN, ...
    pts_LMN(:,1), pts_LMN(:,2), pts_LMN(:,3), ...
    'FaceColor','magenta','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
hold on;
plot3(pts_LMN(:,1), pts_LMN(:,2), pts_LMN(:,3), 'ro', 'MarkerSize', 3);

xlabel('Roll Moment L (Nm)');
ylabel('Pitch Moment M (Nm)');
zlabel('Yaw Moment N (Nm)');
title('ACS: (Roll L, Pitch M, Yaw N)');
grid on; view(3);
camlight; lighting gouraud;
