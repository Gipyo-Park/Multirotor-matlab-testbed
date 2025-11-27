clear all; clc; close all;

%% 설정 변수
n = 4;                      % 모터(액추에이터) 개수
w_min_val = 0;
w_max_val = 925;         % 액추에이터 각속도 [rad/sec]
m = 1.6;                     % 비행체 질량 (kg)
g = 9.81;                   % 중력가속도 (m/s^2)

% Inject_Fault: 고장 날 액추에이터 번호 및 고장 정도
% 예: 1번 액추에이터는 완전 고장, 4번은 50% 고장, 8번은 75% 고장
Inject_Fault = [1];
Fault_Level = [1];  % 1이면 완전, 0이면 정상; 여기서는 0.999로 거의 완전 고장

%% [Normal Case] 정상 조건의 8차원 박스 생성, A*x <= b
A_box = [ eye(n); -eye(n) ];
b_box = [ w_max_val * ones(n,1); -w_min_val * ones(n,1) ];
[V_box, nr_box, nre_box] = lcon2vert(A_box, b_box, [], [], 1e-10, true);   % (9) 식

% --- [추가] 수치적 오차를 0으로 정리 ---
tolerance = 1e-9; % 허용 오차 설정 (lcon2vert의 tol보다 약간 크게 설정)
V_box(abs(V_box) < tolerance) = 0;
% ---------------------------------------

disp('정상 조건: 4차원 박스의 극점 (V_box):');   % V_box 는 9식의 V_y 이다
disp(V_box);



%% 2. 제어 효과 행렬 K 적용 (정상 및 고장 조건 모두 동일)
% XYZ are in front-right-down coordinates
% 출력: [Thrust T, Roll L, Pitch M, Yaw N]
% 단위: T (N), L/M/N (Nm)
ct_  = 1.3328e-5;
ct_val = ct_ * w_max_val;
cq_  = 1.3858e-6;
cq_val = cq_ * w_max_val;
L= 0.56;

% QuadrotorX
%   cw  3   1 ccw
%         ^ x
%         |
%         B-->y
%    
%   ccw 2   4 cw

% K = [ ct_val,    ct_val,    ct_val,    ct_val;
%       -ct_val*L,  ct_val*L, ct_val*L, -ct_val*L;
%        ct_val*L, ct_val*L,  -ct_val*L, -ct_val*L;
%       cq_val,    -cq_val,    cq_val,     -cq_val];

% Quadrotor+
%            1 ccw 
%            ^ x
%            |
%   cw 4     B-->y    2 cw
% 
%            3 ccw 

K = [ ct_val,    ct_val,    ct_val,    ct_val;
      0,  -ct_val*L, 0, ct_val*L;
       ct_val*L, 0,  -ct_val*L, 0;
      cq_val,    -cq_val,    cq_val,     -cq_val];

ACS_points_normal = (K * V_box')';          % 정상 조건: [T,L,M,N], (10)식, ACS_points_normal 은 V'_omega이다


%% 3. 정상 ACS 시각화: (Roll, Pitch, Thrust)
pts_LMT = ACS_points_normal(:, [2, 3, 1]);   % x=Roll, y=Pitch, z=Thrust
[hull_LMT, convex_vol_LMT] = convhulln(pts_LMT);    % (11)식
convexIdx_LMT = unique(hull_LMT);
figure('Name','Normal ACS: Roll, Pitch, Thrust','NumberTitle','off');
trisurf(hull_LMT, pts_LMT(:,1), pts_LMT(:,2), pts_LMT(:,3), ...
    'FaceColor','cyan','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
hold on;
plot3(pts_LMT(:,1), pts_LMT(:,2), pts_LMT(:,3), 'bo','MarkerSize',10);
plot3(pts_LMT(convexIdx_LMT,1), pts_LMT(convexIdx_LMT,2), pts_LMT(convexIdx_LMT,3), ...
    'ks','MarkerSize',8,'MarkerFaceColor','r');
xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Thrust T (N)');
title('Normal ACS: (Roll, Pitch, Thrust)');
grid on; view(3); camlight; lighting gouraud;
disp('Normal convex_vol_LMT = '); disp(convex_vol_LMT);

%% 4. 정상 ACS 시각화: (Roll, Pitch, Yaw)
pts_LMN = ACS_points_normal(:, [2, 3, 4]);   % x=Roll, y=Pitch, z=Yaw
[hull_LMN, convex_vol_LMN] = convhulln(pts_LMN);    % (11)식
convexIdx_LMN = unique(hull_LMN);
figure('Name','Normal ACS: Roll, Pitch, Yaw','NumberTitle','off');
trisurf(hull_LMN, pts_LMN(:,1), pts_LMN(:,2), pts_LMN(:,3), ...
    'FaceColor','magenta','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
hold on;
plot3(pts_LMN(:,1), pts_LMN(:,2), pts_LMN(:,3), 'ro','MarkerSize',10);
plot3(pts_LMN(convexIdx_LMN,1), pts_LMN(convexIdx_LMN,2), pts_LMN(convexIdx_LMN,3), ...
    'ks','MarkerSize',8,'MarkerFaceColor','b');
xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Yaw Moment N (Nm)');
title('Normal ACS: (Roll, Pitch, Yaw)');
grid on; view(3); camlight; lighting gouraud;
disp('Normal convex_vol_LMN = '); disp(convex_vol_LMN);

%% 5. 정상 H-Representation 도출 (L,M,T)
[A_all, b_all, Aeq_all, beq_all] = vert2lcon(pts_LMT, 1e-10);
disp('Normal ACS (Roll, Pitch, Thrust) (LMT)의 부등식 조건:');
disp('Inequality A = '); disp(A_all);
disp('Inequality b = '); disp(b_all);
if ~isempty(Aeq_all)
    disp('Equality 조건:'); disp(Aeq_all); disp(beq_all);
else
    disp('Equality 조건은 없습니다. \n');
end

%% 6. 정상 H-Representation 도출 (L,M,N)
[A_all_LMN, b_all_LMN, Aeq_all_LMN, beq_all_LMN] = vert2lcon(pts_LMN, 1e-10);
disp('Normal ACS (Roll, Pitch, Yaw) (LMN)의 부등식 조건:');
disp('Inequality A = '); disp(A_all_LMN);
disp('Inequality b = '); disp(b_all_LMN);
if ~isempty(Aeq_all_LMN)
    disp('Equality 조건:'); disp(Aeq_all_LMN); disp(beq_all_LMN);
else
    disp('Equality 조건은 없습니다. \n');
end

%% 7. 정상 H-Representation 도출 (T,L,M,N, 4차원)
pts_TLMN = ACS_points_normal;  % [T,L,M,N]
[hull_TLMN, convex_vol_TLMN] = convhulln(pts_TLMN);    % (11)식
% hull_TLMN: 볼록 껍질을 구성하는 다면체(facet) 정보를 담고 있습니다. 각 행은 하나의 다면체를 이루는 점들의 인덱스를 나타냅니다
% convex_vol_TLMN: 계산된 4차원 볼록 껍질의 "부피(hypervolume)"를 의미합니다.
convexIdx_TLMN = unique(hull_TLMN);     % convexIdx_TLMN 이게 (11)식에서 바라는 V_omega가 된다
[A_all_TLMN, b_all_TLMN, Aeq_all_TLMN, beq_all_TLMN] = vert2lcon(pts_TLMN, 1e-10);
% 이론상 vert2lcon 에 pts_TLMN를 넣으면 안된다. V_omega = pts_TLMN(convexIdx_TLMN, :) 해서 V_omega 이거를 넣어야한다
% 하지만 vert2lcon(pts_TLMN, 1e-10); 를 해도 맞는 이유는 vert2lcon 가 어차피 내부점들을 소거하기 때문이다
disp('Normal ACS (T, L, M, N)의 부등식 조건:');
disp('Inequality A = '); disp(A_all_TLMN);
disp('Inequality b = '); disp(b_all_TLMN);
if ~isempty(Aeq_all_TLMN)
    disp('Equality 조건:'); disp(Aeq_all_TLMN); disp(beq_all_TLMN);
else
    disp('Equality 조건은 없습니다. \n');
end
disp('Normal convex_vol_TLMN = '); disp(convex_vol_TLMN);

%% 8. 정상 슬라이싱: N=0 => 3차원 (T,L,M) 시각화
% 드론이 수직축을 중심으로 회전하지 않는 상태를 강제하는 조건
% 예를 들어, 드론이 앞뒤나 좌우로 움직일 때 제자리에서 팽이처럼 돌지 않고 안정적으로 기동하는 상황을 시뮬레이션하는 것이다
Aeq_N0 = [0 0 0 1]; beq_N0 = 0;
[V_sliceN0, nr_sliceN0, nre_sliceN0] = lcon2vert(A_all_TLMN, b_all_TLMN, Aeq_N0, beq_N0, 1e-10, true);
if isempty(V_sliceN0)
    disp('Normal: N=0 슬라이스가 유효하지 않거나, 점이 없습니다.');
else
    sliceN0_RPT = V_sliceN0(:, [2,3,1]);  % x=Roll, y=Pitch, z=Thrust
    if size(sliceN0_RPT,1) < 4
        disp('Normal: N=0 슬라이스에서 점이 너무 적습니다.');
    else
        [hullN0_RPT, vol_N0] = convhulln(sliceN0_RPT);
        convexIdx_N0 = unique(hullN0_RPT);
        figure('Name','Normal Slice: N=0 => 3D in (Roll, Pitch, Thrust)','NumberTitle','off');
        trisurf(hullN0_RPT, sliceN0_RPT(:,1), sliceN0_RPT(:,2), sliceN0_RPT(:,3), ...
            'FaceColor','green','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
        hold on;
        plot3(sliceN0_RPT(:,1), sliceN0_RPT(:,2), sliceN0_RPT(:,3), 'bo','MarkerSize',10);
        plot3(sliceN0_RPT(convexIdx_N0,1), sliceN0_RPT(convexIdx_N0,2), sliceN0_RPT(convexIdx_N0,3), ...
            'ks','MarkerSize',8,'MarkerFaceColor','r');
        xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Thrust T (N)');
        title('Normal Slice: N=0 (3D) in (Roll, Pitch, Thrust)');
        grid on; view(3); camlight; lighting gouraud;
        disp('Normal convex_vol_N0 (for N=0 slice) = '); disp(vol_N0);
    end
end

%% 9. 정상 슬라이싱: T=mg, N=0 => 2차원 (L,M) 시각화
% 호버링하면서 드론이 팽이처럼 돌지 않는 기동을 하는 상황
Aeq_TN = [1 0 0 0; 0 0 0 1]; 
beq_TN = [m*g; 0];
[V_sliceTN, nr_sliceTN, nre_sliceTN] = lcon2vert(A_all_TLMN, b_all_TLMN, Aeq_TN, beq_TN, 1e-10, true);
if isempty(V_sliceTN)
    disp('Normal: T=mg, N=0 슬라이스가 유효하지 않거나, 점이 없습니다.');
else
    sliceTN_LM = V_sliceTN(:, [2,3]);  % (L,M)
    if size(sliceTN_LM,1) < 3
        disp('Normal: T=mg, N=0 슬라이스에서 점이 너무 적습니다.');
    else
        [hullTN_LM, vol_TN] = convhulln(sliceTN_LM);
        convexIdx_TN = unique(hullTN_LM);
        figure('Name','Normal Slice: T=mg, N=0 => 2D in (Roll, Pitch)','NumberTitle','off');
        plot(sliceTN_LM(:,1), sliceTN_LM(:,2), 'bo','MarkerSize',10);
        hold on;
        plot(sliceTN_LM(hullTN_LM,1), sliceTN_LM(hullTN_LM,2), 'r-','LineWidth',2);
        plot(sliceTN_LM(convexIdx_TN,1), sliceTN_LM(convexIdx_TN,2), 'ks','MarkerSize',8,'MarkerFaceColor','g');
        xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)');
        title('Normal Slice: T=mg, N=0 (2D) in (Roll, Pitch)');
        grid on;
        disp('Normal convex_vol_TN (for T=mg, N=0 slice) = '); disp(vol_TN);
    end
end

%% 10. 정상 슬라이싱: T=mg => 3차원 (L,M,N) 시각화
% 드론이 고도를 유지하며 제자리 비행(호버링)하는 조건을 의미
% 드론이 고도를 잃거나 얻지 않으면서 최대로 만들어낼 수 있는 롤, 피치, 요 모멘트의 범위를 보여줍니다
% 이는 기체의 호버링 시 안정성과 기동성을 나타내는 매우 중요한 지표입니다
Aeq_T = [1 0 0 0]; beq_T = m*g;
[V_sliceT, nr_sliceT, nre_sliceT] = lcon2vert(A_all_TLMN, b_all_TLMN, Aeq_T, beq_T, 1e-10, true);
if isempty(V_sliceT)
    disp('Normal: T=mg 슬라이스가 유효하지 않거나, 점이 없습니다.');
else
    sliceT_LMN = V_sliceT(:, [2,3,4]);  % (L,M,N)
    if size(sliceT_LMN,1) < 4
        disp('Normal: T=mg 슬라이스에서 점이 너무 적습니다.');
    else
        [hullT_LMN, vol_T] = convhulln(sliceT_LMN);
        convexIdx_T = unique(hullT_LMN);
        figure('Name','Normal Slice: T=mg => 3D in (Roll, Pitch, Yaw)','NumberTitle','off');
        trisurf(hullT_LMN, sliceT_LMN(:,1), sliceT_LMN(:,2), sliceT_LMN(:,3), ...
            'FaceColor','yellow','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
        hold on;
        plot3(sliceT_LMN(:,1), sliceT_LMN(:,2), sliceT_LMN(:,3), 'bo','MarkerSize',10);
        plot3(sliceT_LMN(convexIdx_T,1), sliceT_LMN(convexIdx_T,2), sliceT_LMN(convexIdx_T,3), ...
            'ks','MarkerSize',8,'MarkerFaceColor','m');
        xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Yaw Moment N (Nm)');
        title('Normal Slice: T=mg (3D) in (Roll, Pitch, Yaw)');
        grid on; view(3); camlight; lighting gouraud;
        disp('Normal convex_vol_T (for T=mg slice) = '); disp(vol_T);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% [Fault Injection Case] 고장 조건 (등식 적용)
% 고장 조건: Inject_Fault에 해당하는 액추에이터에 대해 w(i) = (1 - fault_level)*w_max_val
disp('--------------[Fault Injection Case]------------');
% Fault_Level이 정확히 1인지 확인
is_complete_fault = all(Fault_Level == 1);

if is_complete_fault
    % --- Fault_Level이 1일 경우: lcon2vert를 우회하여 꼭짓점 직접 계산 ---
    disp('Fault_Level = 1이므로, lcon2vert를 사용하지 않고 꼭짓점을 직접 계산합니다.');

    % 1. 전체 모터, 고장 모터, 정상 작동 모터의 인덱스를 구분
    all_motors = 1:n;
    faulty_motors = Inject_Fault;
    free_motors = setdiff(all_motors, faulty_motors); % 자유롭게 움직이는 모터
    
    num_free_motors = length(free_motors);
    num_vertices = 2^num_free_motors;
    
    % 2. 자유로운 모터들이 가질 수 있는 min/max 값의 모든 조합을 생성
    min_max_cell = repmat({[w_min_val, w_max_val]}, 1, num_free_motors);
    grids = cell(1, num_free_motors);
    [grids{:}] = ndgrid(min_max_cell{:});
    
    free_motor_vertices = zeros(num_vertices, num_free_motors);
    for i = 1:num_free_motors
        free_motor_vertices(:, i) = grids{i}(:);
    end

    % 3. 최종 꼭짓점 행렬(V_box_fault_eq)을 조립
    V_box_fault_eq = zeros(num_vertices, n);
    %   - 고장난 모터 위치에는 0을 채움
    V_box_fault_eq(:, faulty_motors) = 0;
    %   - 자유로운 모터 위치에는 위에서 생성한 min/max 조합을 채움
    V_box_fault_eq(:, free_motors) = free_motor_vertices;

else
    % --- Fault_Level이 1이 아닐 경우: 기존 lcon2vert 방식 사용 ---
    disp('Fault_Level이 1이 아니므로, 기존 lcon2vert 방식을 사용합니다.');
    
    Aeq_fault = zeros(length(Inject_Fault), n);
    beq_fault = zeros(length(Inject_Fault), 1);
    for i = 1:length(Inject_Fault)
        idx = Inject_Fault(i);
        Aeq_fault(i, idx) = 1;
        beq_fault(i) = (1 - Fault_Level(i)) * w_max_val;
    end
    [V_box_fault_eq, ~, ~] = lcon2vert(A_box, b_box, Aeq_fault, beq_fault, 1e-10, true);
end
% --- [추가] 수치적 오차를 0으로 정리 ---
tolerance = 1e-9; % 허용 오차 설정 (lcon2vert의 tol보다 약간 크게 설정)
V_box_fault_eq(abs(V_box_fault_eq) < tolerance) = 0;
% ---------------------------------------
disp('Fault Injection 결과로 얻은 꼭짓점 (V_box_fault_eq):');
disp(V_box_fault_eq);

%% Faulty ACS_points 계산 (Fault Injection Case)
ACS_points_fault = (K * V_box_fault_eq')';   % 고장 조건 (등식 적용): [T,L,M,N]

%% F3. Faulty ACS 시각화: (Roll, Pitch, Thrust)
pts_LMT_fault = ACS_points_fault(:, [2,3,1]);
[hull_LMT_fault, convex_vol_LMT_fault] = convhulln(pts_LMT_fault);
convexIdx_LMT_fault = unique(hull_LMT_fault);
figure('Name','Faulty ACS: Roll, Pitch, Thrust','NumberTitle','off');
trisurf(hull_LMT_fault, pts_LMT_fault(:,1), pts_LMT_fault(:,2), pts_LMT_fault(:,3), ...
    'FaceColor','cyan','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
hold on;
plot3(pts_LMT_fault(:,1), pts_LMT_fault(:,2), pts_LMT_fault(:,3), 'bo','MarkerSize',10);
plot3(pts_LMT_fault(convexIdx_LMT_fault,1), pts_LMT_fault(convexIdx_LMT_fault,2), pts_LMT_fault(convexIdx_LMT_fault,3), ...
    'ks','MarkerSize',8,'MarkerFaceColor','r');
xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Thrust T (N)');
title('Faulty ACS: (Roll, Pitch, Thrust)');
grid on; view(3); camlight; lighting gouraud;
disp('Faulty convex_vol_LMT = '); disp(convex_vol_LMT_fault);

%% F4. Faulty ACS 시각화: (Roll, Pitch, Yaw)
pts_LMN_fault = ACS_points_fault(:, [2,3,4]);

[hull_LMN_fault, convex_vol_LMN_fault] = convhulln(pts_LMN_fault);
convexIdx_LMN_fault = unique(hull_LMN_fault);
figure('Name','Faulty ACS: Roll, Pitch, Yaw','NumberTitle','off');
trisurf(hull_LMN_fault, pts_LMN_fault(:,1), pts_LMN_fault(:,2), pts_LMN_fault(:,3), ...
    'FaceColor','magenta','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
hold on;
plot3(pts_LMN_fault(:,1), pts_LMN_fault(:,2), pts_LMN_fault(:,3), 'ro','MarkerSize',10);
plot3(pts_LMN_fault(convexIdx_LMN_fault,1), pts_LMN_fault(convexIdx_LMN_fault,2), pts_LMN_fault(convexIdx_LMN_fault,3), ...
    'ks','MarkerSize',8,'MarkerFaceColor','b');
xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Yaw Moment N (Nm)');
title('Faulty ACS: (Roll, Pitch, Yaw)');
grid on; view(3); camlight; lighting gouraud;
disp('Faulty convex_vol_LMN = '); disp(convex_vol_LMN_fault);


%% F5. Faulty H-Representation 도출 (L,M,T)
[A_all_fault, b_all_fault, Aeq_fault_out, beq_fault_out] = vert2lcon(pts_LMT_fault, 1e-10);
disp('Faulty ACS (Roll, Pitch, Thrust)의 부등식 조건:');
disp('Inequality A = '); disp(A_all_fault);
disp('Inequality b = '); disp(b_all_fault);
if ~isempty(Aeq_fault_out)
    disp('Faulty Equality 조건:'); disp(Aeq_fault_out); disp(beq_fault_out);
else
    disp('Faulty Equality 조건은 없습니다.');
end

%% F6. Faulty H-Representation 도출 (L,M,N)
[A_all_fault_LMN, b_all_fault_LMN, Aeq_fault_LMN, beq_fault_LMN] = vert2lcon(pts_LMN_fault, 1e-10);
disp('Faulty ACS (Roll, Pitch, Yaw)의 부등식 조건:');
disp('Inequality A = '); disp(A_all_fault_LMN);
disp('Inequality b = '); disp(b_all_fault_LMN);
if ~isempty(Aeq_fault_LMN)
    disp('Faulty Equality 조건:'); disp(Aeq_fault_LMN); disp(beq_fault_LMN);
else
    disp('Faulty Equality 조건은 없습니다.');
end

%% F7. Faulty H-Representation 도출 (T,L,M,N, 4차원)
pts_TLMN_fault = ACS_points_fault;  % [T,L,M,N]

% 데이터의 중심을 원점으로 이동 (평균 제거)
centered_pts = pts_TLMN_fault - mean(pts_TLMN_fault);

% 행렬의 랭크를 계산하여 실제 차원 확인
data_rank = rank(centered_pts);

disp(['data_rank 데이터의 실제 차원 (Rank): ', num2str(data_rank)]);

if data_rank < 4
    disp('data_rank 데이터의 차원이 4보다 작으므로, 차원 붕괴가 발생했습니다.');
else
    disp('data_rank 데이터는 4차원 공간을 모두 채우고 있습니다.');
end

% try
    % --- [TRY] 4차원 볼록 껍질 계산을 먼저 시도 ---
    disp('4차원 Faulty ACS 계산을 시도합니다...');
    % [hull_TLMN_fault, convex_vol_TLMN_fault] = convhulln(pts_TLMN_fault);

    % convexIdx_TLMN_fault = unique(hull_TLMN_fault);
    [A_all_TLMN_fault, b_all_TLMN_fault, Aeq_all_TLMN_fault, beq_all_TLMN_fault] = vert2lcon(pts_TLMN_fault, 1e-10);
    disp('Faulty ACS (T, L, M, N)의 부등식 조건:');
    disp('Inequality A = '); disp(A_all_TLMN_fault);
    disp('Inequality b = '); disp(b_all_TLMN_fault);
    if ~isempty(Aeq_all_TLMN_fault)
        disp('Faulty Equality 조건:'); 
        disp('Equality Aeq = '); 
        disp(Aeq_all_TLMN_fault); 
        disp('Equality beq = ');
        disp(beq_all_TLMN_fault);
    else
        disp('Faulty Equality 조건은 없습니다.');
    end
    
    % 1. centered_pts 데이터에 PCA를 적용
    %   coeff: 주성분(새로운 축) 방향 벡터들
    %   score: 새로운 축을 기준으로 변환된 데이터 좌표
    %   latent: 각 축의 중요도(분산)
    [coeff, score, latent] = pca(centered_pts);

    % 2. 가장 중요한 3개의 축(PC1, PC2, PC3)에 해당하는 데이터만 선택
    %   이것이 바로 '진정한' 3차원 데이터입니다.
    pts_faulty_pca = score(:, 1:3);
    
    % '진짜' 3차원 데이터로 3D convex hull과 3D 부피를 계산
    [hull_faulty_pca, vol_faulty_pca] = convhulln(pts_faulty_pca);
    
    disp('각 주성분의 중요도(분산):');
    disp(latent);
    fprintf('4번째 주성분의 분산(latent) 값이 거의 0이므로, 데이터가 3차원임을 확인할 수 있습니다.\n');
    % 주성분 축의 의미 분석
    disp('----------------------------------------------------');
    disp('주성분 축 계수 행렬 (Principal Component Loadings):');

    % 보기 쉬운 테이블 형태로 출력
    loadings = array2table(coeff(:,1:3), ...  % 3개의 주성분만 확인
               'VariableNames', {'PC1', 'PC2', 'PC3'}, ...
                'RowNames', {'Thrust', 'Roll', 'Pitch', 'Yaw'});
    disp(loadings);

    fprintf('\n* 해석: 각 PC열에서 절댓값이 가장 큰 행이 해당 축의 주된 의미입니다.\n');
    % ★ 바로 이 값이 우리가 원하는 진짜 3차원 부피입니다.
    disp('Faulty ACS의 실제 3차원 부피 (PCA 기반):');
    disp(vol_faulty_pca);

    % disp('Faulty convex_vol_TLMN = '); disp(convex_vol_TLMN_fault);
% catch ME
%     % --- [CATCH] 4차원 계산이 실패할 경우 (데이터가 평면일 때) ---
%     warning('QHULL 오류: 4차원 볼록 껍질 생성에 실패했습니다. 데이터가 3차원 초평면에 있는 것으로 보입니다.');
%     warning('대신 3차원 (Roll, Pitch, Thrust) 공간에서의 ACS를 계산합니다.');
% 
%     % 비상 대책: Roll, Pitch, Thrust 3차원 공간에서 시각화
%     pts_LMT_fault = ACS_points_fault(:, [2, 3, 1]);  % x=Roll, y=Pitch, z=Thrust
% 
%     if size(pts_LMT_fault, 1) >= 4 % 3D 도형을 만들 최소 점 개수 확인
%         [hull_LMT_fault, convex_vol_LMT_fault] = convhulln(pts_LMT_fault);
% 
%         figure('Name','Faulty ACS (3D Fallback Visualization)','NumberTitle','off');
%         trisurf(hull_LMT_fault, pts_LMT_fault(:,1), pts_LMT_fault(:,2), pts_LMT_fault(:,3), ...
%             'FaceColor','red','FaceAlpha',0.5,'EdgeColor','k');
%         xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Thrust T (N)');
%         title('고장 시 축소된 3D ACS (Roll, Pitch, Thrust)');
%         grid on; view(3); camlight; lighting gouraud;
% 
%         disp('계산된 3차원 Faulty ACS의 부피:');
%         disp(convex_vol_LMT_fault);
%     else
%         disp('3차원 시각화를 위한 점의 개수가 부족합니다.');
%     end
% end


%% F8. Faulty 슬라이싱: N=0 => 3차원 (T,L,M) 시각화
Aeq_N0 = [0 0 0 1]; beq_N0 = 0;

% 데이터 고유의 등식 제약과 슬라이싱 조건을 합칩니다.
% Aeq_combined_N0 = [Aeq_all_TLMN_fault; Aeq_N0];
% beq_combined_N0 = [beq_all_TLMN_fault; beq_N0];
Aeq_combined_N0 = [Aeq_N0];
beq_combined_N0 = [beq_N0];

[V_sliceN0_fault, nr_sliceN0_fault, nre_sliceN0_fault] = lcon2vert(A_all_TLMN_fault, b_all_TLMN_fault, Aeq_combined_N0, beq_combined_N0, 1e-10, true);
if isempty(V_sliceN0_fault)
    disp('Faulty: N=0 슬라이스가 유효하지 않거나, 점이 없습니다.');
else
    sliceN0_RPT_fault = V_sliceN0_fault(:, [2,3,1]);  % x=Roll, y=Pitch, z=Thrust

    if size(sliceN0_RPT_fault,1) < 3
        disp('Faulty: N=0 슬라이스에서 점이 너무 적습니다.');
    else
         try
             [hullN0_RPT_fault, vol_N0_fault] = convhulln(sliceN0_RPT_fault);
             convexIdx_N0_fault = unique(hullN0_RPT_fault);
             figure('Name','Faulty Slice: N=0 => 3D in (Roll, Pitch, Thrust)','NumberTitle','off');
             trisurf(hullN0_RPT_fault, sliceN0_RPT_fault(:,1), sliceN0_RPT_fault(:,2), sliceN0_RPT_fault(:,3), ...
                 'FaceColor','green','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
             hold on;
             plot3(sliceN0_RPT_fault(:,1), sliceN0_RPT_fault(:,2), sliceN0_RPT_fault(:,3), 'bo','MarkerSize',10);
             plot3(sliceN0_RPT_fault(convexIdx_N0_fault,1), sliceN0_RPT_fault(convexIdx_N0_fault,2), sliceN0_RPT_fault(convexIdx_N0_fault,3), ...
                 'ks','MarkerSize',8,'MarkerFaceColor','r');
             xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Thrust T (N)');
             title('Faulty Slice: N=0 (3D) in (Roll, Pitch, Thrust)');
             grid on; view(3); camlight; lighting gouraud;
             disp('Faulty convex_vol_N0 (for N=0 slice) = '); disp(vol_N0_fault);
             
         catch
            % 3D Hull이 실패하면, 데이터가 2D 평면이라고 간주하고 2D로 시각화합니다.
            warning('3D convex hull 실패. 데이터가 2D 평면으로 추정되어 2D로 시각화합니다.');
            
            % Roll-Pitch 평면에 대해 2D convex hull을 계산합니다.
            k_2D = convhull(sliceN0_RPT_fault(:,1), sliceN0_RPT_fault(:,2));
            
            figure('Name','Faulty Slice: N=0 => 2D Visualization','NumberTitle','off');
            plot(sliceN0_RPT_fault(:,1), sliceN0_RPT_fault(:,2), 'bo', 'MarkerFaceColor', 'b');
            hold on;
            plot(sliceN0_RPT_fault(k_2D, 1), sliceN0_RPT_fault(k_2D, 2), 'r-', 'LineWidth', 2);
            title('Faulty Slice: N=0 (2D View on Roll-Pitch Plane)');
            xlabel('Roll Moment (Nm)'); ylabel('Pitch Moment (Nm)');
            grid on; axis equal;
         end
    end
end

%% F9. Faulty 슬라이싱: T=mg, N=0 => 2차원 (L,M) 시각화
Aeq_TN = [1 0 0 0; 0 0 0 1]; beq_TN = [m*g; 0];

% 데이터 고유의 등식 제약과 슬라이싱 조건을 합칩니다.
% Aeq_combined_TN = [Aeq_all_TLMN_fault; Aeq_TN];
% beq_combined_TN = [beq_all_TLMN_fault; beq_TN];
Aeq_combined_TN = [Aeq_TN];
beq_combined_TN = [beq_TN];

[V_sliceTN_fault, nr_sliceTN_fault, nre_sliceTN_fault] = lcon2vert(A_all_TLMN_fault, b_all_TLMN_fault, Aeq_combined_TN, beq_combined_TN, 1e-10, true);
if isempty(V_sliceTN_fault)
    disp('Faulty: T=mg, N=0 슬라이스가 유효하지 않거나, 점이 없습니다.');
else
    sliceTN_LM_fault = V_sliceTN_fault(:, [2,3]);  % (L,M)
    if size(sliceTN_LM_fault,1) < 3
        disp('Faulty: T=mg, N=0 슬라이스에서 점이 너무 적습니다.');
    else
        [hullTN_LM_fault, vol_TN_fault] = convhulln(sliceTN_LM_fault);
        convexIdx_TN_fault = unique(hullTN_LM_fault);
        figure('Name','Faulty Slice: T=mg, N=0 => 2D in (Roll, Pitch)','NumberTitle','off');
        plot(sliceTN_LM_fault(:,1), sliceTN_LM_fault(:,2), 'bo','MarkerSize',10);
        hold on;
        plot(sliceTN_LM_fault(hullTN_LM_fault,1), sliceTN_LM_fault(hullTN_LM_fault,2), 'r-','LineWidth',2);
        plot(sliceTN_LM_fault(convexIdx_TN_fault,1), sliceTN_LM_fault(convexIdx_TN_fault,2), 'ks','MarkerSize',8,'MarkerFaceColor','g');
        xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)');
        title('Faulty Slice: T=mg, N=0 (2D) in (Roll, Pitch)');
        grid on;
        disp('Faulty convex_vol_TN (for T=mg, N=0 slice) = '); disp(vol_TN_fault);
    end
end

%% F10. Faulty 슬라이싱: T=mg => 3차원 (L,M,N) 시각화
Aeq_T = [1 0 0 0]; beq_T = m*g;

% 데이터 고유의 등식 제약과 슬라이싱 조건을 합칩니다.
% Aeq_combined_T = [Aeq_all_TLMN_fault; Aeq_T];
% beq_combined_T = [beq_all_TLMN_fault; beq_T];
Aeq_combined_T = [Aeq_T];
beq_combined_T = [beq_T];

[V_sliceT_fault, nr_sliceT_fault, nre_sliceT_fault] = lcon2vert(A_all_TLMN_fault, b_all_TLMN_fault, Aeq_combined_T, beq_combined_T, 1e-10, true);
if isempty(V_sliceT_fault)
    disp('Faulty: T=mg 슬라이스가 유효하지 않거나, 점이 없습니다.');
else
    sliceT_LMN_fault = V_sliceT_fault(:, [2,3,4]);  % (L,M,N)
    if size(sliceT_LMN_fault,1) < 4
        disp('Faulty: T=mg 슬라이스에서 점이 너무 적습니다.');
    else
        [hullT_LMN_fault, vol_T_fault] = convhulln(sliceT_LMN_fault);
        convexIdx_T_fault = unique(hullT_LMN_fault);
        figure('Name','Faulty Slice: T=mg => 3D in (Roll, Pitch, Yaw)','NumberTitle','off');
        trisurf(hullT_LMN_fault, sliceT_LMN_fault(:,1), sliceT_LMN_fault(:,2), sliceT_LMN_fault(:,3), ...
            'FaceColor','yellow','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
        hold on;
        plot3(sliceT_LMN_fault(:,1), sliceT_LMN_fault(:,2), sliceT_LMN_fault(:,3), 'bo','MarkerSize',10);
        plot3(sliceT_LMN_fault(convexIdx_T_fault,1), sliceT_LMN_fault(convexIdx_T_fault,2), sliceT_LMN_fault(convexIdx_T_fault,3), ...
            'ks','MarkerSize',8,'MarkerFaceColor','m');
        xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Yaw Moment N (Nm)');
        title('Faulty Slice: T=mg (3D) in (Roll, Pitch, Yaw)');
        grid on; view(3); camlight; lighting gouraud;
        disp('Faulty convex_vol_T (for T=mg slice) = '); disp(vol_T_fault);
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% (C) Combined 그래프 5개 (정상 + 고장: 등식 방식)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 먼저 정상 조건 변수들 재정의
pts_LMT_normal = ACS_points_normal(:, [2,3,1]);  % Roll, Pitch, Thrust
[hull_LMT_normal, vol_LMT_normal] = convhulln(pts_LMT_normal);
convexIdx_LMT_normal = unique(hull_LMT_normal);

pts_LMN_normal = ACS_points_normal(:, [2,3,4]);   % Roll, Pitch, Yaw
[hull_LMN_normal, vol_LMN_normal] = convhulln(pts_LMN_normal);
convexIdx_LMN_normal = unique(hull_LMN_normal);

pts_TLMN_normal = ACS_points_normal;              % [T,L,M,N]
[A_all_TLMN_normal, b_all_TLMN_normal, Aeq_all_TLMN_normal, beq_all_TLMN_normal] = vert2lcon(pts_TLMN_normal, 1e-10);

% 고장 조건 변수들
pts_LMT_fault = ACS_points_fault(:, [2,3,1]);  % Roll, Pitch, Thrust
[hull_LMT_fault, vol_LMT_fault] = convhulln(pts_LMT_fault);
convexIdx_LMT_fault = unique(hull_LMT_fault);

pts_LMN_fault = ACS_points_fault(:, [2,3,4]);  % Roll, Pitch, Yaw
[hull_LMN_fault, vol_LMN_fault] = convhulln(pts_LMN_fault);
convexIdx_LMN_fault = unique(hull_LMN_fault);

pts_TLMN_fault = ACS_points_fault;             % [T,L,M,N]
[A_all_TLMN_fault, b_all_TLMN_fault, Aeq_all_TLMN_fault, beq_all_TLMN_fault] = vert2lcon(pts_TLMN_fault, 1e-10);

% Combined Graphs

%% C1) Combined ACS: (Roll, Pitch, Thrust)
figure('Name','(C1) Combined: Roll, Pitch, Thrust','NumberTitle','off');
hold on;
% 정상
trisurf(hull_LMT_normal, pts_LMT_normal(:,1), pts_LMT_normal(:,2), pts_LMT_normal(:,3), ...
    'FaceColor','cyan','FaceAlpha',0.3,'EdgeColor','none');
plot3(pts_LMT_normal(:,1), pts_LMT_normal(:,2), pts_LMT_normal(:,3), 'bo','MarkerSize',10);
% 고장
trisurf(hull_LMT_fault, pts_LMT_fault(:,1), pts_LMT_fault(:,2), pts_LMT_fault(:,3), ...
    'FaceColor','magenta','FaceAlpha',0.3,'EdgeColor','none');
plot3(pts_LMT_fault(:,1), pts_LMT_fault(:,2), pts_LMT_fault(:,3), 'ro','MarkerSize',10);
xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Thrust T (N)');
title('(C1) Combined ACS: (Roll, Pitch, Thrust)');
grid on; view(3); camlight; lighting gouraud;
legend('Normal surface','Normal pts','Faulty surface','Faulty pts','Location','best');

%% C2) Combined ACS: (Roll, Pitch, Yaw)
figure('Name','(C2) Combined: Roll, Pitch, Yaw','NumberTitle','off');
hold on;
% 정상
trisurf(hull_LMN_normal, pts_LMN_normal(:,1), pts_LMN_normal(:,2), pts_LMN_normal(:,3), ...
    'FaceColor','cyan','FaceAlpha',0.3,'EdgeColor','none');
plot3(pts_LMN_normal(:,1), pts_LMN_normal(:,2), pts_LMN_normal(:,3), 'bo','MarkerSize',10);
% 고장
trisurf(hull_LMN_fault, pts_LMN_fault(:,1), pts_LMN_fault(:,2), pts_LMN_fault(:,3), ...
    'FaceColor','magenta','FaceAlpha',0.3,'EdgeColor','none');
plot3(pts_LMN_fault(:,1), pts_LMN_fault(:,2), pts_LMN_fault(:,3), 'ro','MarkerSize',10);
xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Yaw Moment N (Nm)');
title('(C2) Combined ACS: (Roll, Pitch, Yaw)');
grid on; view(3); camlight; lighting gouraud;
legend('Normal surface','Normal pts','Faulty surface','Faulty pts','Location','best');

%% C3) Combined Slice: N=0 => 3D (T,L,M)
Aeq_N0 = [0 0 0 1]; beq_N0 = 0;
[V_sliceN0_normal, ~, ~] = lcon2vert(A_all_TLMN_normal, b_all_TLMN_normal, Aeq_N0, beq_N0, 1e-10, true);
[V_sliceN0_fault, ~, ~]  = lcon2vert(A_all_TLMN_fault, b_all_TLMN_fault, Aeq_N0, beq_N0, 1e-10, true);
figure('Name','(C3) Combined Slice: N=0 => 3D (T,L,M)','NumberTitle','off');
hold on;
if ~isempty(V_sliceN0_normal)
    ptsN0_norm = V_sliceN0_normal(:, [2,3,1]);  % x=Roll, y=Pitch, z=Thrust
    [hullN0_norm, volN0_norm] = convhulln(ptsN0_norm);
    trisurf(hullN0_norm, ptsN0_norm(:,1), ptsN0_norm(:,2), ptsN0_norm(:,3), ...
        'FaceColor','cyan','FaceAlpha',0.3,'EdgeColor','none');
    plot3(ptsN0_norm(:,1), ptsN0_norm(:,2), ptsN0_norm(:,3), 'bo','MarkerSize',10);
end
if ~isempty(V_sliceN0_fault)
    ptsN0_faul = V_sliceN0_fault(:, [2,3,1]);
    [hullN0_faul, volN0_faul] = convhulln(ptsN0_faul);
    trisurf(hullN0_faul, ptsN0_faul(:,1), ptsN0_faul(:,2), ptsN0_faul(:,3), ...
        'FaceColor','magenta','FaceAlpha',0.3,'EdgeColor','none');
    plot3(ptsN0_faul(:,1), ptsN0_faul(:,2), ptsN0_faul(:,3), 'ro','MarkerSize',10);
end
xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Thrust T (N)');
title('(C3) Combined Slice: N=0 => (T,L,M)');
grid on; view(3); camlight; lighting gouraud;
legend('Normal surface','Normal pts','Faulty surface','Faulty pts','Location','best');

%% C4) Combined Slice: T=mg, N=0 => 2D (L,M)
Aeq_TN = [1 0 0 0; 0 0 0 1]; 
beq_TN = [m*g; 0];
[V_sliceTN_normal, ~, ~] = lcon2vert(A_all_TLMN_normal, b_all_TLMN_normal, Aeq_TN, beq_TN, 1e-10, true);
[V_sliceTN_fault, ~, ~]  = lcon2vert(A_all_TLMN_fault, b_all_TLMN_fault, Aeq_TN, beq_TN, 1e-10, true);
figure('Name','(C4) Combined Slice: T=mg, N=0 => 2D (L,M)','NumberTitle','off');
hold on;
if ~isempty(V_sliceTN_normal)
    ptsTN_norm = V_sliceTN_normal(:, [2,3]);
    [hullTN_norm, volTN_norm] = convhulln(ptsTN_norm);
    plot(ptsTN_norm(:,1), ptsTN_norm(:,2), 'bo','MarkerSize',10);
    plot(ptsTN_norm(hullTN_norm,1), ptsTN_norm(hullTN_norm,2), 'b-','LineWidth',2);
end
if ~isempty(V_sliceTN_fault)
    ptsTN_faul = V_sliceTN_fault(:, [2,3]);
    [hullTN_faul, volTN_faul] = convhulln(ptsTN_faul);
    plot(ptsTN_faul(:,1), ptsTN_faul(:,2), 'ro','MarkerSize',10);
    plot(ptsTN_faul(hullTN_faul,1), ptsTN_faul(hullTN_faul,2), 'r-','LineWidth',2);
end
xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)');
title('(C4) Combined Slice: T=mg, N=0 => (L,M)');
grid on;
legend('Normal pts','Normal hull','Faulty pts','Faulty hull','Location','best');

%% C5) Combined Slice: T=mg => 3D (L,M,N)
Aeq_T = [1 0 0 0]; beq_T = m*g;
[V_sliceT_normal, ~, ~] = lcon2vert(A_all_TLMN_normal, b_all_TLMN_normal, Aeq_T, beq_T, 1e-10, true);
[V_sliceT_fault, ~, ~]  = lcon2vert(A_all_TLMN_fault, b_all_TLMN_fault, Aeq_T, beq_T, 1e-10, true);
figure('Name','(C5) Combined Slice: T=mg => 3D (L,M,N)','NumberTitle','off');
hold on;
if ~isempty(V_sliceT_normal)
    ptsT_norm = V_sliceT_normal(:, [2,3,4]);
    [hullT_norm, volT_norm] = convhulln(ptsT_norm);
    trisurf(hullT_norm, ptsT_norm(:,1), ptsT_norm(:,2), ptsT_norm(:,3), ...
        'FaceColor','cyan','FaceAlpha',0.3,'EdgeColor','none');
    plot3(ptsT_norm(:,1), ptsT_norm(:,2), ptsT_norm(:,3), 'bo','MarkerSize',10);
end
if ~isempty(V_sliceT_fault)
    ptsT_faul = V_sliceT_fault(:, [2,3,4]);
    [hullT_faul, volT_faul] = convhulln(ptsT_faul);
    trisurf(hullT_faul, ptsT_faul(:,1), ptsT_faul(:,2), ptsT_faul(:,3), ...
        'FaceColor','magenta','FaceAlpha',0.3,'EdgeColor','none');
    plot3(ptsT_faul(:,1), ptsT_faul(:,2), ptsT_faul(:,3), 'ro','MarkerSize',10);
end
xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Yaw Moment N (Nm)');
title('(C5) Combined Slice: T=mg => (L,M,N)');
grid on; view(3); camlight; lighting gouraud;
legend('Normal surface','Normal pts','Faulty surface','Faulty pts','Location','best');