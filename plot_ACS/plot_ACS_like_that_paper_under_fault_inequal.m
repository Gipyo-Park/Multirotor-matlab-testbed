clear all; clc; close all;

%% 설정 변수
n = 8;                      % 모터(액추에이터) 개수
w_min_val = 0;
w_max_val = 425.47;         % 액추에이터 각속도 [rad/sec]
m = 20;                     % 비행체 질량 (kg)
g = 9.81;                   % 중력가속도 (m/s^2)

% Inject_Fault: 고장 날 액추에이터 번호 및 고장 정도 (0: 정상, 1: 완전 고장, 중간값: 부분 고장)
% 예를 들어, 1번 액추에이터는 완전 고장, 4번은 50% 고장, 8번은 75% 고장
Inject_Fault = [1];
Fault_Level = [0.999];  % Inject_Fault와 같은 순서

%% [Normal Case] 정상 조건의 8차원 박스 생성
% 각 액추에이터의 상한은 모두 정상: w_max_val
A_box = [ eye(n); -eye(n) ];
b_box = [ w_max_val * ones(n,1); -w_min_val * ones(n,1) ];
[V_box, nr_box, nre_box] = lcon2vert(A_box, b_box, [], [], 1e-10, true);
disp('정상 조건: 8차원 박스의 극점 (V_box):');
disp(V_box);

%% [Fault Injection Case] 고장 조건 적용 (부등식으로 상한 조절)
% 각 액추에이터별 fault level을 0으로 초기화 (정상은 0)
fault_levels = zeros(n,1);
% Inject_Fault에 해당하는 인덱스에 대해 fault level 할당
for i = 1:length(Inject_Fault)
    fault_levels(Inject_Fault(i)) = Fault_Level(i);
end
% 고장 조건에서 각 액추에이터의 최대값은 (1 - fault_level)*w_max_val
w_max_fault = (1 - fault_levels) .* w_max_val;  % 벡터 (n x 1)

% 고장 조건에 맞게 박스 제약 재구성 (하한은 그대로)
A_box_fault = [ eye(n); -eye(n) ];
b_box_fault = [ w_max_fault; -w_min_val * ones(n,1) ];
[V_box_fault, nr_box_fault, nre_box_fault] = lcon2vert(A_box_fault, b_box_fault, [], [], 1e-10, true);
disp('고장 조건: 8차원 박스의 극점 (V_box_fault):');
disp(V_box_fault);

%% 3. 제어 효과 행렬 K 적용 (정상 및 고장 조건 모두 동일)
% 출력: [Thrust T, Roll L, Pitch M, Yaw N]
% 단위: T (N), L/M/N (Nm)
b_  = -0.00053;
b = b_ * w_max_val;
d_  = 0.0000169;
d = d_ * w_max_val;
L1 = 0.545;  L2 = 0.36;  L3 = 1.08;

K = [ -b,    -b,    -b,    -b,    -b,    -b,    -b,    -b;
      -b*L1,  b*L1, -b*L1, -b*L1,  b*L1,  b*L1,  b*L1, -b*L1;
       b*L3, -b*L3,  b*L2, -b*L3,  b*L3, -b*L2,  b*L2, -b*L2;
      -d,    -d,     d,     d,     d,     d,    -d,    -d ];

ACS_points_normal = (K * V_box')';       % 정상 조건: [T,L,M,N]
ACS_points_fault  = (K * V_box_fault')';  % 고장 조건: [T,L,M,N]

%% 4. 정상 ACS 시각화: (Roll, Pitch, Thrust)
pts_LMT = ACS_points_normal(:, [2,3,1]);   % x=Roll, y=Pitch, z=Thrust
[hull_LMT, convex_vol_LMT] = convhulln(pts_LMT);
convexIdx_LMT = unique(hull_LMT);
figure('Name','Normal ACS: Roll, Pitch, Thrust','NumberTitle','off');
trisurf(hull_LMT, pts_LMT(:,1), pts_LMT(:,2), pts_LMT(:,3), ...
    'FaceColor','cyan','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
hold on;
plot3(pts_LMT(:,1), pts_LMT(:,2), pts_LMT(:,3), 'bo','MarkerSize',3);
plot3(pts_LMT(convexIdx_LMT,1), pts_LMT(convexIdx_LMT,2), pts_LMT(convexIdx_LMT,3), ...
    'ks','MarkerSize',8,'MarkerFaceColor','r');
xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Thrust T (N)');
title('Normal ACS: (Roll, Pitch, Thrust)');
grid on; view(3); camlight; lighting gouraud;
disp('Normal convex_vol_LMT = '); disp(convex_vol_LMT);

%% 5. 정상 ACS 시각화: (Roll, Pitch, Yaw)
pts_LMN = ACS_points_normal(:, [2,3,4]);   % x=Roll, y=Pitch, z=Yaw
[hull_LMN, convex_vol_LMN] = convhulln(pts_LMN);
convexIdx_LMN = unique(hull_LMN);
figure('Name','Normal ACS: Roll, Pitch, Yaw','NumberTitle','off');
trisurf(hull_LMN, pts_LMN(:,1), pts_LMN(:,2), pts_LMN(:,3), ...
    'FaceColor','magenta','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
hold on;
plot3(pts_LMN(:,1), pts_LMN(:,2), pts_LMN(:,3), 'ro','MarkerSize',3);
plot3(pts_LMN(convexIdx_LMN,1), pts_LMN(convexIdx_LMN,2), pts_LMN(convexIdx_LMN,3), ...
    'ks','MarkerSize',8,'MarkerFaceColor','b');
xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Yaw Moment N (Nm)');
title('Normal ACS: (Roll, Pitch, Yaw)');
grid on; view(3); camlight; lighting gouraud;
disp('Normal convex_vol_LMN = '); disp(convex_vol_LMN);

%% 6. 정상 H-Representation 도출 (L,M,T)
[A_all, b_all, Aeq_all, beq_all] = vert2lcon(pts_LMT, 1e-10);
disp('Normal ACS (Roll, Pitch, Thrust)의 부등식 조건:');
disp('Inequality A = '); disp(A_all);
disp('Inequality b = '); disp(b_all);
if ~isempty(Aeq_all)
    disp('Equality 조건:'); disp(Aeq_all); disp(beq_all);
else
    disp('Equality 조건은 없습니다.');
end

%% 7. 정상 H-Representation 도출 (L,M,N)
[A_all_LMN, b_all_LMN, Aeq_all_LMN, beq_all_LMN] = vert2lcon(pts_LMN, 1e-10);
disp('Normal ACS (Roll, Pitch, Yaw)의 부등식 조건:');
disp('Inequality A = '); disp(A_all_LMN);
disp('Inequality b = '); disp(b_all_LMN);
if ~isempty(Aeq_all_LMN)
    disp('Equality 조건:'); disp(Aeq_all_LMN); disp(beq_all_LMN);
else
    disp('Equality 조건은 없습니다.');
end

%% 8. 정상 H-Representation 도출 (T,L,M,N, 4차원)
pts_TLMN = ACS_points_normal;  % [T,L,M,N]
[hull_TLMN, convex_vol_TLMN] = convhulln(pts_TLMN);
convexIdx_TLMN = unique(hull_TLMN);
[A_all_TLMN, b_all_TLMN, Aeq_all_TLMN, beq_all_TLMN] = vert2lcon(pts_TLMN, 1e-10);
disp('Normal ACS (T, L, M, N)의 부등식 조건:');
disp('Inequality A = '); disp(A_all_TLMN);
disp('Inequality b = '); disp(b_all_TLMN);
if ~isempty(Aeq_all_TLMN)
    disp('Equality 조건:'); disp(Aeq_all_TLMN); disp(beq_all_TLMN);
else
    disp('Equality 조건은 없습니다.');
end
disp('Normal convex_vol_TLMN = '); disp(convex_vol_TLMN);

%% 9. 정상 슬라이싱: N=0 => 3차원 (T,L,M) 시각화
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
        plot3(sliceN0_RPT(:,1), sliceN0_RPT(:,2), sliceN0_RPT(:,3), 'bo','MarkerSize',3);
        plot3(sliceN0_RPT(convexIdx_N0,1), sliceN0_RPT(convexIdx_N0,2), sliceN0_RPT(convexIdx_N0,3), ...
            'ks','MarkerSize',8,'MarkerFaceColor','r');
        xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Thrust T (N)');
        title('Normal Slice: N=0 (3D) in (Roll, Pitch, Thrust)');
        grid on; view(3); camlight; lighting gouraud;
        disp('Normal convex_vol_N0 = '); disp(vol_N0);
    end
end

%% 10. 정상 슬라이싱: T=mg, N=0 => 2차원 (L,M) 시각화
Aeq_TN = [1 0 0 0; 0 0 0 1]; beq_TN = [m*g; 0];
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
        plot(sliceTN_LM(:,1), sliceTN_LM(:,2), 'bo','MarkerSize',4);
        hold on;
        plot(sliceTN_LM(hullTN_LM,1), sliceTN_LM(hullTN_LM,2), 'r-','LineWidth',2);
        plot(sliceTN_LM(convexIdx_TN,1), sliceTN_LM(convexIdx_TN,2), 'ks','MarkerSize',8,'MarkerFaceColor','g');
        xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)');
        title('Normal Slice: T=mg, N=0 (2D) in (Roll, Pitch)');
        grid on;
        disp('Normal convex_vol_TN = '); disp(vol_TN);
    end
end

%% 11. 정상 슬라이싱: T=mg => 3차원 (L,M,N) 시각화
Aeq_T = [1 0 0 0]; beq_T = m*g;
[V_sliceT, nr_sliceT, nre_sliceT] = lcon2vert(A_all_TLMN, b_all_TLMN, Aeq_T, beq_T, 1e-10, true);
if isempty(V_sliceT)
    disp('Normal: T=mg 슬라이스가 유효하지 않거나, 점이 없습니다.');
else
    sliceT_LMN = V_sliceT(:, [2,3,4]);  % (L,M,N)
    if size(sliceT_LMN,1) < 4
        disp('Normal: T=mg 슬라이싱에서 점이 너무 적습니다.');
    else
        [hullT_LMN, vol_T] = convhulln(sliceT_LMN);
        convexIdx_T = unique(hullT_LMN);
        figure('Name','Normal Slice: T=mg => 3D in (Roll, Pitch, Yaw)','NumberTitle','off');
        trisurf(hullT_LMN, sliceT_LMN(:,1), sliceT_LMN(:,2), sliceT_LMN(:,3), ...
            'FaceColor','yellow','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
        hold on;
        plot3(sliceT_LMN(:,1), sliceT_LMN(:,2), sliceT_LMN(:,3), 'bo','MarkerSize',3);
        plot3(sliceT_LMN(convexIdx_T,1), sliceT_LMN(convexIdx_T,2), sliceT_LMN(convexIdx_T,3), ...
            'ks','MarkerSize',8,'MarkerFaceColor','m');
        xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Yaw Moment N (Nm)');
        title('Normal Slice: T=mg (3D) in (Roll, Pitch, Yaw)');
        grid on; view(3); camlight; lighting gouraud;
        disp('Normal convex_vol_T (for T=mg slice) = '); disp(vol_T);
    end
end

%% [Fault Injection Case] 고장 조건 적용 (부등식으로 상한 조절)
% 각 액추에이터별 fault level (0: 정상, 1: 완전 고장, 중간값: 부분 고장)
fault_levels = zeros(n,1);
for i = 1:length(Inject_Fault)
    fault_levels(Inject_Fault(i)) = Fault_Level(find(Inject_Fault==Inject_Fault(i),1));
end
w_max_fault = (1 - fault_levels) .* w_max_val;  % 고장 조건에서 상한

A_box_fault = [ eye(n); -eye(n) ];
b_box_fault = [ w_max_fault; -w_min_val * ones(n,1) ];
[V_box_fault, nr_box_fault, nre_box_fault] = lcon2vert(A_box_fault, b_box_fault, [], [], 1e-10, true);
disp('Fault Injection: 8차원 박스의 극점 (V_box_fault):');
disp(V_box_fault);

%% Faulty ACS_points 계산
ACS_points_fault = (K * V_box_fault')';   % [T,L,M,N] 고장 조건

%% F3. Faulty ACS 시각화: (Roll, Pitch, Thrust)
pts_LMT_fault = ACS_points_fault(:, [2,3,1]);
[hull_LMT_fault, convex_vol_LMT_fault] = convhulln(pts_LMT_fault);
convexIdx_LMT_fault = unique(hull_LMT_fault);
figure('Name','Faulty ACS: Roll, Pitch, Thrust','NumberTitle','off');
trisurf(hull_LMT_fault, pts_LMT_fault(:,1), pts_LMT_fault(:,2), pts_LMT_fault(:,3), ...
    'FaceColor','cyan','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
hold on;
plot3(pts_LMT_fault(:,1), pts_LMT_fault(:,2), pts_LMT_fault(:,3), 'bo','MarkerSize',3);
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
plot3(pts_LMN_fault(:,1), pts_LMN_fault(:,2), pts_LMN_fault(:,3), 'ro','MarkerSize',3);
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
    disp('Equality 조건:'); disp(Aeq_fault_out); disp(beq_fault_out);
else
    disp('Equality 조건은 없습니다.');
end

%% F6. Faulty H-Representation 도출 (L,M,N)
[A_all_fault_LMN, b_all_fault_LMN, Aeq_fault_LMN, beq_fault_LMN] = vert2lcon(pts_LMN_fault, 1e-10);
disp('Faulty ACS (Roll, Pitch, Yaw)의 부등식 조건:');
disp('Inequality A = '); disp(A_all_fault_LMN);
disp('Inequality b = '); disp(b_all_fault_LMN);
if ~isempty(Aeq_fault_LMN)
    disp('Equality 조건:'); disp(Aeq_fault_LMN); disp(beq_fault_LMN);
else
    disp('Equality 조건은 없습니다.');
end

%% F7. Faulty H-Representation 도출 (T,L,M,N, 4차원)
pts_TLMN_fault = ACS_points_fault;  % [T,L,M,N]
[hull_TLMN_fault, convex_vol_TLMN_fault] = convhulln(pts_TLMN_fault);
convexIdx_TLMN_fault = unique(hull_TLMN_fault);
[A_all_TLMN_fault, b_all_TLMN_fault, Aeq_all_TLMN_fault, beq_all_TLMN_fault] = vert2lcon(pts_TLMN_fault, 1e-10);
disp('Faulty ACS (T, L, M, N)의 부등식 조건:');
disp('Inequality A = '); disp(A_all_TLMN_fault);
disp('Inequality b = '); disp(b_all_TLMN_fault);
if ~isempty(Aeq_all_TLMN_fault)
    disp('Equality 조건:'); disp(Aeq_all_TLMN_fault); disp(beq_all_TLMN_fault);
else
    disp('Equality 조건은 없습니다.');
end
disp('Faulty convex_vol_TLMN = '); disp(convex_vol_TLMN_fault);

%% F8. Faulty 슬라이싱: N=0 => 3차원 (T,L,M) 시각화
Aeq_N0 = [0 0 0 1]; beq_N0 = 0;
[V_sliceN0_fault, nr_sliceN0_fault, nre_sliceN0_fault] = lcon2vert(A_all_TLMN_fault, b_all_TLMN_fault, Aeq_N0, beq_N0, 1e-10, true);
if isempty(V_sliceN0_fault)
    disp('Faulty: N=0 슬라이스가 유효하지 않거나, 점이 없습니다.');
else
    sliceN0_RPT_fault = V_sliceN0_fault(:, [2,3,1]);  % x=Roll, y=Pitch, z=Thrust
    if size(sliceN0_RPT_fault,1) < 4
        disp('Faulty: N=0 슬라이스에서 점이 너무 적습니다.');
    else
        [hullN0_RPT_fault, vol_N0_fault] = convhulln(sliceN0_RPT_fault);
        convexIdx_N0_fault = unique(hullN0_RPT_fault);
        figure('Name','Faulty Slice: N=0 => 3D in (Roll, Pitch, Thrust)','NumberTitle','off');
        trisurf(hullN0_RPT_fault, sliceN0_RPT_fault(:,1), sliceN0_RPT_fault(:,2), sliceN0_RPT_fault(:,3), ...
            'FaceColor','green','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
        hold on;
        plot3(sliceN0_RPT_fault(:,1), sliceN0_RPT_fault(:,2), sliceN0_RPT_fault(:,3), 'bo','MarkerSize',3);
        plot3(sliceN0_RPT_fault(convexIdx_N0_fault,1), sliceN0_RPT_fault(convexIdx_N0_fault,2), sliceN0_RPT_fault(convexIdx_N0_fault,3), ...
            'ks','MarkerSize',8,'MarkerFaceColor','r');
        xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Thrust T (N)');
        title('Faulty Slice: N=0 (3D) in (Roll, Pitch, Thrust)');
        grid on; view(3); camlight; lighting gouraud;
        disp('Faulty convex_vol_N0 = '); disp(vol_N0_fault);
    end
end

%% F9. Faulty 슬라이싱: T=mg, N=0 => 2차원 (L,M) 시각화
Aeq_TN = [1 0 0 0; 0 0 0 1]; beq_TN = [m*g; 0];
[V_sliceTN_fault, nr_sliceTN_fault, nre_sliceTN_fault] = lcon2vert(A_all_TLMN_fault, b_all_TLMN_fault, Aeq_TN, beq_TN, 1e-10, true);
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
        plot(sliceTN_LM_fault(:,1), sliceTN_LM_fault(:,2), 'bo','MarkerSize',4);
        hold on;
        plot(sliceTN_LM_fault(hullTN_LM_fault,1), sliceTN_LM_fault(hullTN_LM_fault,2), 'r-','LineWidth',2);
        plot(sliceTN_LM_fault(convexIdx_TN_fault,1), sliceTN_LM_fault(convexIdx_TN_fault,2), 'ks','MarkerSize',8,'MarkerFaceColor','g');
        xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)');
        title('Faulty Slice: T=mg, N=0 (2D) in (Roll, Pitch)');
        grid on;
        disp('Faulty convex_vol_TN = '); disp(vol_TN_fault);
    end
end

%% F10. Faulty 슬라이싱: T=mg => 3차원 (L,M,N) 시각화
Aeq_T = [1 0 0 0]; beq_T = m*g;
[V_sliceT_fault, nr_sliceT_fault, nre_sliceT_fault] = lcon2vert(A_all_TLMN_fault, b_all_TLMN_fault, Aeq_T, beq_T, 1e-10, true);
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
        plot3(sliceT_LMN_fault(:,1), sliceT_LMN_fault(:,2), sliceT_LMN_fault(:,3), 'bo','MarkerSize',3);
        plot3(sliceT_LMN_fault(convexIdx_T_fault,1), sliceT_LMN_fault(convexIdx_T_fault,2), sliceT_LMN_fault(convexIdx_T_fault,3), ...
            'ks','MarkerSize',8,'MarkerFaceColor','m');
        xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Yaw Moment N (Nm)');
        title('Faulty Slice: T=mg (3D) in (Roll, Pitch, Yaw)');
        grid on; view(3); camlight; lighting gouraud;
        disp('Faulty convex_vol_T (for T=mg slice) = '); disp(vol_T_fault);
    end
end
