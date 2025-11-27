clear all; clc; close all;

%% 1. 8차원 박스(모터 입력 범위) 극점 구하기 using lcon2vert
n = 8;  % 모터 개수
w_min_val = 0;
w_max_val = 425.47;   % 액츄에이터 각속도 [rad/sec]
m = 20;               % 비행체 질량 (예: 20kg)
g = 9.81;             % 중력가속도 (9.81 m/s^2)



A_box = [ eye(n);    % w_i <= w_max
         -eye(n)];   % -w_i <= -w_min
b_box = [ w_max_val * ones(n,1);
         -w_min_val * ones(n,1)];


[V_box, nr_box, nre_box] = lcon2vert(A_box, b_box, [], [], 1e-10, true);
disp('8차원 박스의 극점 (V_box):');
disp(V_box);

%% 2. 제어 효과 행렬 K 적용 
%  - 출력: [Thrust T, Roll L, Pitch M, Yaw N]
%  - 단위: T (N), L/M/N (Nm)
b_  = -0.00053;     % [kg*m] = [N*s^2/rad^2]
b = b_ * w_max_val;
d_  = 0.0000169;    % [N*m*s^2/rad^2]
d = d_ * w_max_val;
L1 = 0.545;  
L2 = 0.36;   
L3 = 1.08;   

K = [ ...
    -b,    -b,    -b,    -b,    -b,    -b,    -b,    -b;       % T: Thrust (N)
    -b*L1,  b*L1, -b*L1, -b*L1,  b*L1,  b*L1,  b*L1, -b*L1;     % L: Roll Moment (Nm)
     b*L3, -b*L3,  b*L2, -b*L3,  b*L3, -b*L2,  b*L2, -b*L2;     % M: Pitch Moment (Nm)
    -d,    -d,     d,     d,     d,     d,    -d,    -d        % N: Yaw Moment (Nm)
];

ACS_points = (K * V_box')';   % (num_vertices x 4) : [T, L, M, N]

%% 3. ACS 시각화: (Roll, Pitch, Thrust)
% pts_LMT: x = Roll (열2), y = Pitch (열3), z = Thrust (열1)
pts_LMT = ACS_points(:, [2, 3, 1]);
[hull_LMT, convex_vol_LMT] = convhulln(pts_LMT);
convexIdx_LMT = unique(hull_LMT);

figure('Name','ACS: Roll, Pitch, Thrust','NumberTitle','off');
trisurf(hull_LMT, pts_LMT(:,1), pts_LMT(:,2), pts_LMT(:,3), ...
    'FaceColor','cyan','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
hold on;
plot3(pts_LMT(:,1), pts_LMT(:,2), pts_LMT(:,3), 'bo', 'MarkerSize', 3);
plot3(pts_LMT(convexIdx_LMT,1), pts_LMT(convexIdx_LMT,2), pts_LMT(convexIdx_LMT,3), ...
    'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('Roll Moment L (Nm)');
ylabel('Pitch Moment M (Nm)');
zlabel('Thrust T (N)');
title('ACS: (Roll L, Pitch M, Thrust T)');
grid on; view(3); camlight; lighting gouraud;
disp('convex_vol_LMT = ');
disp(convex_vol_LMT);

%% 4. ACS 시각화: (Roll, Pitch, Yaw)
% pts_LMN: x = Roll (열2), y = Pitch (열3), z = Yaw (열4)
pts_LMN = ACS_points(:, [2, 3, 4]);
[hull_LMN, convex_vol_LMN] = convhulln(pts_LMN);
convexIdx_LMN = unique(hull_LMN);

figure('Name','ACS: Roll, Pitch, Yaw','NumberTitle','off');
trisurf(hull_LMN, pts_LMN(:,1), pts_LMN(:,2), pts_LMN(:,3), ...
    'FaceColor','magenta','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
hold on;
plot3(pts_LMN(:,1), pts_LMN(:,2), pts_LMN(:,3), 'ro', 'MarkerSize', 3);
plot3(pts_LMN(convexIdx_LMN,1), pts_LMN(convexIdx_LMN,2), pts_LMN(convexIdx_LMN,3), ...
    'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
xlabel('Roll Moment L (Nm)');
ylabel('Pitch Moment M (Nm)');
zlabel('Yaw Moment N (Nm)');
title('ACS: (Roll L, Pitch M, Yaw N)');
grid on; view(3); camlight; lighting gouraud;
disp('convex_vol_LMN = ');
disp(convex_vol_LMN);

%% 5. H-Representation 도출 (L,M,T)
[A_all, b_all, Aeq_all, beq_all] = vert2lcon(pts_LMT, 1e-10);
disp('전체 ACS (Roll, Pitch, Thrust)의 부등식 조건 A*x <= b:');
disp('Inequality A = '); disp(A_all);
disp('Inequality b = '); disp(b_all);
if ~isempty(Aeq_all)
    disp('Equality 조건 Aeq*x = beq:');
    disp(Aeq_all); disp(beq_all);
else
    disp('Equality 조건은 없습니다.');
end

V_convex = pts_LMT(convexIdx_LMT, :);
[A_convex, b_convex, Aeq_convex, beq_convex] = vert2lcon(V_convex, 1e-10);
disp('Convex hull (Roll, Pitch, Thrust)의 부등식 조건 A*x <= b:');
disp('Inequality A = '); disp(A_convex);
disp('Inequality b = '); disp(b_convex);
if ~isempty(Aeq_convex)
    disp('Convex hull의 등식 조건 Aeq*x = beq:');
    disp(Aeq_convex); disp(beq_convex);
else
    disp('Equality 조건은 없습니다.');
end

%% 6. H-Representation 도출 (L,M,N)
[A_all_LMN, b_all_LMN, Aeq_all_LMN, beq_all_LMN] = vert2lcon(pts_LMN, 1e-10);
disp('전체 ACS (Roll, Pitch, Yaw)의 부등식 조건 A*x <= b:');
disp('Inequality A = '); disp(A_all_LMN);
disp('Inequality b = '); disp(b_all_LMN);
if ~isempty(Aeq_all_LMN)
    disp('Equality 조건 Aeq*x = beq:');
    disp(Aeq_all_LMN); disp(beq_all_LMN);
else
    disp('Equality 조건은 없습니다.');
end

V_convex_LMN = pts_LMN(convexIdx_LMN, :);
[A_convex_LMN, b_convex_LMN, Aeq_convex_LMN, beq_convex_LMN] = vert2lcon(V_convex_LMN, 1e-10);
disp('Convex hull (Roll, Pitch, Yaw)의 부등식 조건 A*x <= b:');
disp('Inequality A = '); disp(A_convex_LMN);
disp('Inequality b = '); disp(b_convex_LMN);
if ~isempty(Aeq_convex_LMN)
    disp('Convex hull의 등식 조건 Aeq*x = beq:');
    disp(Aeq_convex_LMN); disp(beq_convex_LMN);
else
    disp('Equality 조건은 없습니다.');
end

%% 7. H-Representation 도출 (T,L,M,N, 4차원)
pts_TLMN = ACS_points;  % [T,L,M,N]
[hull_TLMN, convex_vol_TLMN] = convhulln(pts_TLMN);
convexIdx_TLMN = unique(hull_TLMN);

[A_all_TLMN, b_all_TLMN, Aeq_all_TLMN, beq_all_TLMN] = vert2lcon(pts_TLMN, 1e-10);
disp('전체 ACS (T, L, M, N)의 부등식 조건 A*x <= b:');
disp('Inequality A = '); disp(A_all_TLMN);
disp('Inequality b = '); disp(b_all_TLMN);
if ~isempty(Aeq_all_TLMN)
    disp('Equality 조건 Aeq*x = beq:');
    disp(Aeq_all_TLMN); disp(beq_all_TLMN);
else
    disp('Equality 조건은 없습니다.');
end

V_convex_TLMN = pts_TLMN(convexIdx_TLMN, :);
[A_convex_TLMN, b_convex_TLMN, Aeq_convex_TLMN, beq_convex_TLMN] = vert2lcon(V_convex_TLMN, 1e-10);
disp('Convex hull (T, L, M, N)의 부등식 조건 A*x <= b:');
disp('Inequality A = '); disp(A_convex_TLMN);
disp('Inequality b = '); disp(b_convex_TLMN);
if ~isempty(Aeq_convex_TLMN)
    disp('Convex hull의 등식 조건 Aeq*x = beq:');
    disp(Aeq_convex_TLMN); disp(beq_convex_TLMN);
else
    disp('Equality 조건은 없습니다.');
end
disp('convex_vol_TLMN = ');
disp(convex_vol_TLMN);

%% 8. 슬라이싱 예시: N=0 => 3차원 (T,L,M) 시각화
% N=0 조건: [0 0 0 1]*[T,L,M,N]' = 0
Aeq_N0 = [0 0 0 1];
beq_N0 = 0;

[V_sliceN0, nr_sliceN0, nre_sliceN0] = lcon2vert(A_all_TLMN, b_all_TLMN, Aeq_N0, beq_N0, 1e-10, true);

if isempty(V_sliceN0)
    disp('N=0 슬라이스가 유효하지 않거나, 점이 없습니다.');
else
    % ACS_points는 [T,L,M,N] 순이므로, x = Roll (열2), y = Pitch (열3), z = Thrust (열1)
    sliceN0_RPT = V_sliceN0(:, [2,3,1]);
    if size(sliceN0_RPT,1) < 4
        disp('N=0 슬라이스에서 점이 너무 적어 3D 볼록껍질을 그리기 어렵습니다.');
    else
        [hullN0_RPT, vol_N0] = convhulln(sliceN0_RPT);
        convexIdx_N0 = unique(hullN0_RPT);

        figure('Name','Slice: N=0 => 3D in (Roll, Pitch, Thrust)','NumberTitle','off');
        trisurf(hullN0_RPT, sliceN0_RPT(:,1), sliceN0_RPT(:,2), sliceN0_RPT(:,3), ...
            'FaceColor','green','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
        hold on;
        plot3(sliceN0_RPT(:,1), sliceN0_RPT(:,2), sliceN0_RPT(:,3), 'bo', 'MarkerSize', 3);
        plot3(sliceN0_RPT(convexIdx_N0,1), sliceN0_RPT(convexIdx_N0,2), sliceN0_RPT(convexIdx_N0,3), ...
            'ks', 'MarkerSize', 8, 'MarkerFaceColor','r');
        xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Thrust T (N)');
        title('Sliced: N=0 (3D) in (Roll, Pitch, Thrust)');
        grid on; view(3); camlight; lighting gouraud;
        disp('convex_vol_N0 = ');
        disp(vol_N0);
    end
end

%% 10. 슬라이싱 예시: T=mg => 3차원 (L,M,N) 시각화
% T=mg 조건: [1 0 0 0]*[T,L,M,N]' = mg
Aeq_T = [1 0 0 0];
beq_T = m * g;

[V_sliceT, nr_sliceT, nre_sliceT] = lcon2vert(A_all_TLMN, b_all_TLMN, Aeq_T, beq_T, 1e-10, true);

if isempty(V_sliceT)
    disp('T=mg 슬라이스가 유효하지 않거나, 점이 없습니다.');
else
    % 결과 V_sliceT는 [T,L,M,N]에서 T=mg를 만족하는 점들이므로, 
    % 3차원 시각화를 위해 나머지 변수 L, M, N (열 2, 3, 4)를 사용함.
    sliceT_LMN = V_sliceT(:, [2,3,4]);
    if size(sliceT_LMN,1) < 4
        disp('T=mg 슬라이스에서 점이 너무 적어 3D 볼록껍질을 그리기 어렵습니다.');
    else
        [hullT_LMN, vol_T] = convhulln(sliceT_LMN);
        convexIdx_T = unique(hullT_LMN);
        
        figure('Name','Slice: T=mg => 3D in (Roll, Pitch, Yaw)','NumberTitle','off');
        trisurf(hullT_LMN, sliceT_LMN(:,1), sliceT_LMN(:,2), sliceT_LMN(:,3), ...
            'FaceColor','yellow','FaceAlpha',0.5,'EdgeColor','k','LineWidth',1.0);
        hold on;
        plot3(sliceT_LMN(:,1), sliceT_LMN(:,2), sliceT_LMN(:,3), 'bo', 'MarkerSize', 3);
        plot3(sliceT_LMN(convexIdx_T,1), sliceT_LMN(convexIdx_T,2), sliceT_LMN(convexIdx_T,3), ...
            'ks', 'MarkerSize', 8, 'MarkerFaceColor','m');
        xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)'); zlabel('Yaw Moment N (Nm)');
        title('Sliced: T=mg (3D) in (Roll, Pitch, Yaw)');
        grid on; view(3); camlight; lighting gouraud;
        disp('convex_vol_T (for T=mg slice) = ');
        disp(vol_T);
    end
end

%% 11. 슬라이싱 예시: T=mg, N=0 => 2차원 (L,M) 시각화
% T=mg, N=0 조건:
Aeq_TN = [1 0 0 0;   % T = mg
          0 0 0 1];   % N = 0
beq_TN = [m*g; 0];

[V_sliceTN, nr_sliceTN, nre_sliceTN] = lcon2vert(A_all_TLMN, b_all_TLMN, Aeq_TN, beq_TN, 1e-10, true);

if isempty(V_sliceTN)
    disp('T=mg, N=0 슬라이스가 유효하지 않거나, 점이 없습니다.');
else
    % ACS_points의 열 순서는 [T,L,M,N]에서 L=2, M=3
    sliceTN_LM = V_sliceTN(:, [2,3]);
    if size(sliceTN_LM,1) < 3
        disp('T=mg, N=0 슬라이스에서 점이 너무 적어 2D 볼록껍질을 그리기 어렵습니다.');
    else
        [hullTN_LM, vol_TN] = convhulln(sliceTN_LM);
        convexIdx_TN = unique(hullTN_LM);

        figure('Name','Slice: T=mg, N=0 => 2D in (Roll, Pitch)','NumberTitle','off');
        plot(sliceTN_LM(:,1), sliceTN_LM(:,2), 'bo','MarkerSize',4);
        hold on;
        plot(sliceTN_LM(hullTN_LM,1), sliceTN_LM(hullTN_LM,2), 'r-','LineWidth',2);
        plot(sliceTN_LM(convexIdx_TN,1), sliceTN_LM(convexIdx_TN,2), ...
             'ks','MarkerSize',8,'MarkerFaceColor','g');
        xlabel('Roll Moment L (Nm)'); ylabel('Pitch Moment M (Nm)');
        title('Sliced: T=mg, N=0 (2D) in (Roll, Pitch)');
        grid on;
        disp('convex_vol_TN = ');
        disp(vol_TN);
    end
end
