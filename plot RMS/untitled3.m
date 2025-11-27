%% RMS 및 Controllability Margin 계산 스크립트
% 이 스크립트는 논문의 식 (1) ~ (32) 개념에 따라,
% 주어진 속도/자세 범위에서 트림 해(및 그 주변 오버슈트)를 구한 후,
% disturbance와 maneuver 점들의 Minkowski 합으로 RMS 세트를 구성하고,
% 4차원 방향( (n_z, p_dot, q_dot, r_dot) 공간)에서의 교차값을 이용해
% AMS와 RMS의 margin을 계산하는 예시입니다.
%
% 참고: AMS는 여기서는 단순히 RMS 점들을 1.1배 스케일하여 예시로 사용합니다.

clear; clc; close all;

%% 1. 기본 파라미터 설정
mass = 5.0;         % 질량 (kg)
g = 9.81;           % 중력 가속도 (m/s^2)
Ixx = 0.2; Iyy = 0.2; Izz = 0.3;  % 관성 모멘트 (예시)
% Inertia = diag([Ixx, Iyy, Izz]);  % (여기서는 직접 L = Ixx*p_dot 등으로 사용)

rho = 1.225;        % 공기 밀도 (kg/m^3)
Sref = 0.3;         % 기준 면적 (m^2)
cbar = 0.3;         % 평균 현 (m)
bspan = 1.0;        % 스팬 (m)

%% 2. 공력 계수 함수 (간단 모델)
% 힘 계수: [Cx, Cy, Cz]
aeroForceCoeffs = @(alpha, beta) [0.2*alpha, 0.1*beta, -0.3*alpha];
% 모멘트 계수: [Cl, Cm, Cn]
aeroMomentCoeffs = @(alpha, beta) [0.01*beta, 0.02*alpha, 0.01*beta];

%% 3. Airspeed Envelope 및 자세 범위, 오버슈트 설정
% Airspeed Envelope (body frame): (u, v, w) 범위
uC_min = -8;  uC_max = 8;  n_u = 3;
vC_min = -8;  vC_max = 8;  n_v = 3;
wC_min = -3;  wC_max = 3;  n_w = 3;
u_vals = linspace(uC_min, uC_max, n_u);
v_vals = linspace(vC_min, vC_max, n_v);
w_vals = linspace(wC_min, wC_max, n_w);

% 자세 범위 (트림 상태를 구할 때 사용, deg 단위)
phi_list   = [-10, 0, 10];    % Roll (deg)
theta_list = [-10, 0, 10];    % Pitch (deg)
% 오버슈트 범위 (논문 식 (11)~(14), deg 단위)
overshoot_phi = 5;          
overshoot_theta = 5;

%% 4. 트림 주변 교란(오버슈트) 점들 생성
% disturbance_pts: 각 점은 [T, L, M, N] (추력 및 모멘트)로, 이 값은 해당
% (u,v,w)와 (phi,theta) (오버슈트 포함)에서 기체가 등속 상태 유지하기 위한 해.
disturbance_pts = [];
for u = u_vals
    for phi_deg = phi_list
        for theta_deg = theta_list
            % deg -> rad
            phi = deg2rad(phi_deg);
            theta = deg2rad(theta_deg);
            % v, w는 0으로 가정하여 트림 해 계산
            trim_sol = trimSolve(u, 0, 0, phi, theta, mass, g, rho, Sref, cbar, bspan);
            if isempty(trim_sol)
                continue;
            end
            % 트림 해는 [T_trim, L_trim, M_trim, N_trim] (여기선 실제 값은 사용하지 않고,
            % 오버슈트 시의 공력 변화 평가에 집중)
            
            % 오버슈트 각도 범위 (deg)
            phi_ov_range = linspace(phi_deg - overshoot_phi, phi_deg + overshoot_phi, 3);
            theta_ov_range = linspace(theta_deg - overshoot_theta, theta_deg + overshoot_theta, 3);
            
            for phi_ov_deg = phi_ov_range
                for theta_ov_deg = theta_ov_range
                    phi_ov = deg2rad(phi_ov_deg);
                    theta_ov = deg2rad(theta_ov_deg);
                    
                    % (A) 에어로힘 계산 (식 (3)-(4) 개념)
                    [Faero, Maero] = computeAeroForcesMoments(u, 0, 0, phi_ov, theta_ov, rho, Sref, cbar, bspan, aeroForceCoeffs, aeroMomentCoeffs);
                    % (B) 중력 성분 계산 (자세에 따른 body 좌표계)
                    Fg = gravityBody(phi_ov, theta_ov, mass, g);
                    
                    sumF_aero = Faero + Fg;
                    % 프로펠러는 body -Z 방향으로 작용한다고 가정하므로,
                    % 등속 상태 유지에 필요한 추력 T_req = - (sumF의 z 성분)
                    T_req = -sumF_aero(3);
                    % 모멘트는 공력 모멘트를 상쇄하는 제어 모멘트로 계산
                    L_req = -Maero(1);
                    M_req = -Maero(2);
                    N_req = -Maero(3);
                    
                    disturbance_pts = [disturbance_pts; T_req, L_req, M_req, N_req];
                end
            end
        end
    end
end

%% 5. 기동 요구 점들 생성 (Maneuver points, Cuboid 형태, 식 (25)-(28))
% 기동 한계 (예시 값)
pdot_max = 100;  % 롤 각가속도 최대 (deg/s^2)
qdot_max = 100;  % 피치 각가속도 최대 (deg/s^2)
rdot_max = 30;   % 요 각가속도 최대 (deg/s^2)
nz_max   = 2;    % 최대 수직 가속도 load factor (배수)
num_steps = 5;  % 샘플링 개수

maneuver_pts = [];
p_vals = linspace(-pdot_max, pdot_max, num_steps);
q_vals = linspace(-qdot_max, qdot_max, num_steps);
r_vals = linspace(-rdot_max, rdot_max, num_steps);
nz_vals = linspace(1, nz_max, num_steps);
for p_val = p_vals
    for q_val = q_vals
        for r_val = r_vals
            for nz_val = nz_vals
                % 단위 변환 (deg/s^2 -> rad/s^2)
                p_rad = deg2rad(p_val);
                q_rad = deg2rad(q_val);
                r_rad = deg2rad(r_val);
                % 단순 모델: L = Ixx*p_dot, M = Iyy*q_dot, N = Izz*r_dot
                L_maneuver = Ixx * p_rad;
                M_maneuver = Iyy * q_rad;
                N_maneuver = Izz * r_rad;
                % T는 load factor: T = nz * m * g
                T_maneuver = nz_val * mass * g;
                maneuver_pts = [maneuver_pts; T_maneuver, L_maneuver, M_maneuver, N_maneuver];
            end
        end
    end
end

%% 6. Minkowski 합: disturbance_pts + maneuver_pts → RMS 점들
all_pts = [];
for i = 1:size(disturbance_pts,1)
    for j = 1:size(maneuver_pts,1)
        all_pts = [all_pts; disturbance_pts(i,:) + maneuver_pts(j,:)];
    end
end

%% 7. Convex Hull 계산 → RMS 세트
hull_indices = convhulln(all_pts);
% RMS 세트의 점들은 all_pts의 hull_indices에 해당하는 점들입니다.
unique_vertices = unique(hull_indices(:));
disp('===== RMS Convex Hull =====');
disp(['전체 입력 점 개수: ', num2str(size(all_pts,1))]);
disp(['볼록껍질 꼭짓점 개수: ', num2str(length(unique_vertices))]);
disp('일부 꼭짓점 (T, L, M, N):');
disp(all_pts(unique_vertices(1:min(10,length(unique_vertices))), :));

%% 8. 4차원 방향 벡터 생성 (n_z, p_dot, q_dot, r_dot 공간)
nBeta1 = 12;   % 첫번째 각도 개수
nBeta2 = 12;   % 두번째 각도 개수
nBeta3 = 24;   % 세번째 각도 개수
nz_scale   = 2.5;  
pdot_scale = 200;
qdot_scale = 200;
rdot_scale = 120;
beta1_vals = linspace(0, pi, nBeta1);
beta2_vals = linspace(0, pi, nBeta2);
beta3_vals = linspace(0, 2*pi, nBeta3);
D = [];
for b1 = beta1_vals
    for b2 = beta2_vals
        for b3 = beta3_vals
            d_nz    = nz_scale   * cos(b1);
            d_pdot  = pdot_scale * sin(b1)*cos(b2);
            d_qdot  = qdot_scale * sin(b1)*sin(b2)*cos(b3);
            d_rdot  = rdot_scale * sin(b1)*sin(b2)*sin(b3);
            len = sqrt(d_nz^2 + d_pdot^2 + d_qdot^2 + d_rdot^2);
            if len < 1e-12
                continue;
            end
            d_vec = [d_nz; d_pdot; d_qdot; d_rdot] / len;
            D = [D, d_vec];
        end
    end
end

%% 9. 각 쿼리 방향에 대해 RMS 크기 계산 (Disturbance + Maneuver)
nDir = size(D,2);
aRMS_D = zeros(1, nDir);
for iDir = 1:nDir
    dirVec = D(:, iDir);
    tVal = findCrossSection4D(all_pts, hull_indices, dirVec);
    aRMS_D(iDir) = tVal;
end

% Maneuver 부분: Cuboid의 각 한계값으로부터의 cross section (식 (25)-(28))
aRMS_M = zeros(1, nDir);
for iDir = 1:nDir
    dirVec = D(:, iDir);
    denom = [nz_max; pdot_max; qdot_max; rdot_max];
    ratio = abs(dirVec ./ denom);
    val = max(ratio);
    if val < 1e-9
        aRMS_M(iDir) = 0;
    else
        aRMS_M(iDir) = 1 / val;
    end
end
aRMS = aRMS_D + aRMS_M;  % 총 RMS 크기 in each direction

%% 10. AMS (Attainable Moment Set) 예시 계산
% 여기서는 단순히 RMS 점들의 1.1배로 설정 (실제 시스템에 맞게 계산 필요)
AMS_points = 1.1 * all_pts;
hull_indices_AMS = convhulln(AMS_points);
aAMS = zeros(1, nDir);
for iDir = 1:nDir
    dirVec = D(:, iDir);
    tValAMS = findCrossSection4D(AMS_points, hull_indices_AMS, dirVec);
    aAMS(iDir) = tValAMS;
end

%% 11. Controllability 평가 및 Margin 계산 (식 (31)~(32))
diffVals = aAMS - aRMS;
if min(diffVals) >= 0
    disp('모든 방향에서 AMS가 RMS를 만족합니다. (Controllability OK)');
else
    disp('일부 방향에서 RMS가 AMS를 초과합니다. (Re-design 필요)');
end

mVals = zeros(1, nDir);
for iDir = 1:nDir
    if abs(aAMS(iDir)) < 1e-9
        mVals(iDir) = -999;  % 정의 불가
    else
        mVals(iDir) = (aAMS(iDir) - aRMS(iDir)) / aAMS(iDir);
    end
end
meanMargin = mean(mVals);
minMargin  = min(mVals);
failRatio  = 100 * sum(mVals < 0) / length(mVals);
disp('=== RMS vs. AMS 비교 결과 ===');
fprintf('평균 margin     = %.3f\n', meanMargin);
fprintf('최소 margin     = %.3f\n', minMargin);
fprintf('실패 비율      = %.2f %%\n', failRatio);

%% 12. 결과 시각화 (예: n_z vs. p_dot, q_dot, r_dot)
figure;
plot(all_pts(:,1), all_pts(:,2), 'ro');
xlabel('n_z'); ylabel('p_{dot}');
title('RMS: n_z vs. p_{dot}');
grid on;
figure;
plot(all_pts(:,1), all_pts(:,3), 'bx');
xlabel('n_z'); ylabel('q_{dot}');
title('RMS: n_z vs. q_{dot}');
grid on;
figure;
plot(all_pts(:,1), all_pts(:,4), 'k^');
xlabel('n_z'); ylabel('r_{dot}');
title('RMS: n_z vs. r_{dot}');
grid on;
disp('=== RMS 및 Margin 계산 완료 ===');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 아래는 스크립트에서 사용하는 로컬 함수들
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function sol = trimSolve(u, v, w, phi, theta, mass, g, rho, Sref, cbar, bspan)
    % 주어진 u, v, w, phi, theta에서 p=q=r=0, 가속도=0 조건을 만족하는
    % 추력 T와 모멘트 (L, M, N)를 fsolve로 구하는 함수.
    init_guess = [mass*g, 0, 0, 0];  % 초기 추정값
    options = optimoptions('fsolve','Display','off');
    eqFunc = @(x) trimEquations(x, u, v, w, phi, theta, mass, g, rho, Sref, cbar, bspan);
    [sol, ~, exitflag] = fsolve(eqFunc, init_guess, options);
    if exitflag <= 0
        sol = [];
    end
end

function F = trimEquations(x, u, v, w, phi, theta, mass, g, rho, Sref, cbar, bspan)
    % x = [T, L, M, N]
    T = x(1); L = x(2); M = x(3); N = x(4);
    [Faero, Maero] = computeAeroForcesMoments(u, v, w, phi, theta, rho, Sref, cbar, bspan, ...
                                @(alpha, beta) [0.2*alpha, 0.1*beta, -0.3*alpha], ...
                                @(alpha, beta) [0.01*beta, 0.02*alpha, 0.01*beta]);
    Fg = gravityBody(phi, theta, mass, g);
    Fprop = [0; 0; -T];  % 프로펠러 추력 (body -Z 방향)
    Mprop = [L; M; N];
    F_total = Faero + Fg + Fprop;
    M_total = Maero + Mprop;
    F = [F_total(1); F_total(2); F_total(3); M_total(1); M_total(2); M_total(3)];
end

function [Force, Moment] = computeAeroForcesMoments(u, v, w, phi, theta, rho, Sref, cbar, bspan, aeroForceCoeffs, aeroMomentCoeffs)
    % 주어진 속도와 자세에서 공력과 모멘트를 계산 (매우 단순화한 모델)
    V = sqrt(u^2 + v^2 + w^2);
    if V < 1e-6, V = 1e-6; end
    alpha = atan2(w, u);
    beta  = atan2(v, u);
    qbar = 0.5 * rho * V^2;
    coeffsF = aeroForceCoeffs(alpha, beta);
    Force = qbar * Sref * coeffsF(:);
    coeffsM = aeroMomentCoeffs(alpha, beta);
    L = qbar * Sref * bspan * coeffsM(1);
    M = qbar * Sref * cbar  * coeffsM(2);
    N = qbar * Sref * bspan * coeffsM(3);
    Moment = [L; M; N];
end

function Fg = gravityBody(phi, theta, mass, g)
    % 주어진 phi, theta에서 body 좌표계 내 중력 성분 계산 (간단 모델)
    Xg = mass * g * sin(theta);
    Yg = -mass * g * sin(phi) * cos(theta);
    Zg = -mass * g * cos(phi) * cos(theta);
    Fg = [Xg; Yg; Zg];
end

function tVal = findCrossSection4D(pointSet, hullIndices, dirVec)
    % 주어진 4D 점 집합(pointSet)와 볼록껍질(hullIndices)에 대해,
    % dirVec 방향으로의 내적 최댓값(교차점)을 반환.
    dvals = pointSet * dirVec;
    tVal = max(dvals);
end
