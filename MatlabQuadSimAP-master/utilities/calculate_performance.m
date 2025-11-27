function calculate_performance
% =========================================================================
% CALCULATE_PERFORMANCE
%
%   시뮬레이션 로그 데이터를 기반으로 경로 추종 성능 지표를 계산하고
%   Command Window에 결과를 출력하는 함수입니다.
%
%   - 계산 지표: RMSE, MAE, Max Error (각 축 및 3D 전체)
%   - 호출 위치: quadrotor_sim.m (메인 시뮬레이션 루프 종료 후)
% =========================================================================

global Quad;

%% --- 데이터 준비 ---
% 시뮬레이션 데이터와 목표 경로 데이터의 길이를 맞춥니다.
len = min(length(Quad.X_log), length(Quad.X_des_log));
if len == 0
    fprintf('[Performance] No data logged. Cannot calculate performance.\n');
    return;
end

% 분석할 데이터만 추출
sim_traj = [Quad.X_log(1:len); Quad.Y_log(1:len); Quad.Z_log(1:len)];
ref_traj = [Quad.X_des_log(1:len); Quad.Y_des_log(1:len); Quad.Z_des_log(1:len)];

%% --- 성능 지표 계산 ---
error = sim_traj - ref_traj; % 각 축(x,y,z)에 대한 오차
total_distance_error = sqrt(sum(error.^2, 1)); % 각 시간 스텝에서의 3D 거리 오차

% 1. RMSE (Root Mean Square Error)
rmse_per_axis = sqrt(mean(error.^2, 2));
total_rmse = sqrt(mean(total_distance_error.^2));

% 2. MAE (Mean Absolute Error)
mae_per_axis = mean(abs(error), 2);
total_mae = mean(total_distance_error);

% 3. Max Error (최대 오차)
max_error_per_axis = max(abs(error), [], 2);
max_total_distance_error = max(total_distance_error);


%% --- 결과 출력 ---
fprintf('\n==================================================\n');
fprintf('    Trajectory Tracking Performance Metrics\n');
fprintf('==================================================\n\n');

fprintf('--- RMSE (Root Mean Square Error) ---\n');
fprintf('RMSE X: %.4f m\n', rmse_per_axis(1));
fprintf('RMSE Y: %.4f m\n', rmse_per_axis(2));
fprintf('RMSE Z: %.4f m\n', rmse_per_axis(3));
fprintf('Total 3D RMSE: %.4f m  (평균적인 3D 오차 거리)\n\n', total_rmse);

fprintf('--- MAE (Mean Absolute Error) ---\n');
fprintf('MAE X: %.4f m\n', mae_per_axis(1));
fprintf('MAE Y: %.4f m\n', mae_per_axis(2));
fprintf('MAE Z: %.4f m\n', mae_per_axis(3));
fprintf('Total 3D MAE: %.4f m  (이상치에 덜 민감한 평균 오차)\n\n', total_mae);

fprintf('--- Max Error ---\n');
fprintf('Max Error X: %.4f m\n', max_error_per_axis(1));
fprintf('Max Error Y: %.4f m\n', max_error_per_axis(2));
fprintf('Max Error Z: %.4f m\n', max_error_per_axis(3));
fprintf('Max 3D Distance Error: %.4f m  (가장 크게 벗어난 거리)\n', max_total_distance_error);
fprintf('\n==================================================\n');

end