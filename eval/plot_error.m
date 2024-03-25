close all

%%
figure 
plot(TrajectoryError_Stein(stein_start_idx_test:end, 1), TrajectoryError_Stein(stein_start_idx_test:end, 2));
hold on
plot(pose_stein(stein_start_idx_test:end,1), sqrt(cov_stein(stein_start_idx_test:end,1)))

%%
figure 
plot(TrajectoryError_Stein(stein_start_idx_test:end, 1), PositionError_stein(stein_start_idx_test:end));
hold on
plot(pose_stein(stein_start_idx_test:end,1), PositionCov_stein(stein_start_idx_test:end))

%%
plot(TrajectoryError_Stein(stein_start_idx_test:end, 1), PositionError_stein(stein_start_idx_test:end));
hold on
plot(TrajectoryError_KISS(kiss_start_idx_test:end, 1), PositionError_KISS(kiss_start_idx_test:end));

%%  
figure
plot(ground_truth(:, 1) ,...
     ground_truth(:, 5));
hold on
plot(pose_stein(:,1), pose_stein(:,5));
plot(pose_KISS(:,1), pose_KISS(:,5));

figure
plot(ground_truth(:, 1) ,...
     ground_truth(:, 6));
hold on
plot(pose_stein(:,1), pose_stein(:,6));