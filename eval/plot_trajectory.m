%% Import ground truth
dataset_id = "Boreas/Day";
ground_truth = LoadGroundTruth(dataset_id);
[stein_idx_test, stein_idx_gt] = FindIdxInterval(pose_stein, ground_truth);
pose_stein = Transform2InitCoordiate(pose_stein, stein_idx_test.start);
% pose_stein = time_repair(pose_stein);
ground_truth = Transform2InitCoordiate(ground_truth, stein_idx_gt.start);
gt_end = stein_idx_gt.end;
ground_truth(gt_end+1:end, :) = [];

%%
[kiss_idx_test, kiss_idx_gt] = FindIdxInterval(pose_KISS, ground_truth);
% idx_diff = stein_idx_gt.start - kiss_idx_gt.start;
pose_KISS = Transform2InitCoordiate(pose_KISS, kiss_idx_test.start+idx_diff);

gt_end = min(stein_idx_gt.end, kiss_idx_gt.end);
ground_truth(gt_end+1:end,:) = [];
gt_start = max(stein_idx_gt.start, kiss_idx_gt.start);
ground_truth(1:gt_start,:) = [];


%% Plot Ground Truth
figure
plot(ground_truth(:, 2), ground_truth(:, 3),'LineWidth', 1);
% plot(ground_truth(:, 2), ground_truth(:, 3),'LineWidth', 1);
hold on

%  PLot Stein ICP trajectory
plot(pose_stein(stein_idx_test.start:stein_idx_test.end,2), ...
     pose_stein(stein_idx_test.start:stein_idx_test.end,3), 'LineWidth', 2)

[TrajectoryError_Stein, MeanTrajecortyError, matchedGt_Stein] = TrajectoryError(pose_stein, ground_truth);
PositionError_stein = sqrt(TrajectoryError_Stein(:,2).*TrajectoryError_Stein(:,2)+ ...
                     TrajectoryError_Stein(:,3).*TrajectoryError_Stein(:,3));

MeanPositionError_Stein = mean(PositionError_stein);
PositionCov_stein = sqrt(cov_stein(:,1) + cov_stein(:,2));

% PLot Covariance Ellipse
for i = 1:3:length(pose_stein) 
    % cov_xyz_rot = eul2rotm(pose_stein(i,5:7), 'XYZ') * diag(cov_stein(i,1:3)) * eul2rotm(pose_stein(i,5:7), 'XYZ')';
    cov_mat = diag(cov_stein(i,1:2));
    % plterrel(position(i,2), position(i,1), cov_mat, 2)
    error_ellipse(cov_mat, pose_stein(i,2:4))
    hold on
end
% legend('cov(x)','cov(y)')

%% Plot KISS ICP
plot(pose_KISS(kiss_idx_test.start:kiss_idx_test.end,2), ...
    pose_KISS(kiss_idx_test.start:kiss_idx_test.end,3), 'LineWidth', 2, 'Color', [0.4660 0.6740 0.1880]);

[TrajectoryError_KISS, MeanTrajecortyError_KISS, matchedGt_KISS] = TrajectoryError(pose_KISS, ground_truth);
PositionError_KISS = sqrt(TrajectoryError_KISS(:,2).*TrajectoryError_KISS(:,2)+ ...
                     TrajectoryError_KISS(:,3).*TrajectoryError_KISS(:,3));
MeanPositionError_KISS = mean(PositionError_KISS); 

%%
function [error_timeseries, error_average, matched_gt] = TrajectoryError(pose, ground_truth)
         pose = time_repair(pose);
         for i = 1 : length(pose)
             time_diff = abs(pose(i,1)-ground_truth(:,1));
             % gt_idx_tmp = find( time_diff<0.025 );
             [~, gt_idx] = min(time_diff);
             if time_diff(gt_idx) > 0.02
                 continue;
             end

             if ~isempty(gt_idx)
                 % T_ref = se3(ground_truth(gt_idx,5:7), 'eul', 'XYZ', ground_truth(gt_idx, 2:4), 'trvec');
                 % T_pose = se3()
                 % T_e = inv(T_ref) * T_pose
                 error_timeseries(i, 1) = pose(i, 1);
                 error_timeseries(i, 2:7) = abs(pose(i, 2:7) - ground_truth(gt_idx, 2:7));
                 matched_gt(i, :) = ground_truth(gt_idx, :); 
             else
                 continue;
             end
         end
         error_timeseries(find(error_timeseries(:,1)==0), :) = [];
         error_average = mean(error_timeseries(:,2:7),1);

end

%%
function repaired_value = time_repair(pose)
         time_error_idx = find(pose(:,1)==0);
         for i = 1:length(time_error_idx)
             if isempty(find(time_error_idx==time_error_idx(i)+1)) & isempty(find(time_error_idx==time_error_idx(i)-1))
             pose(time_error_idx(i), 1) = (pose(time_error_idx(i)+1, 1) + pose(time_error_idx(i)+1, 1))/2;
             end
         end
         repaired_value = pose;
end

