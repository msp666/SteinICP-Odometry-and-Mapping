%%
dataset_id = "CA";
ground_truth = LoadGroundTruth(dataset_id);
[stein_idx_test, stein_idx_gt] = FindIdxInterval(pose_stein, ground_truth);
[kiss_idx_test, kiss_idx_gt] = FindIdxInterval(pose_KISS, ground_truth);
% gt_start = max(kiss_idx_gt.start, stein_idx_gt.start);
% gt_end = min(kiss_idx_gt.end, stein_idx_gt.end);
% ground_truth = ground_truth(gt_start: gt_end, :);
[pose_stein_GtFrame, matchedGt_Stein, tf_Stein] = Transform2GT(pose_stein, ground_truth);
[pose_kiss_GtFrame, matchedGt_KISS, tf_KISS] = Transform2GT(pose_KISS, ground_truth);

pose_stein = pose_stein_GtFrame;
pose_KISS = pose_kiss_GtFrame;

%%
figure; hold on
plot(matchedGt_KISS(:,2), matchedGt_KISS(:,3));
plot(pose_stein_GtFrame(:,2), pose_stein_GtFrame(:,3));
plot(pose_kiss_GtFrame(:,2), pose_kiss_GtFrame(:,3));

TrajectoryError_KISS(:,1) = matchedGt_KISS(:,1);
TrajectoryError_KISS(:,2:7) = abs( matchedGt_KISS(:,2:7) - pose_kiss_GtFrame(:,2:7));
TrajectoryError_Stein(:,1) = matchedGt_Stein(:,1);
TrajectoryError_Stein(:,2:7) = abs(matchedGt_Stein(:,2:7) - pose_stein_GtFrame(:,2:7));
MeanTrajecortyError_KISS = mean(TrajectoryError_KISS(:, 2:7), 1);
MeanTrajectoryError_Stein = mean(TrajectoryError_Stein(:, 2:7), 1);

PositionError_KISS = sqrt(TrajectoryError_KISS(:,2).*TrajectoryError_KISS(:,2)+ ...
                     TrajectoryError_KISS(:,3).*TrajectoryError_KISS(:,3));
PositionError_stein = sqrt(TrajectoryError_Stein(:,2).*TrajectoryError_Stein(:,2)+ ...
                     TrajectoryError_Stein(:,3).*TrajectoryError_Stein(:,3));
MeanPositionError_Stein = mean(PositionError_stein);
MeanPositionError_KISS = mean(PositionError_KISS); 