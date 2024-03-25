T_test = eul2tform([pose_stein(stein_idx_test.start, 5), ...
                    pose_stein(stein_idx_test.start, 6), ...
                    pose_stein(stein_idx_test.start, 7)], 'XYZ');
% T_test = eul2tform(pose_stein(stein_start_idx_test, 5:7),'XYZ');
T_test(1:3, 4) = pose_stein(stein_idx_test.start, 2:4)';

T_gt = eul2tform([ground_truth(stein_idx_test.start, 5), ...
                  ground_truth(stein_idx_test.start, 6), ...
                  ground_truth(stein_idx_test.start, 7)], 'XYZ');
% T_gt = eul2tform(ground_truth(stein_start_idx_gt, 5:7));

T_gt(1:3,4) = ground_truth(stein_idx_test.start, 2:4)';
T_test_gt = T_test * inv(T_gt);

%%
for i = 1 : length(ground_truth)
    T_gt_tmp = eul2tform([ground_truth(i, 5), ...
                          ground_truth(i, 6), ...
                          ground_truth(i, 7)], 'XYZ');
    % T_gt_tmp = eul2tform(ground_truth(i, 5:7));
    T_gt_tmp(1:3,4) = ground_truth(i, 2:4)';
    T_test_tmp = T_test_gt * T_gt_tmp;
    gt_test_frame(i,1) = ground_truth(i,1);
    gt_test_frame(i,2:4) = T_test_tmp(1:3,4)';
    gt_test_frame(i,5:7) = rotm2eul(T_test_tmp(1:3,1:3));
end
%%
plot(gt_test_frame(:,2), gt_test_frame(:,3))
hold on

%%
plot(gt_test_frame(:,1), gt_test_frame(:,7))
