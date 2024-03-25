figure; hold on
plot(BR_Day_KFVoxel1.Covariace(:,2));
plot(BR_Snowing_KFVoxel1.Covariace(:,2));

CovMean_Day = mean(BR_Day_KFVoxel1.Covariace, 1);
CovMean_Snowing = mean(BR_Snowing_KFVoxel1.Covariace, 1);

%%
figure; hold on
plot(BR_Dat_Voxel10.Pose(:,2), BR_Dat_Voxel10.Pose(:,3))
plot(BR_Snowing_Voxel1.Pose(:,2), BR_Snowing_Voxel1.Pose(:,3))
for i = 1:3:length(BR_Dat_Voxel10.Pose) 
    % cov_xyz_rot = eul2rotm(pose_stein(i,5:7), 'XYZ') * diag(cov_stein(i,1:3)) * eul2rotm(pose_stein(i,5:7), 'XYZ')';
    cov_mat = diag(BR_Dat_Voxel10.Covariace(i,1:2));
    % plterrel(position(i,2), position(i,1), cov_mat, 2)
    error_ellipse(cov_mat, BR_Dat_Voxel10.Pose(i,2:4))
    hold on
end
for i = 1:3:length(BR_Snowing_Voxel1.Pose) 
    % cov_xyz_rot = eul2rotm(pose_stein(i,5:7), 'XYZ') * diag(cov_stein(i,1:3)) * eul2rotm(pose_stein(i,5:7), 'XYZ')';
    cov_mat = diag(BR_Snowing_Voxel1.Covariace(i,1:2));
    % plterrel(position(i,2), position(i,1), cov_mat, 2)
    error_ellipse(cov_mat, BR_Snowing_Voxel1.Pose(i,2:4))
    hold on
end