%% calculate Odometry and Ground Truth Odometry
BetweenPose_Stein = BetweenPose(pose_stein, ground_truth);
BetweenPose_kiss = BetweenPose(pose_KISS, ground_truth);
OdomMeanError_Stein = mean(abs(BetweenPose_Stein.error.timeseries(:,3:8)), 1);
OdomMeanError_kiss = mean(abs(BetweenPose_kiss.error.timeseries(:, 3:8)), 1);

figure
plot(BetweenPose_Stein.error.timeseries(:,1)-BetweenPose_Stein.error.timeseries(1,1), BetweenPose_Stein.error.timeseries(:,9))
hold on
plot(BetweenPose_kiss.error.timeseries(:,1)- BetweenPose_kiss.error.timeseries(1,1), BetweenPose_kiss.error.timeseries(:,9))

%% plot Errors of Odometry
figure
plot(BetweenPose_Stein.gt(:,1)-BetweenPose_Stein.gt(1,1), BetweenPose_Stein.gt(:,6), 'Marker','o');
hold on
plot(BetweenPose_Stein.test(:,1)- BetweenPose_Stein.test(1,1), BetweenPose_Stein.test(:,6), 'Marker', 'o');

figure 
plot(BetweenPose_kiss.gt(:,1)-BetweenPose_kiss.gt(1,1), BetweenPose_kiss.gt(:,3), 'Marker','o')
hold on
plot(BetweenPose_kiss.test(:,1)-BetweenPose_kiss.gt(1,1), BetweenPose_kiss.test(:,3), 'Marker','o')

figure
plot(BetweenPose_Stein.gt(:,1)-BetweenPose_Stein.gt(1,1) , BetweenPose_Stein.error.timeseries(:,3));
hold on
plot(BetweenPose_kiss.gt(:,1)-BetweenPose_kiss.gt(1,1), BetweenPose_kiss.error.timeseries(:,3));

%% plot Error vs Covariance or STD
figure 
% cov_stein = cov_stein ./ source_pc_number*1e4;
% cov_stein(2:777, 1) = cov_stein(2:777,1) .*sign(BetweenPose_Stein.error.timeseries(:,3));
% cov_stein(1:800, :) = cov_stein(1:800, :)./max_abs;
plot(BetweenPose_Stein.test(:,2), BetweenPose_Stein.error.timeseries(:,3));
hold on
plot(pose_stein(1:end,1), sqrt(cov_stein(:, 1)));
% plot(pose_stein(1:end,1), sqrt(cov_weighted(:, 1))/3);
% plot(pose_stein(:,1), source_pc_number/5e5)
% plot(pose_stein(:,1), smoothdata(sqrt(cov_stein(:,1))/3, 'gaussian'), 'LineWidth', 1.5);
% plot(BetweenPose_Stein.test(:,2), smoothdata(OdomError_Stein(:,1), 'gaussian'));
%% Correlation between Covariance and Error (No Smoothing)
xcorrelation = CrossCorr(BetweenPose_Stein.error.timeseries(:,3:8), ...
    cov_stein(2:length(BetweenPose_Stein.error.timeseries(:,3:8))+1, :));

figure;
plot(xcorrelation(:,1), xcorrelation(:,2));

% plot( (cov_stein(2:length(OdomError_Stein)+1, 1)) ./OdomError_Stein(:,1) );

%% Correlation between added Cov
cov_new = cov_stein(1:end-1, :) + cov_stein(2:end, :);
for i = 1:length(pose_stein) - 1
    cov_mat= eul2rotm(pose_stein(i,5:7), 'XYZ')' * diag(cov_new(i,1:3)) * eul2rotm(pose_stein(i,5:7), 'XYZ') ;
    % cov_mat(cov_mat < 0 ) = 0;
    eigval = eig(cov_mat);
    cov_new(i, 1:3) = eigval(1:3)';
    % cov_xyz_rot = eul2rotm(pose_stein(i,5:7), 'XYZ') * diag(cov_stein(i,1:3)) * eul2rotm(pose_stein(i,5:7), 'XYZ')';
    % cov_stein(i, 1:3) = diag(cov_xyz_rot)';
end
error_smooth = smoothdata(BetweenPose_Stein.error.timeseries(:,3:10), 1,'gaussian', 20);
cov_new_smooth = smoothdata(cov_new, 1, "gaussian",20);
figure
plot(BetweenPose_Stein.test(:,2), error_smooth(:,1));
hold on
% plot(pose_stein(:,1), sqrt(cov_stein(:,1))/3);
% plot(pose_stein(2:end,1), sqrt(cov_new(:,1))/4, 'LineWidth', 1.5);
plot(pose_stein(2:end,1), sqrt(cov_new_smooth(:,1))/2, 'LineWidth', 1.5);

%%
xcorrelation_new = CrossCorr(BetweenPose_Stein.error.timeseries(:,3:8), (sqrt(cov_new(2:length(BetweenPose_Stein.error.timeseries)+1, :))));
figure
plot(xcorrelation_new(:,1), xcorrelation_new(:,2));

%%
figure
plot(BetweenPose_Stein.test(:,2), BetweenPose_Stein.error.timeseries(:,9));
hold on
plot(pose_stein(2:end,1), sqrt(sum(cov_new(:,1:3),2))/3);

figure
% plot(BetweenPose_Stein.test(:,2), error_smooth(:,7));
hold on
plot(BetweenPose_Stein.test(:,2), BetweenPose_Stein.error.timeseries(:,9));
plot(pose_stein(2:end,1), sqrt(sum(cov_new_smooth(:,1:3),2))/2);

[x_corr_trans, delay] = xcorr(error_smooth(:,1), ...
             sqrt(cov_new_smooth(:,1))/3 , 200);


%% 
figure
bound_x = [pose_stein(2:end,1)' pose_stein(end:-1:2, 1)'];
bound_y = [sqrt(cov_new_smooth(:, 1))'/4 -sqrt(cov_new_smooth(end:-1:1, 1))'/4];
fill(bound_x, bound_y, 'red', FaceColor=[1,0.5,0.5]);
hold on
% plot(BetweenPose_Stein.test(:,2), error_smooth(:,1));
plot(BetweenPose_Stein.test(:,2), BetweenPose_Stein.error.timeseries(:,3));