[d, Z, tran] = procrustes(BetweenPose_Stein.error.timeseries(:,3), ...
    cov_stein(2:length(BetweenPose_Stein.error.timeseries(:,3:8))+1, 1));
plot(BetweenPose_Stein.test(:,2), BetweenPose_Stein.error.timeseries(:,3));
hold on
plot(BetweenPose_Stein.test(:,2), Z);

%%
[d, Z, tran] = procrustes(error_smooth(:,1), ...
    cov_new_smooth(1:length(error_smooth),1));

%%  
dtw(BetweenPose_Stein.error.timeseries(:,3), ...
    cov_stein(2:length(BetweenPose_Stein.error.timeseries(:,3:8))+1, 1), ...
    1);

