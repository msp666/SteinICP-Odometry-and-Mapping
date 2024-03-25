%% trajectory alignment
traj_trans_stein = estgeotform3d(pose_stein(:,2:4), matchedGt_Stein(:,2:4), "rigid");
figure; hold on
plot(matchedGt_Stein(:,2), matchedGt_Stein(:,3));

for i = 1:length(pose_stein)
position_stein_matched(i,:) = (traj_trans_stein.A * [pose_stein(i,2:4), 1]')';
end
position_stein_matched = position_stein_matched - position_stein_matched(1,:);
plot(position_stein_matched(:,1), position_stein_matched(:,2));


%%
traj_trans_kiss = estgeotform3d(pose_KISS(:,2:4), matchedGt_KISS(:,2:4), "rigid");
figure; hold on
plot(matchedGt_KISS(:,2), matchedGt_KISS(:,3));

for i = 1:length(pose_KISS)
position_kiss_matched(i,:) = (traj_trans_kiss.A * [pose_KISS(i,2:4), 1]')';
end
position_kiss_matched = position_kiss_matched - position_kiss_matched(1,:);
plot(position_kiss_matched(:,1), position_kiss_matched(:,2));

%% 
Error_Stein = mean(abs(position_stein_matched(:,1:3) - matchedGt_Stein(:,2:4)));
Error2D_Stein = mean(vecnorm(position_stein_matched(:,1:2) - matchedGt_Stein(:,2:3),2,2));

Error_Kiss = mean(abs(position_kiss_matched(:,1:3) - matchedGt_KISS(:,2:4)));
Error2D_Kiss = mean(vecnorm(position_kiss_matched(:,1:2) - matchedGt_KISS(:,2:3),2,2));