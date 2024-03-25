quat1 = [gps_gt_msg{6575,1}.pose.pose.orientation.w, ...
         gps_gt_msg{6575,1}.pose.pose.orientation.x, ...
         gps_gt_msg{6575,1}.pose.pose.orientation.y, ...
         gps_gt_msg{6575,1}.pose.pose.orientation.z];

eul1 = quat2eul(quat1, "ZYX")

%%
figure
plot(boreas_gt(:,2)-boreas_gt(1,2), boreas_gt(:,3)-boreas_gt(1,3));
hold on
plot(gps_gt(:,2)-gps_gt(1,2), gps_gt(:,3)-gps_gt(1,3));
%%
figure 
plot(boreas_gt(:,1), boreas_gt(:,7));
hold on
plot(gps_gt(:,1), gps_gt(:,7))