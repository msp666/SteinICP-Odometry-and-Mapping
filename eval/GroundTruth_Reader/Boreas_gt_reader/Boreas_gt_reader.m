%% Boreas gt

%% read ros2bag
boreas_bag = ros2bagreader('/mnt/Data/ros2bag_boreas-2020-12-18-13-44/');

%% 
lidar_gt_sel = select(boreas_bag, Topic= "/boreas/lidar_pose");

%%
lidar_gt_msg = readMessages(lidar_gt_sel);
%%

R_enu_ned = [0,1,0; 1,0,0; 0,0,-1];

for i = 1:length(lidar_gt_msg)
    boreas_gt(i, 1) = double(lidar_gt_msg{i,1}.header.stamp.sec) ...
                   + 1e-9*double(lidar_gt_msg{i,1}.header.stamp.nanosec);

    RPY_ZYX = [-lidar_gt_msg{i,1}.heading, -lidar_gt_msg{i,1}.pitch, -lidar_gt_msg{i,1}.roll];
    RPY_XYZ = rotm2eul( eul2rotm(RPY_ZYX, 'ZYX'), 'XYZ' );
    RPY_XYZ = [-lidar_gt_msg{i,1}.roll, -lidar_gt_msg{i,1}.pitch, -lidar_gt_msg{i,1}.heading];
    boreas_gt(i, 2:4) = [lidar_gt_msg{i,1}.pos_easting, ...
                         lidar_gt_msg{i,1}.pos_northing, ...
                         lidar_gt_msg{i,1}.pos_altitude];
    boreas_gt(i, 5:7) = RPY_XYZ;

    % boreas_gt(i, 2:7) = [lidar_gt_msg{i,1}.pos_easting, ...
    %                      lidar_gt_msg{i,1}.pos_northing, ...
    %                      lidar_gt_msg{i,1}.pos_altitude, ...
    %                      lidar_gt_msg{i,1}.roll, ...
    %                      -lidar_gt_msg{i,1}.pitch,...
    %                      -lidar_gt_msg{i,1}.heading];
    % boreas_gt(i,5:7) = rotm2eul( R_enu_ned*eul2rotm(boreas_gt(i,5:7), 'XYZ'), 'XYZ' )';
end



%%
gps_gt_sel = select(boreas_bag, 'Topic','/boreas/gps_gt_enu');
gps_gt_msg = readMessages(gps_gt_sel);
%%
for i = 1:length(gps_gt_msg)
    gps_gt(i,1) = double(gps_gt_msg{i,1}.header.stamp.sec) + ...
                  1e-9 * double(gps_gt_msg{i,1}.header.stamp.nanosec);
    gps_gt(i,2:4) = [gps_gt_msg{i,1}.pose.pose.position.x, ...
                     gps_gt_msg{i,1}.pose.pose.position.y, ...
                     gps_gt_msg{i,1}.pose.pose.position.z];
    quat = [gps_gt_msg{i,1}.pose.pose.orientation.w, ...
            gps_gt_msg{i,1}.pose.pose.orientation.x, ...
            gps_gt_msg{i,1}.pose.pose.orientation.y, ...
            gps_gt_msg{i,1}.pose.pose.orientation.z];
    RPY_XYZ = quat2eul(quat, 'XYZ');
    gps_gt(i, 5:7) = RPY_XYZ;
end
