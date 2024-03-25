%% CA bag read


CA_bag = ros2bagreader("/media/haoming/Volume/CA/CA-20190828151211_blur_align-006/CA-20190828151211_blur_align-006_0.db3");
%%
gt_sel = select(CA_bag, 'Topic', '/novatel_data/inspvax');

%%
gt_msg = readMessages(gt_sel);

%%
lla0 = [gt_msg{1,1}.latitude, gt_msg{1,1}.longitude, gt_msg{1,1}.height];
for i = 1:length(gt_msg)
     CA_gt(i, 1) = double(gt_msg{i,1}.header.stamp.sec) ...
                   + 1e-9*double(gt_msg{i,1}.header.stamp.nanosec);
     position_lla = [gt_msg{i,1}.latitude, gt_msg{i,1}.longitude, gt_msg{i,1}.height];
     position_enu = lla2enu(position_lla, lla0, 'flat');
     CA_gt(i, 2:4) = position_enu;
     RPY_ZYX = [-gt_msg{i,1}.azimuth+90, gt_msg{i,1}.roll, gt_msg{i,1}.pitch]/180*pi;
     RPY_XYZ = quat2eul( eul2quat(RPY_ZYX, 'ZYX'), 'XYZ' );
     CA_gt(i, 5:7) = RPY_XYZ;

end


%%
lidar_sel = select(CA_bag, 'Topic', '/rslidar_points');
%%
lidar_msg = readMessages(lidar_sel);

%% interpolate pose for Lidar Timestamp

gt_timeseries = CA_gt(:,1) - CA_gt(1,1);
lidar_timestamps = Lidar_gt(:,1)-CA_gt(1,1);
Lidar_gt = [];
for i = 1: length(lidar_timestamps)   
    % lidar_timestamp = double(lidar_msg{i,1}.header.stamp.sec) ...
    %                   + 1e-9*double( lidar_msg{i,1}.header.stamp.nanosec)
    %                  -double(lidar_msg{1,1}.header.stamp.sec) ...
    %                   - 1e-9*double( lidar_msg{1,1}.header.stamp.nanosec);
    lidar_timestamp = lidar_timestamps(i);
    [~, nearest_time_idx] = min(abs(gt_timeseries - lidar_timestamp));
    nearest_time = gt_timeseries(nearest_time_idx);
    if nearest_time < lidar_timestamp && nearest_time_idx < length(gt_timeseries)
         start_time_idx = nearest_time_idx;
         finish_time_idx = nearest_time_idx + 1;
         start_time = nearest_time;
         finish_time = gt_timeseries(finish_time_idx);

    elseif nearest_time > lidar_timestamp && nearest_time_idx > 1
         start_time_idx = nearest_time_idx - 1;
         finish_time_idx = nearest_time_idx;
         start_time = gt_timeseries(start_time_idx);
         finish_time = nearest_time;
    elseif nearest_time == lidar_timestamp
        Lidar_gt(end+1,:) = CA_gt(nearest_time_idx,:);
        continue;
    else 
        continue;
    end
    
    start_pose = se3(CA_gt(start_time_idx, 5:7), 'eul', 'XYZ', CA_gt(start_time_idx, 2:4));
    finish_pose = se3(CA_gt(finish_time_idx, 5:7), 'eul', 'XYZ', CA_gt(finish_time_idx, 2:4));
    normalized_time = (lidar_timestamp - start_time) / (finish_time - start_time);

    lidar_pose = interp(start_pose, finish_pose, normalized_time);
    lidar_timestamp = lidar_timestamp + CA_gt(1,1);
    Lidar_gt(end+1, 1) = lidar_timestamp;
    Lidar_gt(end, 2:4) = trvec(lidar_pose);
    Lidar_gt(end, 5:7) = eul(lidar_pose, 'XYZ');
end

%% Interpolate Linear


%% 
IMU_sel = select(CA_bag, 'Topic', '/imu_rt');
%%
IMU_msg = readMessages(IMU_sel);

%%
Lidar_gt = [];
Lidar_gt(:,1) = lidar_timestamps + CA_gt(1,1);
Lidar_gt(:,2) = interp1(CA_gt(:,1)-CA_gt(1,1), CA_gt(:,2), lidar_timestamps);
Lidar_gt(:,3) = interp1(CA_gt(:,1)-CA_gt(1,1), CA_gt(:,3), lidar_timestamps);
Lidar_gt(:,4) = interp1(CA_gt(:,1)-CA_gt(1,1), CA_gt(:,4), lidar_timestamps);
Lidar_gt(:,5) = interp1(CA_gt(:,1)-CA_gt(1,1), CA_gt(:,5), lidar_timestamps);
Lidar_gt(:,6) = interp1(CA_gt(:,1)-CA_gt(1,1), CA_gt(:,6), lidar_timestamps);
Lidar_gt(:,7) = interp1(CA_gt(:,1)-CA_gt(1,1), CA_gt(:,7), lidar_timestamps);
