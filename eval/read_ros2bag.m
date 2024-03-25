
clear all
%% KISS ICP Read ROS2 bag
ros2_bag = ros2bagreader("/mnt/Data/SteinICPResult/BR/Day/kiss-icp/kiss-icp_0.db3");
trajectory_msg_KISS = select(ros2_bag, Topic="/kiss/trajectory");
trajectory_decode_KISS = readMessages(trajectory_msg_KISS);
%%
for i = 1:length(trajectory_decode_KISS{end}.poses)
    pose_KISS(i,1) = double(trajectory_decode_KISS{end}.poses(i).header.stamp.sec) + 1e-9*double(trajectory_decode_KISS{end}.poses(i).header.stamp.nanosec);
    pose_KISS(i,2:4) = [trajectory_decode_KISS{end}.poses(i).pose.position.x,...
                     trajectory_decode_KISS{end}.poses(i).pose.position.y,...
                     trajectory_decode_KISS{end}.poses(i).pose.position.z];
    pose_KISS(i,5:7) = quat2eul([trajectory_decode_KISS{end}.poses(i).pose.orientation.w,...
                     trajectory_decode_KISS{end}.poses(i).pose.orientation.x,...
                     trajectory_decode_KISS{end}.poses(i).pose.orientation.y,...
                     trajectory_decode_KISS{end}.poses(i).pose.orientation.z], 'XYZ');
end

%% Stein ICP
ros2_bag_steinicp = ros2bagreader("/mnt/Data/SteinICPResult/BR/Day/SegVoxel01/SegVoxel01_0.db3");
steinicp_param_sel = select(ros2_bag_steinicp, 'Topic','/stein_icp/parameters');
steinicp_param = readMessages(steinicp_param_sel);
%% Pose with covariance 
PoseWithCov_msg = select(ros2_bag_steinicp, Topic="/stein_icp/pose_with_covariance");
PoseWithCov_decode = readMessages(PoseWithCov_msg);

for i = 1:(length(PoseWithCov_decode))
    pose_stein(i,1) = double(PoseWithCov_decode{i}.header.stamp.sec) + 1e-9*double(PoseWithCov_decode{i}.header.stamp.nanosec);
    pose_stein(i,2:4) = [PoseWithCov_decode{i,1}.pose.pose.position.x, ...
                   PoseWithCov_decode{i,1}.pose.pose.position.y, ...
                   PoseWithCov_decode{i,1}.pose.pose.position.z];
    pose_stein(i,5:7) = quat2eul([PoseWithCov_decode{i,1}.pose.pose.orientation.w ...
                           PoseWithCov_decode{i,1}.pose.pose.orientation.x,...
                           PoseWithCov_decode{i,1}.pose.pose.orientation.y,...
                           PoseWithCov_decode{i,1}.pose.pose.orientation.z], 'XYZ') ;
  
end

for i = 1:length(pose_stein)
    cov_stein(i,:) = PoseWithCov_decode{i,1}.pose.covariance(1:6);
    % cov_xyz_rot = eul2rotm(pose_stein(i,5:7), 'XYZ') * diag(cov_stein(i,1:3)) * eul2rotm(pose_stein(i,5:7), 'XYZ')';
    % cov_stein(i, 1:3) = diag(cov_xyz_rot)';
end



%% (Only for AC)
novatel_odom = select(ros2_bag_steinicp, Topic="/novatel/oem7/odom");
novatel_odom_decode = readMessages(novatel_odom);


