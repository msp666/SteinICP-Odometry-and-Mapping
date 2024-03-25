%% load source cloud & map cloud
source_cloud_sel = select(ros2_bag_steinicp, 'Topic','/stein_icp/source_cloud');
source_cloud_msg = readMessages(source_cloud_sel);

local_map_sel = select(ros2_bag_steinicp, 'Topic','/stein_icp/neighbourmap_cloud');
local_map_msg = readMessages(local_map_sel);

%% matlab icp transform
 x_s = rosReadField(source_cloud_msg{idx}, 'x');
 y_s = rosReadField(source_cloud_msg{idx}, 'y');
 z_s = rosReadField(source_cloud_msg{idx}, 'z');
 pc_source = pointCloud([x_s, y_s, z_s]);
 
 x_map = rosReadField(local_map_msg{idx}, 'x');
 y_map = rosReadField(local_map_msg{idx}, 'y');
 z_map = rosReadField(local_map_msg{idx}, 'z');
 pc_map = pointCloud([x_map, y_map, z_map]);
 pcshowpair(pc_source, pc_map)

 [tf, tf_cloud]=pcregistericp(pc_source, pc_map, "MaxIterations", 500, 'Tolerance',[0.005,0.1])
 figure
 pcshowpair(tf_cloud, pc_map)

 %% stein icp result
 translation = [mean(particle_msg{idx,1}.x),...
                         mean(particle_msg{idx,1}.y),... 
                         mean(particle_msg{idx,1}.z)];
 ypr = [mean(particle_msg{idx,1}.yaw),...
        mean(particle_msg{idx,1}.pitch),... 
        mean(particle_msg{idx,1}.roll)];
 Rotm = eul2rotm(ypr, 'ZYX');
 stein_tf = rigidtform3d(Rotm, translation);
 stein_tf_cloud = pctransform(pc_source, stein_tf);
 figure 
 pcshowpair(stein_tf_cloud, pc_map)

 

%% 
 icp_tf = table('Size',[1,6], ...
    'VariableTypes', {'double', 'double','double','double','double','double'},...
    'VariableNames',{'x','y','z','roll','pitch','yaw'});
for i = 1:length(pose_stein)
     x_s = rosReadField(source_cloud_msg{i}, 'x');
     y_s = rosReadField(source_cloud_msg{i}, 'y');
     z_s = rosReadField(source_cloud_msg{i}, 'z');
     pc_source = pointCloud([x_s, y_s, z_s]);
     
     x_map = rosReadField(local_map_msg{i}, 'x');
     y_map = rosReadField(local_map_msg{i}, 'y');
     z_map = rosReadField(local_map_msg{i}, 'z');
     pc_map = pointCloud([x_map, y_map, z_map]);
     tf = pcregistericp(pc_source, pc_map, "MaxIterations", 500, 'Tolerance',[0.005,0.001]);
     ypr = rotm2eul(tf.R,"ZYX");
     icp_tf(i,:) = {tf.Translation(1), tf.Translation(2), tf.Translation(3), ypr(3),ypr(2),ypr(1)};


end

%%
for i = 1:length(pose_stein) 
     x_s = rosReadField(source_cloud_msg{i}, 'x');
     y_s = rosReadField(source_cloud_msg{i}, 'y');
     z_s = rosReadField(source_cloud_msg{i}, 'z');
     points_s = [x_s, y_s, z_s];
     x_map = rosReadField(local_map_msg{i}, 'x');
     y_map = rosReadField(local_map_msg{i}, 'y');
     z_map = rosReadField(local_map_msg{i}, 'z');
     points_map = [x_map, y_map, z_map];

     transform_cor = rigidtform3d(eul2rotm([particle_weightmean.roll(i), ...
                                        particle_weightmean.pitch(i), ...
                                        particle_weightmean.yaw(i)], 'XYZ'), ...
                              [particle_weightmean.x(i), ...
                              particle_weightmean.y(i), ...
                              particle_weightmean.z(i)]);
     transform_pose = rigidtform3d(eul2rotm(pose_stein(i, 5:7), 'XYZ'), pose_stein(i, 2:4));
     transform_init = rigidtform3d(transform_pose.A * inv(transform_cor.A));
     ps_local = transformPointsInverse(transform_init, points_s);
     pt_local = transformPointsInverse(transform_init, points_map);
     
     max_abs(i,1) = max([ps_local; pt_local], [], 'all');
     % KdTree = KDTreeSearcher(pt_local);
     % correspondence = knnsearch(KdTree, ps_local);
     % Residual(i,:) = mean(abs(ps_local - pt_local(correspondence, :)),1);

end

%%
for i = 1:length(pose_stein) 
    x_s = rosReadField(source_cloud_msg{i}, 'x');
    source_pc_number(i,1) = length(x_s);
end
