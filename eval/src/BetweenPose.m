%% Odometry
function Odometry = BetweenPose(pose, gt)
      for i = 2:length(pose)
          start_timestamp = pose(i-1,1);
          finish_timestamp = pose(i,1);
          % start_gt_idx = find( abs(start_timestamp-gt(:,1)) < 0.05);
          % finish_gt_idx = find( abs(finish_timestamp-gt(:,1)) < 0.05);
          time_diff_start = abs(start_timestamp-gt(:,1));
          time_diff_end = abs(finish_timestamp-gt(:,1));
          [~, start_gt_idx] = min(time_diff_start);
          [~, finish_gt_idx] = min(time_diff_end);
          if time_diff_start(start_gt_idx) > 0.02 ...
             || time_diff_end(finish_gt_idx) >0.02
              continue;
          end

          if isempty(start_gt_idx) || isempty(finish_gt_idx)
              continue;
          end
%% Ground Truth Between Pose
          T_gt_i_1 = eul2tform(gt(start_gt_idx, 5:7), 'XYZ');
          T_gt_i_1(1:3, 4) = gt(start_gt_idx, 2:4)';
          T_gt_i = eul2tform(gt(finish_gt_idx, 5:7), 'XYZ');
          T_gt_i(1:3, 4) = gt(finish_gt_idx, 2:4)';
          T_gt_between = inv(T_gt_i_1) * T_gt_i;
          Odom_gt(i-1, 1) = gt(start_gt_idx, 1);
          Odom_gt(i-1, 2) = gt(finish_gt_idx, 1);
          Odom_gt(i-1, 3:5) = T_gt_between(1:3, 4)';
          Odom_gt(i-1, 6:8) = rotm2eul(T_gt_between(1:3, 1:3), 'XYZ');


%% Test Between Pose
          T_i_1 = eul2tform([pose(i-1, 5:7)], 'XYZ');
          T_i_1(1:3, 4) = pose(i-1, 2:4)';
          T_i   = eul2tform(pose(i, 5:7), 'XYZ');
          T_i(1:3, 4) = pose(i, 2:4)';
          T_between = inv(T_i_1) * T_i;
          
          Odometry_test(i-1, 1) = start_timestamp;
          Odometry_test(i-1, 2) = finish_timestamp;
          Odometry_test(i-1, 3:5) = T_between(1:3,4)';
          Odometry_test(i-1, 6:8) = rotm2eul(T_between(1:3,1:3), 'XYZ');


%% Error
          T_error = inv(T_gt_between) * T_between;
          Odometry_error(i-1, 1) = start_timestamp;
          Odometry_error(i-1, 2) = finish_timestamp;
          Odometry_error(i-1, 3:5) = abs(T_error(1:3, 4)');
          Odometry_error(i-1, 6:8) = abs(rotm2eul(T_error(1:3,1:3), 'XYZ'));
          Odometry_error(i-1, 9) = norm(T_error(1:3,4));
          Odometry_error(i-1,10) = acos( 0.5 * (trace(T_error(1:3,1:3)) - 1) );

      end
      Odometry_test(find(Odometry_test(:,1) == 0), :) = [];
      Odom_gt(find(Odom_gt(:,1)==0), :) = [];
      Odometry_error(find(Odometry_error(:,1)==0), :) = [];
      Odometry.test = Odometry_test;
      Odometry.gt = Odom_gt;
      Odometry.error.timeseries = Odometry_error;
      Odometry.error.MeanTransError = mean(Odometry_error(:,9));
      Odometry.error.MeanRotError = mean(Odometry_error(:,10));
end