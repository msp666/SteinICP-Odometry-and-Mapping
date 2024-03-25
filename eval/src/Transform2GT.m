function [pose_aligned, matched_gt, transformation] = Transform2GT(pose, ground_truth)
          for i = 1:length(pose)
             time_diff = abs(pose(i,1)-ground_truth(:,1));
             % gt_idx_tmp = find( time_diff<0.025 );
             [~, gt_idx] = min(time_diff);
             if time_diff(gt_idx) > 0.03
                 continue;
             end
             matched_gt(i,:) = ground_truth(gt_idx, :);
          end

          % transformation = estgeotform3d(pose(:, 2:4), matched_gt(:,2:4), 'rigid');
          % frame_GT = se3(eul2rotm(matched_gt(1, 5:7), 'XYZ'), ...
          %                            matched_gt(1,2:4));
          % frame_Test = se3(eul2rotm(pose(1,5:7), 'XYZ'), ...
          %                          pose(1, 2:4));
          % transformation = rigidtform3d(tform(frame_GT) * tform(frame_Test.inv));
          [reg, GtFit, ErrState] = absor(pose(:,2:4)', matched_gt(:,2:4)');
          transformation = rigidtform3d(reg.M);

          pose_aligned(:,1) = pose(:,1);
          pose_aligned(:, 2:4) = transformPointsForward(transformation, pose(:,2:4));
          for i = 1:length(pose)
              rotm = eul2rotm(pose(i,5:7), 'XYZ');
              pose_aligned(i, 5:7) = rotm2eul(transformation.R * rotm, "XYZ")';
          end
end