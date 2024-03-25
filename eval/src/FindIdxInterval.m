function [idx_test, idx_gt] = FindIdxInterval(pose, ground_truth)
        
            for i = 1:length(pose)
                 % idx_gt_temp = find( abs(pose(i,1)-ground_truth(:,1))<0.05 );
                 time_diff = abs(pose(i,1)-ground_truth(:,1));
                 [~,idx_gt_temp] = min(time_diff);
                 if time_diff(idx_gt_temp) > 0.05
                     continue;
                 end

                 if isempty(idx_gt_temp)
                     continue;
                 else
                    idx_test.start = i;
                    idx_gt.start = idx_gt_temp;
                    break;
                 end
            end
        
            for i = length(pose):-1:1
                % idx_gt_temp = find( abs(pose(i,1)-ground_truth(:,1))<0.05 );
                time_diff = abs(pose(i,1)-ground_truth(:,1));
                 [~,idx_gt_temp] = min(time_diff);
                 if time_diff(idx_gt_temp) >0.05
                     continue;
                 end

                if isempty(idx_gt_temp)
                    continue;
                else
                    idx_test.end = i;
                    idx_gt.end = idx_gt_temp;
                    break;
                end
            end
       
end