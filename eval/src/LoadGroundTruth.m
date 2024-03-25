%% Ground Truth Load function
function ground_truth = LoadGroundTruth(dataset)
        if strcmp(dataset, "NC/quad_easy");
            ground_truth = LoadGroundTruth_NC('quad_easy');
        elseif strcmp(dataset, "NC/math_easy")
            ground_truth = LoadGroundTruth_NC('math_easy');
        elseif strcmp(dataset, "Boreas/Day")
            boreas_data = load("Boreas_gt_reader/Boreas_gt_2020-12-18-13-44.mat");
            ground_truth = boreas_data.boreas_gt;
            % ground_truth(:,6) = -ground_truth(:,6);
        elseif strcmp(dataset, "Boreas/Snowing")
            boreas_data = load("Boreas_gt_reader/Boreas_gt_2021-01-26-11-22.mat");
            ground_truth = boreas_data.boreas_gt;
        elseif strcmp(dataset, "CA")
            CA = load("CA_gt_reader/CA_Lidar_gt.mat");
            ground_truth = CA.Lidar_gt;
        end
        % ground_truth(:,2:4) = ground_truth(:,2:4) - ground_truth(1, 2:4);
end

%% Newer College Ground Truth Load Function
function [ground_truth] = LoadGroundTruth_NC(collection)
        if collection == 'quad_easy'
           gt_path = "/mnt/Data/collection_1_newer_college/ground_truth/gt-nc-quad-easy.csv";
        elseif collection == 'math_easy'
           gt_path = "/mnt/Data/collection_3_maths_institute/ground_truth/gt_state_easy.csv";
        end
        gt_tmp= import_gt(gt_path, [2,inf]);
        ground_truth = zeros(size(gt_tmp, 1), size(gt_tmp, 2)-2);
        ground_truth(:, 1) = gt_tmp.sec + gt_tmp.nsec * 1e-9;
        ground_truth(:, 2:4) = [gt_tmp.x, gt_tmp.y, gt_tmp.z];
        ground_truth(:, 5:7) = quat2eul([gt_tmp.qw, gt_tmp.qx, gt_tmp.qy, gt_tmp.qz],'XYZ');
        ground_truth(:, 7) = ground_truth(:, 7);

end
