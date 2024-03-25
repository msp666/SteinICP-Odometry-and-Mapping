function pose_transformed = Transform2InitCoordiate(pose, start_idx, varargin)
         pose_transformed = zeros(size(pose));
         pose_transformed(:,1) = pose(:,1);
         if length(varargin) == 0
            Rotm2Init = (eul2rotm(pose(start_idx, 5:7), 'XYZ'))';
            % Rotm2Init = eul2rotm(pose(start_idx, 5:7),"XYZ");
            for i = 1:length(pose)
                 [pose_transformed(i,5:7), alt] = rotm2eul( Rotm2Init * eul2rotm(pose(i,5:7), "XYZ"),'XYZ');
                 % pose_transformed(i,5:7) = [pose_tmp(1), pose_tmp(2), pose_tmp(3)];
            end
            pose_transformed(:, 2:4) = ( Rotm2Init * (pose(:,2:4) - pose(start_idx, 2:4))')';
         end
end