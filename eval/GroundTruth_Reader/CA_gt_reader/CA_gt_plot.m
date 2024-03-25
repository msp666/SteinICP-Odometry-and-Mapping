figure
plot(Lidar_gt(:,2), Lidar_gt(:,3));

%%
for i = 2: length(CA_gt)
    
    T_last_inv = [eul2rotm(CA_gt(i-1, 5:7), "XYZ")', -eul2rotm(CA_gt(i-1, 5:7), "XYZ")'*CA_gt(i-1,2:4)';
                   zeros(1,3) ,1];
    T_new = [eul2rotm(CA_gt(i, 5:7), "XYZ")', CA_gt(i,2:4)';
             zeros(1,3), 1];

    delta_T = T_last_inv * T_new;
    Odom_CA_gt(i, 2:4) = delta_T(1:3, 4)';
    Odom_CA_gt(i, 5:7) = rotm2eul(delta_T(1:3,1:3), "XYZ");
    
end
Odom_CA_gt(:, 1 ) = CA_gt(:, 1);

%%
figure
plot(Odom_CA_gt(2:end,1)-Odom_CA_gt(1,1), Odom_CA_gt(2:end,2))
hold on
% plot(Odom_CA_gt(2:end,1)-Odom_CA_gt(1,1), 0.05*Odom_CA_gt(2:end,2)./diff(Odom_CA_gt(:,1)))
figure
plot(Odom_CA_gt(2:end,1)-Odom_CA_gt(1,1), diff(Odom_CA_gt(:,1)))
% plot(CA_gt(2:end, 1)-CA_gt(1,1), diff(CA_gt(:,2))./diff(CA_gt(:,1)))

%%
figure
plot(CA_gt(:,1), CA_gt(:,6))