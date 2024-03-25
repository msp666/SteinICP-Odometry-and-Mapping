particle_sel = select(ros2_bag_steinicp, Topic="/stein_icp/particles");
particle_msg = readMessages(particle_sel);

%% single frame 
idx = 500
figure
stem(particle_msg{idx,1}.x,particle_msg{idx,1}.weights)
hold on
disp("average: " + num2str(mean(particle_msg{idx,1}.x)));
disp("weighted mean: " + num2str(sum(particle_msg{idx,1}.x.*particle_msg{idx,1}.weights)));

scatter(particle_msg{idx,1}.y, zeros(length(particle_msg{idx,1}.x), 1), 'filled')

%% weighted mean VS average
particle_ave = table('Size',[1,6], ...
    'VariableTypes', {'double', 'double','double','double','double','double'},...
    'VariableNames',{'x','y','z','roll','pitch','yaw'});
particle_weightmean = table('Size',[1,6], ...
    'VariableTypes', {'double', 'double','double','double','double','double'},...
    'VariableNames',{'x','y','z','roll','pitch','yaw'});
for i = 1:length(particle_msg)
    particle_ave(i,:) = {mean(particle_msg{i,1}.x),...
                         mean(particle_msg{i,1}.y),...
                         mean(particle_msg{i,1}.z),...
                         mean(particle_msg{i,1}.roll),...
                         mean(particle_msg{i,1}.pitch),...
                         mean(particle_msg{i,1}.yaw)};
    particle_weightmean(i,:) = {sum(particle_msg{i,1}.x.*particle_msg{i,1}.weights),...
                           sum(particle_msg{i,1}.y.*particle_msg{i,1}.weights),...
                           sum(particle_msg{i,1}.z.*particle_msg{i,1}.weights),...
                           sum(particle_msg{i,1}.roll.*particle_msg{i,1}.weights),...
                           sum(particle_msg{i,1}.pitch.*particle_msg{i,1}.weights),...
                           sum(particle_msg{i,1}.yaw.*particle_msg{i,1}.weights) };
end


%% weight cov
for i = 1:length(particle_msg)
    cov_weighted(i,1) = sum((particle_msg{i,1}.x - particle_weightmean.x(i)).^2 ...
                           .*particle_msg{i,1}.weights);
    cov_weighted(i,2) = sum((particle_msg{i,1}.y - particle_weightmean.y(i)).^2 ...
                           .*particle_msg{i,1}.weights);
    cov_weighted(i,3) = sum((particle_msg{i,1}.z - particle_weightmean.z(i)).^2 ...
                           .*particle_msg{i,1}.weights);
    cov_weighted(i,4) = sum((particle_msg{i,1}.roll - particle_weightmean.roll(i)).^2 ...
                           .*particle_msg{i,1}.weights);
    cov_weighted(i,5) = sum((particle_msg{i,1}.pitch - particle_weightmean.pitch(i)).^2 ...
                           .*particle_msg{i,1}.weights);
    cov_weighted(i,6) = sum((particle_msg{i,1}.yaw - particle_weightmean.yaw(i)).^2 ...
                           .*particle_msg{i,1}.weights);

end

