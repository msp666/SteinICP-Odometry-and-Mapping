%% runtime 
runtime_sel = select(ros2_bag_steinicp, Topic="/stein_icp/runtime");
runtime_msg = readMessages(runtime_sel);

Runtime_steinicp = zeros(length(runtime_msg), 2);
for i = 1:length(runtime_msg)
    Runtime_steinicp(i,1) = runtime_msg{i,1}.preprocessing_time;
    Runtime_steinicp(i,2) = runtime_msg{i,1}.steinicp_time;
end
