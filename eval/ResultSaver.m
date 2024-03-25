%% save results

ResultsToSave = struct;
ResultsToSave.Parameters = steinicp_param{1};
ResultsToSave.Pose = pose_stein;
ResultsToSave.Covariace = cov_stein;
ResultsToSave.Error.TrajectoryError = TrajectoryError_Stein;
ResultsToSave.Error.PositionError2D = PositionError_stein;
ResultsToSave.Error.MeanPositionError = MeanPositionError_Stein;
ResultsToSave.Error.MeanTrajectoryError = MeanTrajecortyError;
ResultsToSave.Runtime.Preprocessing = Runtime_steinicp(:,1);
ResultsToSave.Runtime.SteinICP = Runtime_steinicp(:,2);
ResultsToSave.Runtime.PreprocessinMean = mean(Runtime_steinicp(:,1));
ResultsToSave.Runtime.SteinICPMean = mean(Runtime_steinicp(:,2));
ResultsToSave.Odometry = BetweenPose_Stein;

%% Kiss Save
ResultsToSave = struct;
ResultsToSave.pose = pose_KISS;
ResultsToSave.Odometry = BetweenPose_kiss;
ResultsToSave.Odometry = OdomMeanError_kiss;
ResultsToSave.Error.TrajectoryError = TrajectoryError_KISS;
ResultsToSave.Error.PositionError2D = PositionError_KISS;
ResultsToSave.Error.MeanPositionError = MeanPositionError_KISS;
ResultsToSave.Error.MeanTrajectoryError = MeanTrajecortyError_KISS;


%% 
desired_name.('Day_SegVoxel02') = ResultsToSave;
save('Results/BR/Day/SegVoxel02', '-struct','desired_name');
clear desired_name

