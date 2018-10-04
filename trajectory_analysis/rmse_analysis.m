function [] = drift_analysis(topic_config,... % T_IB (validation), T_JV (groundtruth localization)
                             T_VB,...
                             output_path,...
                             varargin) % fast_groundtruth, use_first_pose_aligner

%% Parameters and prep
          
tight_plot_padding = 10;
plot_resolution = 500;
plot_format = 'jpeg';
trajectory_viz_groundtruth_symbol = ':k';
trajectory_viz_validation_symbol = '-k';
aligner = PoseTrajectoryAligner6Dof();
alignment_orientation_weight = 1.;
fast_groundtruth = false;
if (length(varargin) == 1)
    fast_groundtruth = varargin{1};
elseif (length(varargin) == 2)
    fast_groundtruth = varargin{1};
    if (varargin{2})
        aligner = PoseTrajectoryAlignerFirstPose();
    else
        aligner = PoseTrajectoryAligner6Dof();
    end 
end

%% Loading the data

poses = bags2poses(topic_config);
if(fast_groundtruth)
    [groundtruth_poses, validation_poses] = aligner.truncateAndResampleDatastreams(poses(2), poses(1));
else
    [validation_poses, groundtruth_poses] = aligner.truncateAndResampleDatastreams(poses(1), poses(2));
end
groundtruth_poses.applyStaticTransformRHS(T_VB);

%% RMSE analysis

T_alignment = aligner.calculateAlignmentTransform(validation_poses,...
                                                  groundtruth_poses,...
                                                  alignment_orientation_weight);
groundtruth_poses_aligned = groundtruth_poses.applyStaticTransformLHS(T_alignment);

position_rmse = rms(cellfun(@norm, num2cell((groundtruth_poses_aligned.positions - validation_poses.positions)*100, 2))); %cm
orientation_rmse = rms(cellfun(@k_quat_diff_mag,...
                       num2cell(groundtruth_poses_aligned.orientations, 2),...
                       num2cell(validation_poses.orientations, 2))*180/pi); %deg

close all;
h=figure();
set(gcf,'Visible', 'off');
subplot(1,1,1)
plot3(groundtruth_poses_aligned.positions(:,1),...
      groundtruth_poses_aligned.positions(:,2),...
      groundtruth_poses_aligned.positions(:,3), trajectory_viz_groundtruth_symbol);
hold on;
plot3(groundtruth_poses_aligned.positions(1,1),...
      groundtruth_poses_aligned.positions(1,2),...
      groundtruth_poses_aligned.positions(1,3), 'go');
plot3(groundtruth_poses_aligned.positions(end,1),...
      groundtruth_poses_aligned.positions(end,2),...
      groundtruth_poses_aligned.positions(end,3), 'ro');
title(['Position/orientation RMSE over full trajectory: ' num2str(round(position_rmse,3)) 'cm/' num2str(round(orientation_rmse,3)) 'deg']);
plot3(validation_poses.positions(:,1),...
      validation_poses.positions(:,2),...
      validation_poses.positions(:,3), trajectory_viz_validation_symbol);
plot3(validation_poses.positions(1,1),...
      validation_poses.positions(1,2),...
      validation_poses.positions(1,3), 'go');
plot3(validation_poses.positions(end,1),...
      validation_poses.positions(end,2),...
      validation_poses.positions(end,3), 'ro');
axis equal
grid on
xlabel('_{I}x [m]');
ylabel('_{I}y [m]');
zlabel('_{I}z [m]');
lineobjects = findobj(gca,'Type','line');
legend([lineobjects(3) lineobjects(6)],{[topic_config(1).pose_id ' validation'],[topic_config(2).pose_id ' ground truth']},...
       'Interpreter', 'None', 'Location', 'southoutside');
hold off
saveTightFigure(h,...
                [output_path '/' topic_config(1).pose_id '_trajectories'],...
                plot_format,...
                plot_resolution,...
                tight_plot_padding);


end