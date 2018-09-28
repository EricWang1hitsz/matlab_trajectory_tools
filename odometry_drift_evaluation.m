%% Initialization

clc; close all; clear all;
addpath(genpath(pwd));

%% Loading the data

% params %
rosbag_path = '/home/tree/data/vicon_room/2018-09-24-15-25-06.bag';
%%%%%%%%%%

bag_select = rosbag(rosbag_path);
ros_data_helper = RosDataHelper();

vicon_pose_select = select(bag_select, 'Topic', '/vicon/anymal/anymal');
vicon_transform_stamped_messages = vicon_pose_select.readMessages;
vicon_poses_raw = ros_data_helper.convertTransformStampedMessages(vicon_transform_stamped_messages);
clear vicon_pose_select vicon_transform_stamped_messages;

odom_pose_select = select(bag_select, 'Topic', '/state_estimator/pose_in_odom');
odom_pose_with_covariance_stamped_messages = odom_pose_select.readMessages;
odom_poses_raw = ros_data_helper.convertPoseWithCovarianceStampedMessages(odom_pose_with_covariance_stamped_messages);
clear odom_pose_select odom_pose_with_covariance_stamped_messages;

aligner = PoseTrajectoryAligner6Dof();
[odom_poses, vicon_poses] = aligner.truncateAndResampleDatastreams(odom_poses_raw, vicon_poses_raw); % = [T_IB , T_JV]
clear odom_poses_raw vicon_poses_raw;



%% evaluating slices

T_VB = Transformation();

% params %
T_VB.initializeFromMatrix([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]); %.setData([1 0 0 0], [0 0 0]); % TODO
vicon_poses.applyStaticTransformRHS(T_VB);
alignment_slice = 50;
evaluation_slice = 300;
alignment_orientation_scaling = 10.0;
tight_plot_padding = 10;
plot_resolution = 600;
output_path = pwd;
plot_format = 'pdf';
trajectory_viz_step = evaluation_slice-1;
trajectory_viz_axes_length = 0.1;
trajectory_viz_vicon_symbol = '-k';
trajectory_viz_odom_symbol = '-b';
num_histogram_buckets = 20;
plot_slices = true;
%%%%%%%%%%

num_slices = floor(odom_poses.length/evaluation_slice);

orientation_drifts = zeros(num_slices,1);
position_drifts = zeros(num_slices,1);

for slice_id = 1:num_slices

    start_index = 1+(slice_id-1)*evaluation_slice;
    end_index = slice_id*evaluation_slice;
    odom_poses_slice = odom_poses.getWindowedTrajectory(start_index, end_index);
    vicon_poses_slice = vicon_poses.getWindowedTrajectory(start_index, end_index);
    
    T_alignment = aligner.calculateAlignmentTransform(odom_poses_slice.getWindowedTrajectory(1, alignment_slice),...
                                                      vicon_poses_slice.getWindowedTrajectory(1, alignment_slice),...
                                                      alignment_orientation_scaling);

    start_time = min([odom_poses_slice.times(1) vicon_poses_slice.times(1)]);
    vicon_poses_slice = TransformationTrajectory(vicon_poses_slice.orientations,...
                                                 vicon_poses_slice.positions,...
                                                 vicon_poses_slice.times - start_time);
    odom_poses_slice = TransformationTrajectory(odom_poses_slice.orientations,...
                                                odom_poses_slice.positions,...
                                                odom_poses_slice.times - start_time);
    vicon_poses_slice_aligned = vicon_poses_slice.applyStaticTransformLHS(T_alignment);
    
    dS_orientation = 0;
    dS_position = 0;
    for i=1:vicon_poses_slice_aligned.length-1
        dS_orientation = dS_orientation + norm(k_quat_diff(vicon_poses_slice_aligned.orientations(i+1,:),...
                                                           vicon_poses_slice_aligned.orientations(i,:)));
        dS_position = dS_position + norm(vicon_poses_slice_aligned.positions(i+1,:)...
                                         - vicon_poses_slice_aligned.positions(i,:));
    end
    dX_orientation = norm(k_quat_diff(odom_poses_slice.orientations(end,:),...
                                      vicon_poses_slice_aligned.orientations(end,:)));
    dX_orientation_init = norm(k_quat_diff(odom_poses_slice.orientations(1,:),...
                                           vicon_poses_slice_aligned.orientations(1,:)));
                 
    dX_position = norm(odom_poses_slice.positions(end,:)...
                       - vicon_poses_slice_aligned.positions(end,:));
    dX_position_init = norm(odom_poses_slice.positions(1,:)...
                            - vicon_poses_slice_aligned.positions(1,:));
                        
    orientation_drifts(slice_id) = dX_orientation/dS_orientation;
    position_drifts(slice_id) = dX_position/dS_position;
    
    if (plot_slices)
        h=figure(slice_id);
        set(gcf,'Visible', 'off');
        subplot(1,1,1)
        title(['Orientation/position drift: ' num2str(orientation_drifts(slice_id)) '/' num2str(position_drifts(slice_id))]);
        vicon_poses_slice_aligned.plot(trajectory_viz_step, trajectory_viz_axes_length, trajectory_viz_vicon_symbol)
        hold on
        odom_poses_slice.plot(trajectory_viz_step, trajectory_viz_axes_length, trajectory_viz_odom_symbol)
        hold off
        axis equal
        grid on
        saveTightFigure(h, [output_path '/alignment_slice_' num2str(slice_id)], plot_format, plot_resolution, tight_plot_padding);
        close all; clear h;
    end
end

h=figure();
subplot(1,2,1)
set(gcf,'Visible', 'off');
histogram(orientation_drifts, num_histogram_buckets);
title(['Orientation drift mean/std: ' num2str(round(mean(orientation_drifts), 3)) '/' num2str(round(std(orientation_drifts), 3))]);
subplot(1,2,2)
histogram(position_drifts, num_histogram_buckets);
title(['Position drift mean/std: ' num2str(round(mean(position_drifts), 3)) '/' num2str(round(std(position_drifts), 3))]);
saveTightFigure(h, [output_path '/drift_histogram'], plot_format, plot_resolution, tight_plot_padding);
close all;