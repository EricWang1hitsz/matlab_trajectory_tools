%% Initialization

clc; close all; clear all;
addpath(genpath(pwd));

%% Parameters

rosbag_path = '/home/tree/data/vicon_room/2018-09-24-15-25-06.bag';
alignment_orientation_scaling = 1.0;
trajectory_viz_step = 1000;
trajectory_viz_axes_length = 0.33;
trajectory_viz_vicon_symbol = '-b';
trajectory_viz_odom_symbol = '-k';

tight_plot_padding = 10;
plot_resolution = 600;
output_path = pwd;
plot_format = 'pdf';

%% Loading the data

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

%% Resampling and transforming

aligner = PoseTrajectoryAligner6Dof();
[odom_poses, vicon_poses] = aligner.truncateAndResampleDatastreams(odom_poses_raw, vicon_poses_raw);
T_alignment = aligner.calculateAlignmentTransform(odom_poses.getWindowedTrajectory(1,50), vicon_poses.getWindowedTrajectory(1,50), alignment_orientation_scaling);

start_time = min([odom_poses.times(1) vicon_poses.times(1)]);
vicon_poses = TransformationTrajectory(vicon_poses.orientations,...
                                       vicon_poses.positions,...
                                       vicon_poses.times - start_time);
odom_poses = TransformationTrajectory(odom_poses.orientations,...
                                      odom_poses.positions,...
                                      odom_poses.times - start_time);
vicon_poses_aligned = vicon_poses.applyStaticTransformLHS(T_alignment);

%% Visualization

h=figure(1);

subplot(1,2,1)
vicon_poses.getWindowedTrajectory(1,1000).plot(trajectory_viz_step, trajectory_viz_axes_length, trajectory_viz_vicon_symbol)
hold on
odom_poses.getWindowedTrajectory(1,1000).plot(trajectory_viz_step, trajectory_viz_axes_length, trajectory_viz_odom_symbol);
hold off
title('raw')
axis equal
grid on
subplot(1,2,2)
vicon_poses_aligned.getWindowedTrajectory(1,1000).plot(trajectory_viz_step, trajectory_viz_axes_length, trajectory_viz_vicon_symbol)
hold on
odom_poses.getWindowedTrajectory(1,1000).plot(trajectory_viz_step, trajectory_viz_axes_length, trajectory_viz_odom_symbol)
hold off
title('aligned')
axis equal
grid on

saveTightFigure(h, [output_path '/alignment_vicon_vs_tsif.pdf'], plot_format, plot_resolution, tight_plot_padding);
