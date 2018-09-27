%% Initialization

clc; close all; clear all;
addpath(genpath(pwd));

%% Parameters

rosbag_path = '/home/tree/data/vicon_room/edited/2018-09-24-15-25-06_minimal.bag';
alignment_orientation_scaling = 1.0;
trajectory_viz_step = 500;
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
vicon_poses = ros_data_helper.convertTransformStampedMessages(vicon_transform_stamped_messages);
clear vicon_transform_stamped_messages vicon_pose_select;

odom_pose_select = select(bag_select, 'Topic', '/state_estimator/pose_in_odom');
odom_pose_with_covariance_stamped_messages = odom_pose_select.readMessages;
odom_poses = ros_data_helper.convertPoseWithCovarianceStampedMessages(odom_pose_with_covariance_stamped_messages);
clear odom_pose_select odom_pose_with_covariance_stamped_messages;

clear bag_select ros_data_helper;

%% Resampling and transforming

aligner = PoseTrajectoryAligner6Dof();
[odom_poses_resampled, vicon_poses_resampled] = aligner.truncateAndResampleDatastreams(odom_poses, vicon_poses);
T_alignment = aligner.calculateAlignmentTransform(odom_poses_resampled, vicon_poses_resampled, alignment_orientation_scaling);
clear aligner odom_poses vicon_poses;

%% Setting up trajectories

start_time = min([odom_poses_resampled.times(1) vicon_poses_resampled.times(1)]);
vicon_trajectory_resampled = TransformationTrajectory(vicon_poses_resampled.orientations,...
                                                      vicon_poses_resampled.positions,...
                                                      vicon_poses_resampled.times - start_time);
odom_trajectory_resampled = TransformationTrajectory(odom_poses_resampled.orientations,...
                                                     odom_poses_resampled.positions,...
                                                     odom_poses_resampled.times - start_time);
vicon_trajectory_resampled_aligned = vicon_trajectory_resampled.applyStaticTransformLHS(T_alignment);

clear start_time T_alignment;

%% Visualization

h=figure(1);

subplot(1,2,1)
vicon_trajectory_resampled.plot(trajectory_viz_step, trajectory_viz_axes_length, trajectory_viz_vicon_symbol)
hold on
odom_trajectory_resampled.plot(trajectory_viz_step, trajectory_viz_axes_length, trajectory_viz_odom_symbol);
hold off
axis equal
grid on
subplot(1,2,2)
vicon_trajectory_resampled_aligned.plot(trajectory_viz_step, trajectory_viz_axes_length, trajectory_viz_vicon_symbol)
hold on
odom_trajectory_resampled.plot(trajectory_viz_step, trajectory_viz_axes_length, trajectory_viz_odom_symbol)
hold off
axis equal
grid on

saveTightFigure(h, [output_path '/full_alignment_vicon_vs_tsif.pdf'], plot_format, plot_resolution, tight_plot_padding);
