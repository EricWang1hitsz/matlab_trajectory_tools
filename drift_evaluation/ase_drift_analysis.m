function [] = ase_drift_analysis(duration, dS_position_per_slice, dS_orientation_per_slice, rosbag_path, output_path, data_id)

%% Parameters

T_VB = Transformation();
T_VB.setData([0.99991483442036189...
             -0.0023132531975060644...
              0.0050250814398940576...
              0.011820377416460024],...
              [0.15148234027608337...
               0.0022390589981119815...
              -0.32892799573666176]);
alignment_slice_length = 1;
alignment_orientation_scaling = 10.0;
tight_plot_padding = 10;
plot_resolution = 600;
plot_format = 'pdf';
trajectory_viz_axes_length = 0.1;
trajectory_viz_vicon_symbol = '-k';
trajectory_viz_odom_symbol = '-b';
num_histogram_buckets = 20;
plot_slices = true;
odom_frequency = 400;
odom_msg_skip = 2;
orientation_alignment_threshold = 0.017;
position_alignment_threshold = 0.02;

%% Loading the data

num_odom_msgs = odom_frequency * duration;
bag_select = rosbag(rosbag_path);
ros_data_helper = RosDataHelper();

odom_pose_select = select(bag_select, 'Topic', '/state_estimator/pose_in_odom');
if (num_odom_msgs < 0)
    num_odom_msgs = odom_pose_select.NumMessages;
else
    if (odom_pose_select.NumMessages < num_odom_msgs)
        disp(['Number of requested odometry msgs exceeds number of msgs in bag (' num2str(odom_pose_select.NumMessages) '<' num2str(num_odom_msgs) ')!'])
    end
end
odom_pose_with_covariance_stamped_messages = odom_pose_select.readMessages([1:odom_msg_skip:num_odom_msgs]);
odom_poses_raw = ros_data_helper.convertPoseWithCovarianceStampedMessages(odom_pose_with_covariance_stamped_messages);
clear odom_pose_select odom_pose_with_covariance_stamped_messages;

vicon_pose_select = select(bag_select, 'Topic', '/vicon/anymal/anymal');
vicon_transform_stamped_messages = vicon_pose_select.readMessages;
vicon_poses_raw = ros_data_helper.convertTransformStampedMessages(vicon_transform_stamped_messages);
clear vicon_pose_select vicon_transform_stamped_messages;

if (alignment_slice_length == 1)
    aligner = PoseTrajectoryAlignerFirstPose();
else
    aligner = PoseTrajectoryAligner6Dof();
end
[odom_poses, vicon_poses] = aligner.truncateAndResampleDatastreams(odom_poses_raw, vicon_poses_raw); % = [T_IB , T_JV]
clear odom_poses_raw vicon_poses_raw;

vicon_poses.applyStaticTransformRHS(T_VB);

%% Evaluating slices

orientation_drifts = zeros(0,0);
position_drifts = zeros(0,0);
dS_orientation = 0;
dS_position = 0;
pre_orientation_index = 1;
pre_position_index = 1;

for i = 1:vicon_poses.length-1

    dS_orientation = dS_orientation + k_quat_diff_mag(vicon_poses.orientations(i+1,:),...
                                                      vicon_poses.orientations(i,:));
    dS_position = dS_position + norm(vicon_poses.positions(i+1,:)...
                                     - vicon_poses.positions(i,:));
                                     
    if (dS_orientation > dS_orientation_per_slice)
        slice_length = i - pre_orientation_index + 1;
        
        if (slice_length >= alignment_slice_length) 
            
            odom_poses_slice = odom_poses.getWindowedTrajectory(pre_orientation_index, i+1);
            vicon_poses_slice = vicon_poses.getWindowedTrajectory(pre_orientation_index, i+1);
            
            T_alignment = aligner.calculateAlignmentTransform(odom_poses_slice.getWindowedTrajectory(1, alignment_slice_length),...
                                                              vicon_poses_slice.getWindowedTrajectory(1, alignment_slice_length),...
                                                              alignment_orientation_scaling);

            start_time = min([odom_poses_slice.times(1) vicon_poses_slice.times(1)]);
            vicon_poses_slice = TransformationTrajectory(vicon_poses_slice.orientations,...
                                                         vicon_poses_slice.positions,...
                                                         vicon_poses_slice.times - start_time);
            odom_poses_slice = TransformationTrajectory(odom_poses_slice.orientations,...
                                                        odom_poses_slice.positions,...
                                                        odom_poses_slice.times - start_time);
            vicon_poses_slice_aligned = vicon_poses_slice.applyStaticTransformLHS(T_alignment);
            
            dX_orientation_final = k_quat_diff(odom_poses_slice.orientations(end,:),...
                                               vicon_poses_slice_aligned.orientations(end,:));
            dX_orientation_final_norm = k_quat_diff_mag(odom_poses_slice.orientations(end,:),...
                                                         vicon_poses_slice_aligned.orientations(end,:));
            dX_position_final = odom_poses_slice.positions(end,:)...
                                - vicon_poses_slice_aligned.positions(end,:);
                            
            dX_orientation_init = k_quat_diff(odom_poses_slice.orientations(1,:),...
                                              vicon_poses_slice_aligned.orientations(1,:));
            dX_orientation_init_norm = k_quat_diff_mag(odom_poses_slice.orientations(1,:),...
                                                        vicon_poses_slice_aligned.orientations(1,:));
            dX_position_init = odom_poses_slice.positions(1,:)...
                               - vicon_poses_slice_aligned.positions(1,:);
                          
            if((dX_orientation_init_norm < orientation_alignment_threshold) && (norm(dX_position_init) < position_alignment_threshold))
                if((norm(dX_position_init) < norm(dX_position_final)) && (dX_orientation_init_norm < dX_orientation_final_norm))
                    cur_orientation_drift = dX_orientation_final_norm/dS_orientation;
                    orientation_drifts = [orientation_drifts; cur_orientation_drift];
                    
                    if (plot_slices)
                        close all;
                        h=figure();
                        set(gcf,'Visible', 'off');
                        subplot(1,1,1)
                        title(['Orientation drift during ' num2str(slice_length) ' iterations: ' num2str(round(cur_orientation_drift,3))]);
                        vicon_poses_slice_aligned.plot(vicon_poses_slice_aligned.length-1, trajectory_viz_axes_length, trajectory_viz_vicon_symbol)
                        hold on
                        odom_poses_slice.plot(odom_poses_slice.length-1, trajectory_viz_axes_length, trajectory_viz_odom_symbol)
                        hold off
                        axis equal
                        grid on
                        saveTightFigure(h,...
                                        [output_path '/' data_id '_orientation_alignment_slice_' num2str(size(orientation_drifts,1))],...
                                        plot_format,...
                                        plot_resolution,...
                                        tight_plot_padding);
                    end
                else
                    disp('Initial alignment error is larger than final absolute drift!');
                end
            else
                disp('Initial alignment error exceeds threshold!');
            end
        else
            disp(['Number of iterations in orientation slice is smaller than alignment slice length (' num2str(slice_length) '<' num2str(alignment_slice_length) ')!'])
        end
        
        pre_orientation_index = i+1;
        dS_orientation = 0;
    end
                                 
    if (dS_position > dS_position_per_slice)
        slice_length = i - pre_position_index + 1;
        
        if (slice_length >= alignment_slice_length) 
            
            odom_poses_slice = odom_poses.getWindowedTrajectory(pre_position_index, i+1);
            vicon_poses_slice = vicon_poses.getWindowedTrajectory(pre_position_index, i+1);
            
            T_alignment = aligner.calculateAlignmentTransform(odom_poses_slice.getWindowedTrajectory(1, alignment_slice_length),...
                                                              vicon_poses_slice.getWindowedTrajectory(1, alignment_slice_length),...
                                                              alignment_orientation_scaling);

            start_time = min([odom_poses_slice.times(1) vicon_poses_slice.times(1)]);
            vicon_poses_slice = TransformationTrajectory(vicon_poses_slice.orientations,...
                                                         vicon_poses_slice.positions,...
                                                         vicon_poses_slice.times - start_time);
            odom_poses_slice = TransformationTrajectory(odom_poses_slice.orientations,...
                                                        odom_poses_slice.positions,...
                                                        odom_poses_slice.times - start_time);
            vicon_poses_slice_aligned = vicon_poses_slice.applyStaticTransformLHS(T_alignment);
            
            dX_orientation_final = k_quat_diff(odom_poses_slice.orientations(end,:),...
                                               vicon_poses_slice_aligned.orientations(end,:));
            dX_orientation_final_norm = k_quat_diff_mag(odom_poses_slice.orientations(end,:),...
                                                         vicon_poses_slice_aligned.orientations(end,:));
            dX_position_final = odom_poses_slice.positions(end,:)...
                                - vicon_poses_slice_aligned.positions(end,:);
                            
            dX_orientation_init = k_quat_diff(odom_poses_slice.orientations(1,:),...
                                              vicon_poses_slice_aligned.orientations(1,:));
            dX_orientation_init_norm = k_quat_diff_mag(odom_poses_slice.orientations(1,:),...
                                                        vicon_poses_slice_aligned.orientations(1,:));
            dX_position_init = odom_poses_slice.positions(1,:)...
                               - vicon_poses_slice_aligned.positions(1,:);
            
            if((dX_orientation_init_norm < orientation_alignment_threshold) && (norm(dX_position_init) < position_alignment_threshold))
                if((norm(dX_position_init) < norm(dX_position_final)) && (dX_orientation_init_norm < dX_orientation_final_norm))
                    cur_position_drift = norm(dX_position_final)/dS_position;
                    position_drifts = [position_drifts; cur_position_drift];

                    if (plot_slices)
                        close all;
                        h=figure();
                        set(gcf,'Visible', 'off');
                        subplot(1,1,1)
                        title(['Position drift during ' num2str(slice_length) ' iterations: ' num2str(round(cur_position_drift,3))]);
                        vicon_poses_slice_aligned.plot(vicon_poses_slice_aligned.length-1, trajectory_viz_axes_length, trajectory_viz_vicon_symbol)
                        hold on
                        odom_poses_slice.plot(odom_poses_slice.length-1, trajectory_viz_axes_length, trajectory_viz_odom_symbol)
                        hold off
                        axis equal
                        grid on
                        saveTightFigure(h,...
                                        [output_path '/' data_id '_position_alignment_slice_' num2str(size(position_drifts,1))],...
                                        plot_format,...
                                        plot_resolution,...
                                        tight_plot_padding);
                    end
                else
                    disp('Initial alignment error is larger than final absolute drift!');
                end
            else
                disp('Initial alignment error exceeds threshold!');
            end
        else
            disp(['Number of iterations in position slice is smaller than alignment slice length (' num2str(slice_length) '<' num2str(alignment_slice_length) ')!'])
        end
        
        pre_position_index = i+1;
        dS_position = 0;
    end
end

close all;
h=figure();
subplot(1,2,1)
set(gcf,'Visible', 'off');
histogram(orientation_drifts, num_histogram_buckets);
title(['Orientation drift mean/std/N: ' num2str(round(mean(orientation_drifts), 3)) '/' num2str(round(std(orientation_drifts), 3)) '/' num2str(size(orientation_drifts,1))]);
subplot(1,2,2)
histogram(position_drifts, num_histogram_buckets);
title(['Position drift mean/std/N: ' num2str(round(mean(position_drifts), 3)) '/' num2str(round(std(position_drifts), 3)) '/' num2str(size(position_drifts,1))]);
saveTightFigure(h,...
                [output_path '/' data_id '_drift_histograms'],...
                plot_format,...
                plot_resolution,...
                tight_plot_padding);
close all;

end