function [] = drift_analysis(topic_config,... % T_IB (odometry), T_JV (groundtruth localization)
                             T_VB,...
                             dS_position_per_slice,...
                             dS_orientation_per_slice,...
                             output_path)

%% Parameters and prep
          
tight_plot_padding = 10;
plot_resolution = 600;
plot_format = 'pdf';
trajectory_viz_axes_length = 0.1;
trajectory_viz_vicon_symbol = ':k';
trajectory_viz_odom_symbol = '-k';
num_histogram_buckets = 20;
plot_slices = true;

aligner = PoseTrajectoryAlignerFirstPose();

%% Loading the data

poses = bags2poses(topic_config);

[odom_poses, groundtruth_poses] = aligner.truncateAndResampleDatastreams(poses(1), poses(2));

groundtruth_poses.applyStaticTransformRHS(T_VB);

%% Evaluating slices

orientation_drifts = zeros(0,0);
position_drifts = zeros(0,0);
dS_orientation = 0;
dS_position = 0;
pre_orientation_index = 1;
pre_position_index = 1;

for i = 1:groundtruth_poses.length-1

    dS_orientation = dS_orientation + k_quat_diff_mag(groundtruth_poses.orientations(i+1,:),...
                                                      groundtruth_poses.orientations(i,:));
    dS_position = dS_position + norm(groundtruth_poses.positions(i+1,:)...
                                     - groundtruth_poses.positions(i,:));
                                     
    if (dS_orientation > dS_orientation_per_slice)
        slice_length = i - pre_orientation_index + 1;
        
        if (slice_length >= 2) 
            
            odom_poses_slice = odom_poses.getWindowedTrajectory(pre_orientation_index, i+1);
            vicon_poses_slice = groundtruth_poses.getWindowedTrajectory(pre_orientation_index, i+1);
            
            T_alignment = aligner.calculateAlignmentTransform(odom_poses_slice,...
                                                              vicon_poses_slice,...
                                                              1);

            vicon_poses_slice_aligned = vicon_poses_slice.applyStaticTransformLHS(T_alignment);
            
            dX_orientation_final_norm = k_quat_diff_mag(odom_poses_slice.orientations(end,:),...
                                                         vicon_poses_slice_aligned.orientations(end,:));
                          
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
                xlabel('_{I}x [m]');
                ylabel('_{I}y [m]');
                lineobjects = findobj(gca,'Type','line');
                legend([lineobjects(3) lineobjects(6)],{'Odometry','Ground truth'});
                saveTightFigure(h,...
                                [output_path '/' topic_config(1).pose_id '_orientation_alignment_slice_' num2str(size(orientation_drifts,1))],...
                                plot_format,...
                                plot_resolution,...
                                tight_plot_padding);
            end

        else
            disp(['Number of iterations in orientation slice is smaller than 2!'])
        end
        
        pre_orientation_index = i+1;
        dS_orientation = 0;
    end
                                 
    if (dS_position > dS_position_per_slice)
        slice_length = i - pre_position_index + 1;
        
        if (slice_length >= 2) 
            
            odom_poses_slice = odom_poses.getWindowedTrajectory(pre_position_index, i+1);
            vicon_poses_slice = groundtruth_poses.getWindowedTrajectory(pre_position_index, i+1);
            
            T_alignment = aligner.calculateAlignmentTransform(odom_poses_slice,...
                                                              vicon_poses_slice,...
                                                              1);

            vicon_poses_slice_aligned = vicon_poses_slice.applyStaticTransformLHS(T_alignment);
            
            dX_position_final_norm = norm(odom_poses_slice.positions(end,:)...
                                          - vicon_poses_slice_aligned.positions(end,:));
            
            cur_position_drift = dX_position_final_norm/dS_position;
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
                xlabel('_{I}x [m]');
                ylabel('_{I}y [m]');
                lineobjects = findobj(gca,'Type','line');
                legend([lineobjects(3) lineobjects(6)],{'Odometry','Ground truth'});
                saveTightFigure(h,...
                                [output_path '/' topic_config(1).pose_id '_position_alignment_slice_' num2str(size(position_drifts,1))],...
                                plot_format,...
                                plot_resolution,...
                                tight_plot_padding);
            end
 
        else
            disp(['Number of iterations in orientation slice is smaller than 2!'])
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
xlabel('Drift [1]');
ylabel('n [1]');
subplot(1,2,2)
histogram(position_drifts, num_histogram_buckets);
title(['Position drift mean/std/N: ' num2str(round(mean(position_drifts), 3)) '/' num2str(round(std(position_drifts), 3)) '/' num2str(size(position_drifts,1))]);
xlabel('Drift [1]');
ylabel('n [1]');
saveTightFigure(h,...
                [output_path '/' topic_config(1).pose_id '_drift_histograms'],...
                plot_format,...
                plot_resolution,...
                tight_plot_padding);
close all;

end