function [] = position_drift_analysis(topic_config,... % T_IB (odometry), T_JV (groundtruth localization)
                                      dS_position_per_slice,...
                                      output_path,...
                                      varargin)

%% Parameters and prep
          
tight_plot_padding = 10;
initial_cut_length = 100;
local_alignment_slice_length = 15;
global_alignment_slice_length = 500;
plot_resolution = 500;
plot_format = 'jpeg';
trajectory_viz_axes_length = 0.1;
trajectory_viz_groundtruth_symbol = '--k';
trajectory_viz_odom_symbol = '-m';
num_histogram_buckets = 20;
plot_slices = true;
plot_histogram = true;
plot_aligned_trajectories = true;
aligner = PositionTrajectoryAligner4Dof();
fast_groundtruth = false;
if (length(varargin) == 1)
    fast_groundtruth = varargin{1};
end

%% Loading the data

poses = bags2poses(topic_config);
if(fast_groundtruth)
    [groundtruth_poses, odom_poses] = aligner.truncateAndResampleDatastreams(poses(2), poses(1));
else
    [odom_poses, groundtruth_poses] = aligner.truncateAndResampleDatastreams(poses(1), poses(2));
end

odom_poses = odom_poses.getWindowedTrajectory(initial_cut_length, odom_poses.length);
groundtruth_poses = groundtruth_poses.getWindowedTrajectory(initial_cut_length, groundtruth_poses.length);

%% Evaluating slices

position_drifts = zeros(0,0);
dS_position = 0;
pre_position_index = 1;
dS_position_tot = 0;

for i = 1:groundtruth_poses.length-1

    dS_position_increment = norm(groundtruth_poses.positions(i+1,:)...
                                 - groundtruth_poses.positions(i,:));
    dS_position = dS_position + dS_position_increment;
    dS_position_tot = dS_position_tot + dS_position_increment;
                                 
    if (dS_position > dS_position_per_slice)
        slice_length = i - pre_position_index + 1;

        if(slice_length > 2*local_alignment_slice_length)
            odom_poses_slice = odom_poses.getWindowedTrajectory(pre_position_index, i+1);
            groundtruth_poses_slice = groundtruth_poses.getWindowedTrajectory(pre_position_index, i+1);

            T_alignment = aligner.calculateAlignmentTransform(odom_poses_slice.getWindowedTrajectory(1, local_alignment_slice_length),...
                                                              groundtruth_poses_slice.getWindowedTrajectory(1, local_alignment_slice_length));

            groundtruth_poses_slice_aligned = groundtruth_poses_slice.applyStaticTransformLHS(T_alignment);

            dX_position_initial_norm = norm(odom_poses_slice.positions(1,:)...
                                          - groundtruth_poses_slice_aligned.positions(1,:));
            dX_position_final_norm = norm(odom_poses_slice.positions(end,:)...
                                          - groundtruth_poses_slice_aligned.positions(end,:));
            if (dX_position_initial_norm<dX_position_final_norm)
                cur_position_drift = dX_position_final_norm/dS_position;
                position_drifts = [position_drifts; cur_position_drift];

                if (plot_slices)
                    close all;
                    h=figure();
                    set(gcf,'Visible', 'off');
                    subplot(1,1,1)
                    title(['Position drift during ' num2str(slice_length) ' iterations: ' num2str(round(cur_position_drift,3))]);
                    groundtruth_poses_slice_aligned.plot(trajectory_viz_groundtruth_symbol)
                    hold on
                    odom_poses_slice.plot(trajectory_viz_odom_symbol)
                    hold off
                    axis equal
                    grid on
                    xlabel('_{I}x [m]');
                    ylabel('_{I}y [m]');
                    lineobjects = findobj(gca,'Type','line');
                    legend([lineobjects(3) lineobjects(6)],{[topic_config(1).pose_id ' odometry'],[topic_config(2).pose_id ' ground truth']},...
                           'Interpreter', 'None');
                    saveTightFigure(h,...
                                    [output_path '/' topic_config(1).pose_id '_position_alignment_slice_' num2str(size(position_drifts,1))],...
                                    plot_format,...
                                    plot_resolution,...
                                    tight_plot_padding);
                end
            else
                disp(['Alignment failed, skipping slice!'])
            end
        else
            disp(['Number of iterations in orientation slice is smaller than twice alignment slice length!'])
        end
        
        pre_position_index = i+1;
        dS_position = 0;
    end
end

if(plot_histogram)
    close all;
    h=figure();
    histogram(position_drifts, num_histogram_buckets);
    title({['Position drift mean/std/N: ' num2str(round(mean(position_drifts), 3)) '/'...
          num2str(round(std(position_drifts), 3)) '/'...
          num2str(size(position_drifts,1))], ['for ' topic_config(1).pose_id], ' '}, 'Interpreter', 'none');
    xlabel('Drift [1]');
    ylabel('n [1]');
    saveTightFigure(h,...
                    [output_path '/' topic_config(1).pose_id '_drift_histograms'],...
                    plot_format,...
                    plot_resolution,...
                    tight_plot_padding);
end

if(plot_aligned_trajectories)
    T_alignment = aligner.calculateAlignmentTransform(odom_poses.getWindowedTrajectory(1, global_alignment_slice_length),...
                                                      groundtruth_poses.getWindowedTrajectory(1, global_alignment_slice_length));
                                                  
    groundtruth_poses_aligned = groundtruth_poses.applyStaticTransformLHS(T_alignment);
    
    dX_position_final_norm = norm(odom_poses.positions(end,:)...
                                  - groundtruth_poses_aligned.positions(end,:));

    tot_position_drift = dX_position_final_norm/dS_position_tot;
    
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
    title(['Position drift over full trajectory: ' num2str(round(tot_position_drift,3))]);
    plot3(odom_poses.positions(:,1),...
          odom_poses.positions(:,2),...
          odom_poses.positions(:,3), trajectory_viz_odom_symbol);
    plot3(odom_poses.positions(1,1),...
          odom_poses.positions(1,2),...
          odom_poses.positions(1,3), 'go');
    plot3(odom_poses.positions(end,1),...
          odom_poses.positions(end,2),...
          odom_poses.positions(end,3), 'ro');
    axis equal
    grid on
    xlabel('_{I}x [m]');
    ylabel('_{I}y [m]');
    zlabel('_{I}z [m]');
    lineobjects = findobj(gca,'Type','line');
    legend([lineobjects(3) lineobjects(6)],{[topic_config(1).pose_id ' odometry'],[topic_config(2).pose_id ' ground truth']},...
           'Interpreter', 'None', 'Location', 'southoutside');
    hold off
    saveTightFigure(h,...
                    [output_path '/' topic_config(1).pose_id '_trajectories'],...
                    plot_format,...
                    plot_resolution,...
                    tight_plot_padding);
end

end