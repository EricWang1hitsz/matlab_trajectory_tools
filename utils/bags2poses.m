function [poses] = bags2poses(config)

    ros_data_helper = RosDataHelper();
    start_time = inf;

    for i = 1:size(config,2)

        bag_select = rosbag(config(i).rosbag_path);

        pose_select = select(bag_select, 'Topic', config(i).topic);

        if (config(i).num_msgs < 0)
            config(i).num_msgs = pose_select.NumMessages;
        else
            if (pose_select.NumMessages < config(i).num_msgs)
                disp(['Number of requested msgs for pose ' num2str(i) ' exceeds number of msgs in bag (' num2str(pose_select.NumMessages) '<' num2str(config(i).num_msgs) ')!'])
            end
        end

        ros_msgs = pose_select.readMessages([1:config(i).msg_skip:config(i).num_msgs]);
        if (strcmp(config(i).msg_type,'PoseWithCovarianceStamped'))
            tmp_poses = ros_data_helper.convertPoseWithCovarianceStampedMessages(ros_msgs);
            poses(i).times = tmp_poses.times;
            poses(i).positions = tmp_poses.positions;
            poses(i).orientations = tmp_poses.orientations;
        elseif (strcmp(config(i).msg_type,'TransformStamped'))
            tmp_poses = ros_data_helper.convertTransformStampedMessages(ros_msgs);
            poses(i).times = tmp_poses.times;
            poses(i).positions = tmp_poses.positions;
            poses(i).orientations = tmp_poses.orientations;
        elseif (strcmp(config(i).msg_type,'Odometry'))
            tmp_poses = ros_data_helper.convertOdometryMessages(ros_msgs);
            poses(i).times = tmp_poses.times;
            poses(i).positions = tmp_poses.positions;
            poses(i).orientations = tmp_poses.orientations;
        elseif (strcmp(config(i).msg_type,'PointStamped'))
            tmp_poses = ros_data_helper.convertPointStampedMessages(ros_msgs);
            poses(i).times = tmp_poses.times;
            poses(i).positions = tmp_poses.points;
            poses(i).orientations = [1, 0, 0, 0];
        else
            disp(['Unknown msg type "' config(i).msg_type '" for pose ' num2str(i) '!']);
        end

        start_time = min([start_time poses(i).times(1)]);

        clear ros_msgs;
    
    end
    
    for i=1:size(poses,2)
        poses(i).times = poses(i).times - start_time;
        poses(i).orientations = k_quat_norm(poses(i).orientations);
    end

end
