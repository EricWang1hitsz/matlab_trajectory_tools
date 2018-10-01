function [] = poses2csv(otput_file, poses)
    % v = [t, x, y, z, q_x, q_y, q_z, q_w]
    data = zeros(size(poses.times,1), 8);

    data(:, 1) = poses.times;
    data(:, 2:4) = poses.positions;
    data(:, 5:7) = poses.orientations(:, 2:4);
    data(:, 8) = poses.orientations(:, 1);
    dlmwrite(otput_file, data,'delimiter',',','precision',10);
end
