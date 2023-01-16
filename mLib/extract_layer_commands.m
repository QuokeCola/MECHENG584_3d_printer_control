function [layer_commands]=extract_layer_commands(path, angle_threshold, length_threshold)
    p_diff = diff(path,1,2);
    l_diff = vecnorm(p_diff(1:2,:));
    a_diff = unwrap(angle(p_diff(1,:)+1j*p_diff(2,:)));
    a_diff = diff(a_diff);
    long_linear_traj_start_idx = find(l_diff > length_threshold);
    sharp_turning_idx = find(abs(a_diff)>angle_threshold)+1;
    traj_start_end_idx = union(long_linear_traj_start_idx,long_linear_traj_start_idx+1);
    traj_start_end_idx = union(traj_start_end_idx,sharp_turning_idx);
    if ~any(traj_start_end_idx(:) == 1)
        traj_start_end_idx = [1,traj_start_end_idx];
    end
    if ~any(traj_start_end_idx(:) == length(path))
        traj_start_end_idx = [traj_start_end_idx,length(path)];
    end
    layer_commands = cell(1,length(traj_start_end_idx)-1);
    for i = 1:(length(traj_start_end_idx)-1)
        command.points = path(:,traj_start_end_idx(i):traj_start_end_idx(i+1));
        if i == 1
            % Need full stop when start a new layer
            command.start_angle = inf;
        else 
            command.start_angle = abs(a_diff(traj_start_end_idx(i)-1));
        end
        if i~=(length(traj_start_end_idx)-1)
            command.end_angle = abs(a_diff(traj_start_end_idx(i+1)-1));
        else
            command.end_angle = inf;
        end
        layer_commands{i}=command;
    end
    layer_commands = horzcat(layer_commands{:});
%     debug = 1;
%     if debug
%         figure(3)
%         plot(path(1,:),path(2,:));
%         hold on
%         for i = 1:length(traj_start_end_idx)
%             plot(path(1,traj_start_end_idx(i)),path(2,traj_start_end_idx(i)),'ro');
%         end
%         axis equal
%     end
end