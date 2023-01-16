% Ts       [sec]      sampling time
% feedrate [mm/sec]   desired feedrate
% accel    [mm/sec^2] accel/decel ratio
% gcode_file_name     your gcode file name

function [simulink_commands] = parse_gcode(Ts,feedrate,accel,gcode_file_name)
addpath('mLib')
% Load gcode and generate trajectory
path = read_gcode(gcode_file_name);
commands = extract_commands(path,pi/3,4);
interp_path = cell(1,length(commands));
% Interpolate trajectories
fe = 0;
continuous_threshold = pi/8;
simulink_commands = cell(1,length(commands));
maxlength = 4;
for i = 1:(length(commands)-1)
    % distinguish it's a line or spline
    traj_size = size(commands(i).points);
    next_traj_size = size(commands(i+1).points);
    fs = fe;
    % calculate next trajectory length
    if(next_traj_size(2)==2)
        next_L = commands(i+1).points(:,1)-commands(i+1).points(:,2);
        next_L = vecnorm(next_L(1:3));
    else
        if next_traj_size(2)>maxlength
            maxlength = next_traj_size(2);
        end
        next_L = splineTBI(commands(i+1).points,0,0,feedrate,accel,Ts);
        next_L = sum(vecnorm(diff(next_L(1:2,:),1,2)));
    end
    % determine the feedrate start and feedrate end by turning angle and
    % next path length. (For now, when short path occurs, a full stop
    % occurs)
    % Explaination. If next path is too short that the length is shorter
    % than 1/2*accel*(desired feedrate/accel)^2, the start feedrate should
    % decreased and the current path feedrate also need to decreased. If
    % every path is short, it requires a lot of back iteration, it become
    % expensive for calculation.
    if(commands(i).end_angle > continuous_threshold || ...
      (commands(i+1).end_angle>continuous_threshold && next_L < 1/2*accel*(feedrate/accel)^2))
        fe = 0;
    else 
        fe = feedrate;
    end
    simulink_command.fs = fs;
    simulink_command.fe = fe;
    if(traj_size(2)==2)
        [interp_path{i},fe,L] = linearTBI(commands(i).points(:,1),commands(i).points(:,2),fs,fe,feedrate,accel,Ts);
    else
        [interp_path{i},fe,L] = splineTBI(commands(i).points,fs,fe,feedrate,accel,Ts);
    end
    simulink_command.length_L = L;
    simulink_command.length_N = size(interp_path{i},2);
    simulink_commands{i} = simulink_command;
end
% Interpolate last command
[interp_path{end},~,L] = linearTBI(commands(end).points(:,1),commands(end).points(:,2),fe,0,feedrate,accel,1e-3);
path_size = size(interp_path{end});
simulink_command.fs = 0;
simulink_command.fe = 0;
simulink_command.length_L = L;
simulink_command.length_N = path_size(2);
simulink_commands{end} = simulink_command;
hold off
% Add gcode points to commands.
% figure(1)
% hold on
for i = 1:length(simulink_commands)
    simulink_commands{i}.points = [commands(i).points, repelem(-114514,4,maxlength-size(commands(i).points,2))];
end
interp_path = horzcat(interp_path{:});
simulink_commands = horzcat(simulink_commands{:});
% give a short interpolated path for test. It will not blow up dSPACE's
% memory :D
x_ref = interp_path(1,:);
y_ref = interp_path(2,:);
z_ref = interp_path(3,:);
e_ref = interp_path(4,:);

save('traj_test.mat','x_ref','y_ref','z_ref','e_ref');
save('commands.mat','simulink_commands')
figure(2)
hold on
subplot(4,2,1:4)
axis equal
hold on
% plot3(path(1,:),path(2,:),path(3,:),'ro')
plot3(interp_path(1,:),interp_path(2,:),interp_path(3,:))
subplot(4,2,5)
plot(interp_path(1,:))
subplot(4,2,6)
plot(interp_path(2,:))
subplot(4,2,7)
plot(interp_path(3,:))
subplot(4,2,8)
plot(interp_path(4,:))
end