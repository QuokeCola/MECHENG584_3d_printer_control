function [commands] = extract_commands(path,angle_threshold,linear_threshold)
    debug = 0;
    % Extract layers paths of model. for rest paths, just simply use linear
    % interpolation. 
    % Rest parts are some movements like toolhead cleaning or back to origin. 

    % Retrieve model path.
    modelpath = path(:,9:end-5);
    layers_heights = unique(modelpath(3,:));
    pre_commands = cell(1,8);
    after_commands = cell(1,5);
    layer_commands = cell(1,length(layers_heights));
    if debug
        iterate_range = 1:1;
    else
        iterate_range = 1:length(layers_heights);
    end
    for i=(iterate_range)
        layer_commands{i} = extract_layer_commands(modelpath(:,modelpath(3,:)==layers_heights(i)),angle_threshold,linear_threshold);
        if i < iterate_range(end)
            last_layer_idx = find(modelpath(3,:)==layers_heights(i),1,'last');
            command.points = [modelpath(:,last_layer_idx),modelpath(:,last_layer_idx+1)];
            command.start_angle = inf;
            command.end_angle = inf;
            layer_commands{i} = [layer_commands{i},command];
        end
    end
    for i = 1:8
        pre_commands{i}.points = [path(:,i),path(:,i+1)];
        pre_commands{i}.start_angle = inf;
        pre_commands{i}.end_angle = inf;
    end
    for i = (length(path) - 5):(length(path)-1)
        after_commands{i}.points = [path(:,i),path(:,i+1)];
        after_commands{i}.start_angle = inf;
        after_commands{i}.end_angle = inf;
    end
    commands = [horzcat(pre_commands{:}),horzcat(layer_commands{:}),horzcat(after_commands{:})];
end