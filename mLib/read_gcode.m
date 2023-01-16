%% Gcode parse functions
function [gcode_points] = read_gcode(path)
    prev_location = [0,0,0,0]';  % [x y z e f] in mm/s
    
    % Internal vars for file read
    raw_gcode_id = fopen(path); 
    
    g = textscan(raw_gcode_id,'%s','delimiter','\n');
    raw_gcode_points   = cell(1,length(g{1}));
    for i = 1:length(g{1})
        raw_gcode_line = cell2mat(g{1}(i,1));
        splited_line = strsplit(raw_gcode_line,' ');
        curr_location = get_location(splited_line);
        % Drop useless commands (Only accepts G0 and G1)
        if ~isempty(curr_location)
            % Fill the missing XYZE data
            for index_iterate = 1:4 
                if curr_location(index_iterate) == inf
                    curr_location(index_iterate) = prev_location(index_iterate);
                end
            end
            prev_location   = curr_location;
            raw_gcode_points{i} = curr_location;
        end
    end
    gcode_points = horzcat(raw_gcode_points{:});
end

function point = get_location(splited_line)
% Convention: [x y z e f], inf if parameter is missing
    line_size = size(splited_line);
    line_size = line_size(2);
    point = [];
    if splited_line(1) == "G0" || splited_line(1) == "G1"
        point = Inf(4,1);
        for i=2:line_size
            if(splited_line{i}(1)=="X")
                point(1) = str2double(splited_line{i}(2:end));
            elseif(splited_line{i}(1)=="Y")
                point(2) = str2double(splited_line{i}(2:end));
            elseif(splited_line{i}(1)=="Z")
                point(3) = str2double(splited_line{i}(2:end));
            elseif(splited_line{i}(1)=="E")
                point(4) = str2double(splited_line{i}(2:end));
            elseif(splited_line{i}(1)==";")
                break;
            end
        end
    end
end