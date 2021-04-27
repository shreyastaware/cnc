% This is a CNC Code to 3D Machined Surface Generator
% Instructions for use:
% Keep this file and the code file in the same directory/ folder
% Add the file name to the file name variable in the 3rd from the start 
% of the MATLAB Code
% this code could be updated later with modifications
% https://github.com/shreyastaware/cnc

tic;
clc; clearvars; close all;
filename = 'code4.txt';
fid = fopen(filename);

C = textscan(fid, '%s', 'delimiter', '\n');
codelines = char(C{1});

% Interpolation Modes
Rapid_positioning = 0;
CW_interpolation = 2;
Linear_interpolation = 1;
ACW_interpolation = 3;
arc_offset_mode = 1;
radius_mode = 0;
current_mode = NaN;
circular_interpolation_mode = NaN;
spindle_state = 1;

% Initialize variables
current_position = [0,0,0];
arc_offsets = zeros(1,3); % another way - t = [0 0 0];
cell = cell(1, 5); % another way - ufk{1,5} = [];
cad_model_counter = 0;
toolPath = [];
interpolator_position = [];

for i = colon(1, size(codelines(:,1), 1))
    
    codecommands = strsplit(codelines(i,:),';');
    codecommands = codecommands{1}; % also can use codecommands(2) = [];
    codecommands = strsplit(codecommands);
    
    for j = colon(1, size(codecommands, 2))
        
        if eq(codecommands{j}(1), 'G')
            if ismember(str2double(codecommands{j}(2:end)), [0 1 2 3])
                current_mode = str2double(codecommands{j}(2:end));
            else
                continue;
            end
            
        elseif eq(codecommands{j}(1), 'X')
            current_position(1) = str2double(codecommands{j}(2:end));
        elseif eq(codecommands{j}(1), 'Y')
            current_position(2) = str2double(codecommands{j}(2:end));
        elseif eq(codecommands{j}(1), 'Z')
            current_position(3) = str2double(codecommands{j}(2:end));
            
        elseif eq(codecommands{j}(1), 'I')
            arc_offsets(1) = str2double(codecommands{j}(2:end));
            circular_interpolation_mode = arc_offset_mode;
        elseif eq(codecommands{j}(1), 'J')
            arc_offsets(2) = str2double(codecommands{j}(2:end));
            circular_interpolation_mode = arc_offset_mode;
        elseif eq(codecommands{j}(1), 'K')
            arc_offsets(3) = str2double(codecommands{j}(2:end));
            circular_interpolation_mode = arc_offset_mode;
            
        elseif eq(codecommands{j}(1), 'R')
            radius = str2double(codecommands{j}(2:end));
            circular_interpolation_mode = radius_mode;
            
        elseif eq(codecommands{j}(1), 'M')
            num = str2double(codecommands{j}(2:end));
            if eq(num, 30)
                spindle_state = 0;
            end
        end
    end
    
    if eq(spindle_state, 0)
        break;
    end
    
    if eq(current_mode, Rapid_positioning)
        if ismember(size(toolPath, 1), [0 1])
            toolPath = current_position;
            interpolator_position = [];
        else
            cad_model_counter = cad_model_counter + 1;
            cell{cad_model_counter} = toolPath;
            toolPath = current_position;
            interpolator_position = [];
        end
    
    elseif eq(current_mode, Linear_interpolation)
        
        if ~isempty(gt(toolPath, 0))
            
            interpolator_position = [ ...
                linspace(toolPath(end,1), current_position(1),100)', ...
                linspace(toolPath(end,2), current_position(2),100)', ...
                linspace(toolPath(end,3),current_position(3),100)'];
            % no need of this line; line space but point diff than below
            
            dist = norm((current_position - toolPath(end,:)));
            
            if gt(dist, 0)
                
                unit_vector = (current_position-toolPath(end,:))/dist;
                
                interpolator_position = toolPath(end,:) + unit_vector.* ...
                    linspace(0, dist, 100)';
            end
        else
            interpolator_position = current_position;
        end
        
    elseif eq(current_mode, CW_interpolation)
        
        if eq(circular_interpolation_mode, arc_offset_mode)
            
            center_position = toolPath(end,:) + arc_offsets;
            a1 = (toolPath(end,1:2)-center_position(1:2));
            a2 = (current_position(1:2)-center_position(1:2));
            
            r = norm(current_position(1:2)-center_position(1:2));
            theta_1 = atan2d(a1(2),a1(1));
            theta_2 = atan2d(a2(2),a2(1));
            
            if gt(theta_2, theta_1)
                theta_2 = theta_2-360;
            end
            
            angular_step = abs(theta_1 - theta_2)/99;
            
            cw_increment = theta_1:-angular_step: theta_2;
            interpolator_position = [center_position(1:2) + ...
                [cosd(cw_increment)', sind(cw_increment)']*r, ...
                linspace(center_position(3),current_position(3), ...
            length(cw_increment))'];
            
            interpolator_position = [interpolator_position; ...
                current_position];
            
        elseif eq(circular_interpolation_mode, radius_mode)
            syms xc yc;
            
            [x1, y1, x2, y2] = deal(toolPath(end,1),toolPath(end,2), ...
                current_position(1), current_position(2));
            
            p = (xc - x1)^2 + (yc - y1)^2;
            q = (xc - x2)^2 + (yc - y2)^2;
            
            rootsquad = solve(eq(p, radius^2), eq(q, radius^2), xc, yc);
            
            if eq(size(rootsquad.xc, 1), 1)
                
                center_position = [rootsquad.xc, rootsquad.yc];
                center_position(3) = toolPath(end,3);
                a1 = (toolPath(end,1:2)-center_position(1:2));
                a2 = (current_position(1:2)-center_position(1:2));
                
                r = norm(current_position(1:2)-center_position(1:2));
                theta_1 = atan2d(a1(2),a1(1));
                theta_2 = atan2d(a2(2),a2(1));
                
                if gt(theta_2, theta_1)
                    theta_2 = theta_2-360;
                end
                
                angular_step = abs(theta_1 - theta_2)/99;
                
                cw_increment = theta_1:-angular_step: theta_2;
                interpolator_position = [center_position(1:2) + ...
                    [cosd(cw_increment)', sind(cw_increment)']*r, ...
                    linspace(center_position(3),current_position(3), ...
                    length(cw_increment))'];
                
                interpolator_position = [interpolator_position; ...
                    current_position];
                
            else
                
                center_position = [rootsquad.xc, rootsquad.yc];
                center_position(:,3) = deal(toolPath(end,3));
                a1 = (toolPath(end,1:2) - center_position(:,1:2) );
                a2 = (current_position(1:2)  - center_position(:,1:2) );
                
                theta_1 = atan2d(a1(:,2),a1(:,1));
                theta_2 = atan2d(a2(:,2),a2(:,1));
                
                if gt(theta_2(1), theta_1(1))
                    theta_2(1) = theta_2(1)-360;
                end
                
                if gt(theta_2(2), theta_1(2))
                    theta_2(2) = theta_2(2)-360;
                end
                
                if gt(abs(theta_2(2)-theta_1(2)), ...
                        abs(theta_2(1)-theta_1(1)))
                    center_position(2,:) = [];
                    theta_1(2,:) = [];
                    theta_2(2,:) = [];
                else
                    center_position(1,:) = [];
                    theta_1(1,:) = [];
                    theta_2(1,:) = [];
                end
                
                r = norm(current_position(1:2)-center_position(1:2));
                
                angular_step = abs(theta_1 - theta_2)/99;
                
                cw_increment = theta_1:-angular_step: theta_2;
                interpolator_position = [center_position(1:2) + ...
                    [cosd(cw_increment)', sind(cw_increment)']*r, ...
                    linspace(center_position(3),current_position(3), ...
                    length(cw_increment))'];
                
                interpolator_position = [interpolator_position; ...
                    current_position];
                
            end

        end
        
        
        
    elseif eq(current_mode, ACW_interpolation)
        
        if eq(circular_interpolation_mode, arc_offset_mode)
            
            center_position = toolPath(end,:) + arc_offsets;
            a1 = (toolPath(end,1:2)-center_position(1:2));
            a2 = (current_position(1:2)-center_position(1:2));
            
            r = norm(current_position(1:2)-center_position(1:2));
            theta_1 = atan2d(a1(2),a1(1));
            theta_2 = atan2d(a2(2),a2(1));
            
            if lt(theta_2, theta_1)
                theta_2 = theta_2+360;
            end
            
            angular_step = abs(theta_1 - theta_2)/99;
            
            acw_increment = theta_1:angular_step:theta_2;
            interpolator_position = [center_position(1:2) + ...
                [cosd(acw_increment)', sind(acw_increment)']*r, ...
                linspace(center_position(3),current_position(3), ...
                length(acw_increment))'];
            
            interpolator_position = [interpolator_position; ...
                current_position];
            
        elseif eq(circular_interpolation_mode, radius_mode)
            syms xc yc;
            
            [x1, y1, x2, y2] = deal(toolPath(end,1),toolPath(end,2), ...
                current_position(1), current_position(2));
            
            p = (xc - x1)^2 + (yc - y1)^2;
            q = (xc - x2)^2 + (yc - y2)^2;
            
            rootsquad = solve(eq(p, radius^2), eq(q, radius^2), xc, yc);
            
            if eq(size(rootsquad.xc, 1), 1)
                
                center_position = [rootsquad.xc, rootsquad.yc];
                center_position(3) = toolPath(end,3);
                a1 = (toolPath(end,1:2) - center_position(:,1:2) );
                a2 = (current_position(1:2)  - center_position(:,1:2) );
                
                r = norm(current_position(1:2)-center_position(1:2));
                theta_1 = atan2d(a1(2),a1(1));
                theta_2 = atan2d(a2(2),a2(1));
                
                if lt(theta_2, theta_1)
                    theta_2 = theta_2+360;
                end
                
                angular_step = abs(theta_1 - theta_2)/99;
                
                acw_increment = theta_1:angular_step:theta_2;
                interpolator_position = [center_position(1:2) + ...
                    [cosd(acw_increment)', sind(acw_increment)']*r, ...
                    linspace(center_position(3),current_position(3), ...
                    length(acw_increment))'];
                
                interpolator_position = [interpolator_position; ...
                    current_position];
                
            else
                
                center_position = [rootsquad.xc, rootsquad.yc];
                center_position(:,3) = deal(toolPath(end,3));
                a1 = (toolPath(end,1:2) - center_position(:,1:2) );
                a2 = (current_position(1:2)  - center_position(:,1:2) );
                
                theta_1 = atan2d(a1(:,2),a1(:,1));
                theta_2 = atan2d(a2(:,2),a2(:,1));
                
                if lt(theta_2(1), theta_1(1))
                    theta_2(1) = theta_2(1)+360;
                end
                
                if lt(theta_2(2), theta_1(2))
                    theta_2(2) = theta_2(2)+360;
                end
                
                if gt(abs(theta_2(2)-theta_1(2)), ...
                        abs(theta_2(1)-theta_1(1)))
                    center_position(2,:) = [];
                    theta_1(2,:) = [];
                    theta_2(2,:) = [];
                else
                    center_position(1,:) = [];
                    theta_1(1,:) = [];
                    theta_2(1,:) = [];
                end
                
                r = norm(current_position(1:2)-center_position(1:2));
                
                angular_step = abs(theta_1 - theta_2)/99;
                
                acw_increment = theta_1:angular_step:theta_2;
                interpolator_position = [center_position(1:2) + ...
                    [cosd(acw_increment)', sind(acw_increment)']*r, ...
                    linspace(center_position(3),current_position(3), ...
                    length(acw_increment))'];
                
                interpolator_position = [interpolator_position; ...
                    current_position];
                
            end
        end
        
    end
    
    toolPath = [toolPath; interpolator_position];
    
end

if and(isempty(cell{1}), eq(cad_model_counter, 0))
    cad_model_counter = 1;
    cell{1} = toolPath;
end

model_surface = cell(1:cad_model_counter);

for l = colon(1, cad_model_counter)
    
    model_surface{l}(~any( (model_surface{l}(2:end,:) - ...
        model_surface{l}(1:end-1,:)), 2), :) = [];
    % diff does not work on symbolic variables? diff(x)
    model_surface{l}(model_surface{l}(:,3) > 0, :) = [];
    x = double([model_surface{l}(:,1)'; model_surface{l}(:,1)']);
    y = double([model_surface{l}(:,2)'; model_surface{l}(:,2)']);
    z = double([model_surface{l}(:,3)'; zeros(1, ...
        size(model_surface{1}, 1))]);
    
    if all(~double(model_surface{1}(:,3)))
        figure;
        title('Interpolation Graph');
        xlabel('X Coordinates');
        ylabel('Y Coordinates');
        zlabel('Z Coordinates');
        % axis([-60 60 -10 80 -20 20]);
        daspect([1 1 1]);
        plot3(model_surface{l}(:,1),model_surface{l}(:,2), ...
            model_surface{l}(:,3),'r-');
    end
    
    if any(model_surface{1}(:,3))
        hold on;
        plot3(model_surface{l}(:,1),model_surface{l}(:,2), ...
            model_surface{l}(:,3),'r-');
        hold off;
        figure;
        surf(x, y, z, 'FaceColor', 'g');
        hold on;
        patch(x', y', z', 'r');
        hold off;
    end
end
toc;