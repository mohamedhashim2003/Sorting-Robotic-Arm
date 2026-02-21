function ForwardGUI
    % Create a figure with a specific size
    fig = figure('Name', '3-DOF Robot Arm Simulation', ...
        'Position', [100, 100, 1200, 700], ...
        'NumberTitle', 'off', ...
        'MenuBar', 'none');
    
    % Create panel for controls
    controlPanel = uipanel('Title', 'Robot Arm Controls', ...
        'Position', [0.01, 0.01, 0.25, 0.98]);
    
    % Create panel for visualization
    visPanel = uipanel('Title', 'Robot Arm Visualization', ...
        'Position', [0.27, 0.01, 0.72, 0.98]);
    
    % Create axes for 3D plot
    ax = axes('Parent', visPanel, ...
        'Position', [0.1, 0.1, 0.8, 0.8]);
    
    % Initialize DH parameters - default values
    params = struct(...
        'theta1', 0, 'd1', 1, 'a1', 0, 'alpha1', 90, ...
        'theta2', 0, 'd2', 0, 'a2', 1, 'alpha2', 0, ...
        'theta3', 0, 'd3', 0, 'a3', 1, 'alpha3', 0);
    
    % Initialize transformation matrices
    A = cell(3, 1);
    
    % Create input fields for each parameter
    paramNames = {'theta1', 'theta2', 'theta3', 'd1', 'd2', 'd3', 'a1', 'a2', 'a3', 'alpha1', 'alpha2', 'alpha3'};
    inputFields = cell(length(paramNames), 1);
    inputLabels = cell(length(paramNames), 1);
    
    % Create UI table to display transformation matrices
    matrixPanel = uipanel('Title', 'Transformation Matrices', ...
        'Position', [0.01, 0.01, 0.98, 0.35], ...
        'Parent', visPanel);
    
    % Create text areas for displaying matrices
    txtA1 = uicontrol('Style', 'text', ...
        'Position', [20, 140, 200, 80], ...
        'Parent', matrixPanel);
    txtA2 = uicontrol('Style', 'text', ...
        'Position', [240, 140, 200, 80], ...
        'Parent', matrixPanel);
    txtA3 = uicontrol('Style', 'text', ...
        'Position', [460, 140, 200, 80], ...
        'Parent', matrixPanel);
    txtT = uicontrol('Style', 'text', ...
        'Position', [240, 20, 200, 80], ...
        'Parent', matrixPanel);
    
    % Create labels for matrices
    uicontrol('Style', 'text', ...
        'Position', [80, 220, 80, 20], ...
        'String', 'Matrix A1', ...
        'Parent', matrixPanel);
    uicontrol('Style', 'text', ...
        'Position', [300, 220, 80, 20], ...
        'String', 'Matrix A2', ...
        'Parent', matrixPanel);
    uicontrol('Style', 'text', ...
        'Position', [520, 220, 80, 20], ...
        'String', 'Matrix A3', ...
        'Parent', matrixPanel);
    uicontrol('Style', 'text', ...
        'Position', [280, 100, 120, 20], ...
        'String', 'Final Transform T', ...
        'Parent', matrixPanel);
    
    % Create position display
    posInfo = uicontrol('Style', 'text', ...
        'Position', [20, 10, 640, 20], ...
        'String', 'End Effector Position: [0, 0, 0]', ...
        'HorizontalAlignment', 'left', ...
        'Parent', matrixPanel);
    
    % Create joint parameters input section
    uicontrol('Style', 'text', ...
        'Position', [10, 650, 280, 20], ...
        'String', 'Enter DH Parameters for Each Joint:', ...
        'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', ...
        'Parent', controlPanel);
    
    % Create section title for joint 1
    uicontrol('Style', 'text', ...
        'Position', [10, 620, 280, 20], ...
        'String', 'Joint 1 Parameters:', ...
        'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', ...
        'Parent', controlPanel);
    
    % Create input fields for joint parameters
    fieldWidth = 80;
    fieldHeight = 25;
    labelWidth = 80;
    spacing = 30;
    
    % Function to create input fields for a joint
    function createJointInputs(jointNum, startY)
        baseIdx = (jointNum - 1) * 4;
        
        % Theta
        paramName = ['theta', num2str(jointNum)];
        yPos = startY;
        inputLabels{baseIdx + 1} = uicontrol('Style', 'text', ...
            'Position', [10, yPos, labelWidth, fieldHeight], ...
            'String', ['θ', num2str(jointNum), ' (degrees):'], ...
            'HorizontalAlignment', 'left', ...
            'Parent', controlPanel);
        
        inputFields{baseIdx + 1} = uicontrol('Style', 'edit', ...
            'Position', [labelWidth + 10, yPos, fieldWidth, fieldHeight], ...
            'String', num2str(params.(paramName)), ...
            'Callback', {@updateParam, paramName}, ...
            'Parent', controlPanel);
        
        % d
        paramName = ['d', num2str(jointNum)];
        yPos = startY - spacing;
        inputLabels{baseIdx + 2} = uicontrol('Style', 'text', ...
            'Position', [10, yPos, labelWidth, fieldHeight], ...
            'String', ['d', num2str(jointNum), ' (meters):'], ...
            'HorizontalAlignment', 'left', ...
            'Parent', controlPanel);
        
        inputFields{baseIdx + 2} = uicontrol('Style', 'edit', ...
            'Position', [labelWidth + 10, yPos, fieldWidth, fieldHeight], ...
            'String', num2str(params.(paramName)), ...
            'Callback', {@updateParam, paramName}, ...
            'Parent', controlPanel);
        
        % a
        paramName = ['a', num2str(jointNum)];
        yPos = startY - 2*spacing;
        inputLabels{baseIdx + 3} = uicontrol('Style', 'text', ...
            'Position', [10, yPos, labelWidth, fieldHeight], ...
            'String', ['a', num2str(jointNum), ' (meters):'], ...
            'HorizontalAlignment', 'left', ...
            'Parent', controlPanel);
        
        inputFields{baseIdx + 3} = uicontrol('Style', 'edit', ...
            'Position', [labelWidth + 10, yPos, fieldWidth, fieldHeight], ...
            'String', num2str(params.(paramName)), ...
            'Callback', {@updateParam, paramName}, ...
            'Parent', controlPanel);
        
        % alpha
        paramName = ['alpha', num2str(jointNum)];
        yPos = startY - 3*spacing;
        inputLabels{baseIdx + 4} = uicontrol('Style', 'text', ...
            'Position', [10, yPos, labelWidth, fieldHeight], ...
            'String', ['α', num2str(jointNum), ' (degrees):'], ...
            'HorizontalAlignment', 'left', ...
            'Parent', controlPanel);
        
        inputFields{baseIdx + 4} = uicontrol('Style', 'edit', ...
            'Position', [labelWidth + 10, yPos, fieldWidth, fieldHeight], ...
            'String', num2str(params.(paramName)), ...
            'Callback', {@updateParam, paramName}, ...
            'Parent', controlPanel);
    end
    
    % Create input fields for all three joints
    createJointInputs(1, 590);
    
    % Create section title for joint 2
    uicontrol('Style', 'text', ...
        'Position', [10, 470, 280, 20], ...
        'String', 'Joint 2 Parameters:', ...
        'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', ...
        'Parent', controlPanel);
    createJointInputs(2, 440);
    
    % Create section title for joint 3
    uicontrol('Style', 'text', ...
        'Position', [10, 320, 280, 20], ...
        'String', 'Joint 3 Parameters:', ...
        'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', ...
        'Parent', controlPanel);
    createJointInputs(3, 290);
    
    % Add update button
    uicontrol('Style', 'pushbutton', ...
        'Position', [50, 170, 180, 40], ...
        'String', 'Update Robot Arm', ...
        'FontWeight', 'bold', ...
        'Callback', @updateButtonCallback, ...
        'Parent', controlPanel);
    
    % Preset configurations buttons
    uicontrol('Style', 'pushbutton', ...
        'Position', [50, 120, 180, 30], ...
        'String', 'Home Position', ...
        'Callback', @setHomePosition, ...
        'Parent', controlPanel);
    
    uicontrol('Style', 'pushbutton', ...
        'Position', [50, 80, 180, 30], ...
        'String', 'Reach Forward', ...
        'Callback', @setReachForward, ...
        'Parent', controlPanel);
    
    uicontrol('Style', 'pushbutton', ...
        'Position', [50, 40, 180, 30], ...
        'String', 'Reach Up', ...
        'Callback', @setReachUp, ...
        'Parent', controlPanel);
    
    % Initialize with default values
    updateRobotArm();
    
    % Functions to update parameters from text input
    function updateParam(src, ~, param)
        try
            value = str2double(get(src, 'String'));
            if isnan(value)
                % Restore previous value if input is not a number
                set(src, 'String', num2str(params.(param)));
            else
                params.(param) = value;
            end
        catch
            % Restore previous value if there's an error
            set(src, 'String', num2str(params.(param)));
        end
    end
    
    function updateButtonCallback(~, ~)
        updateRobotArm();
    end
    
    function updateRobotArm()
        % Calculate transformation matrices using DH parameters
        for i = 1:3
            % Get parameters for this joint
            if i == 1
                theta = params.theta1 * pi / 180;
                d = params.d1;
                a = params.a1;
                alpha = params.alpha1 * pi / 180;
            elseif i == 2
                theta = params.theta2 * pi / 180;
                d = params.d2;
                a = params.a2;
                alpha = params.alpha2 * pi / 180;
            else % i == 3
                theta = params.theta3 * pi / 180;
                d = params.d3;
                a = params.a3;
                alpha = params.alpha3 * pi / 180;
            end
            
            % Calculate transformation matrix
            A{i} = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
                   sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                   0, sin(alpha), cos(alpha), d;
                   0, 0, 0, 1];
        end
        
        % Calculate joint positions
        T01 = A{1};
        T02 = T01 * A{2};
        T03 = T02 * A{3};
        
        % Extract positions
        base = [0, 0, 0];
        joint1 = T01(1:3, 4)';
        joint2 = T02(1:3, 4)';
        endEffector = T03(1:3, 4)';
        
        % Update visualization
        cla(ax);
        
        % Draw coordinate system
        drawCoordinateSystem(ax, eye(4), 0.2);
        
        % Draw robot arm links
        line(ax, [base(1), joint1(1)], [base(2), joint1(2)], [base(3), joint1(3)], 'LineWidth', 3, 'Color', 'b');
        line(ax, [joint1(1), joint2(1)], [joint1(2), joint2(2)], [joint1(3), joint2(3)], 'LineWidth', 3, 'Color', 'g');
        line(ax, [joint2(1), endEffector(1)], [joint2(2), endEffector(2)], [joint2(3), endEffector(3)], 'LineWidth', 3, 'Color', 'r');
        
        % Draw joints
        drawJoint(ax, base, 'Base');
        drawJoint(ax, joint1, 'Joint 1');
        drawJoint(ax, joint2, 'Joint 2');
        drawJoint(ax, endEffector, 'End Effector');
        
        % Draw coordinate systems at each joint
        drawCoordinateSystem(ax, eye(4), 0.2); % Base
        drawCoordinateSystem(ax, T01, 0.2);    % Joint 1
        drawCoordinateSystem(ax, T02, 0.2);    % Joint 2
        drawCoordinateSystem(ax, T03, 0.2);    % End Effector
        
        % Set plot properties
        grid(ax, 'on');
        axis(ax, 'equal');
        
        % Set axis limits based on link lengths
        maxDim = max([params.d1, params.d2, params.d3, params.a1, params.a2, params.a3]) * 2.5;
        if maxDim < 2
            maxDim = 2;
        end
        axis(ax, [-maxDim, maxDim, -maxDim, maxDim, -maxDim, maxDim]);
        
        xlabel(ax, 'X');
        ylabel(ax, 'Y');
        zlabel(ax, 'Z');
        title(ax, '3-DOF Robot Arm');
        view(ax, 3);
        
        % Update text displays with matrix values
        set(txtA1, 'String', matrixToString(A{1}));
        set(txtA2, 'String', matrixToString(A{2}));
        set(txtA3, 'String', matrixToString(A{3}));
        
        % Calculate and display final transformation
        T = A{1} * A{2} * A{3};
        set(txtT, 'String', matrixToString(T));
        
        % Update end effector position
        set(posInfo, 'String', sprintf('End Effector Position: [%.2f, %.2f, %.2f]', endEffector(1), endEffector(2), endEffector(3)));
    end
    
    function str = matrixToString(mat)
        % Convert matrix to formatted string
        str = '';
        for i = 1:4
            for j = 1:4
                str = [str, sprintf('%.2f ', mat(i, j))];
            end
            str = [str, newline];
        end
    end
    
    function drawJoint(ax, position, name)
        % Draw a spherical joint at the given position
        scatter3(ax, position(1), position(2), position(3), 100, 'filled', 'MarkerEdgeColor', 'k');
        text(ax, position(1), position(2), position(3), name, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    end
    
    function drawCoordinateSystem(ax, T, scale)
        % Draw coordinate system at position specified by transformation matrix T
        origin = T(1:3, 4)';
        
        % Extract axis vectors
        x_axis = origin + scale * T(1:3, 1)';
        y_axis = origin + scale * T(1:3, 2)';
        z_axis = origin + scale * T(1:3, 3)';
        
        % Draw axis lines
        line(ax, [origin(1), x_axis(1)], [origin(2), x_axis(2)], [origin(3), x_axis(3)], 'Color', 'r', 'LineWidth', 1);
        line(ax, [origin(1), y_axis(1)], [origin(2), y_axis(2)], [origin(3), y_axis(3)], 'Color', 'g', 'LineWidth', 1);
        line(ax, [origin(1), z_axis(1)], [origin(2), z_axis(2)], [origin(3), z_axis(3)], 'Color', 'b', 'LineWidth', 1);
    end
    
    % Update input fields with specified values
    function updateInputFields()
        for i = 1:length(paramNames)
            set(inputFields{i}, 'String', num2str(params.(paramNames{i})));
        end
    end
    
    % Preset configurations
    function setHomePosition(~, ~)
        updatePresetPosition(0, 1, 0, 90, 0, 0, 1, 0, 0, 0, 1, 0);
    end
    
    function setReachForward(~, ~)
        updatePresetPosition(0, 1, 0, 90, 45, 0, 1, 0, -45, 0, 1, 0);
    end
    
    function setReachUp(~, ~)
        updatePresetPosition(0, 1, 0, 90, 90, 0, 1, 0, 0, 0, 1, 0);
    end
    
    function updatePresetPosition(t1, d1, a1, al1, t2, d2, a2, al2, t3, d3, a3, al3)
        % Set all parameters to preset values
        params.theta1 = t1;
        params.d1 = d1;
        params.a1 = a1;
        params.alpha1 = al1;
        params.theta2 = t2;
        params.d2 = d2;
        params.a2 = a2;
        params.alpha2 = al2;
        params.theta3 = t3;
        params.d3 = d3;
        params.a3 = a3;
        params.alpha3 = al3;
        
        % Update input fields
        updateInputFields();
        
        % Update visualization
        updateRobotArm();
    end
end