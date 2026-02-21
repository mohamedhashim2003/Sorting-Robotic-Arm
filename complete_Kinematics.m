function complete_kinematics
% 3-DOF Robotic Arm Interactive GUI
% This GUI allows users to:
% 1. Set link parameters (a, alpha, d, theta)
% 2. Manipulate joint angles
% 3. Visualize the robotic arm
% 4. Calculate forward kinematics
% 5. Calculate Jacobian and inverse Jacobian
% 6. Perform inverse kinematics

% Create the main figure
fig = figure('Name', '3-DOF Robotic Arm GUI', 'Position', [100, 100, 1000, 700], ...
             'NumberTitle', 'off', 'MenuBar', 'none', 'Resize', 'on');

% Default DH parameters for a 3-DOF robotic arm
% Format: [a, alpha, d, theta]
default_dh = [0.3, pi/2, 0.1, 0;
              0.4, 0, 0, 0;
              0.2, 0, 0, 0];
          
% Current joint angles (in radians)
current_angles = [0, 0, 0];

% Current end-effector position
current_position = [0, 0, 0];

% Create panels
param_panel = uipanel(fig, 'Title', 'DH Parameters', 'Position', [0.01, 0.7, 0.28, 0.28]);
control_panel = uipanel(fig, 'Title', 'Control', 'Position', [0.01, 0.4, 0.28, 0.28]);
fk_panel = uipanel(fig, 'Title', 'Forward Kinematics', 'Position', [0.01, 0.2, 0.28, 0.18]);
jacobian_panel = uipanel(fig, 'Title', 'Jacobian', 'Position', [0.01, 0.01, 0.48, 0.18]);
ik_panel = uipanel(fig, 'Title', 'Inverse Kinematics', 'Position', [0.31, 0.2, 0.18, 0.48]);

% Create a panel for the 3D visualization
vis_panel = uipanel(fig, 'Title', 'Visualization', 'Position', [0.5, 0.2, 0.48, 0.78]);
ax = axes('Parent', vis_panel);

% Create UI elements for DH parameters
dh_labels = {'Link', 'a (m)', 'alpha (rad)', 'd (m)', 'theta (rad)'};
for i = 1:5
    uicontrol(param_panel, 'Style', 'text', 'String', dh_labels{i}, ...
              'Position', [10+70*(i-1), 160, 60, 20], 'HorizontalAlignment', 'center');
end

dh_edit = cell(3, 4);
for i = 1:3
    uicontrol(param_panel, 'Style', 'text', 'String', num2str(i), ...
              'Position', [10, 160-30*i, 60, 20], 'HorizontalAlignment', 'center');
    
    for j = 1:4
        dh_edit{i, j} = uicontrol(param_panel, 'Style', 'edit', ...
                                 'String', num2str(default_dh(i, j)), ...
                                 'Position', [10+70*j, 160-30*i, 60, 20], ...
                                 'Callback', @(src, event) updateDH(src, event, i, j));
    end
end

% Create UI elements for joint control
uicontrol(control_panel, 'Style', 'text', 'String', 'Joint Angles (rad)', ...
          'Position', [10, 160, 180, 20], 'HorizontalAlignment', 'center');

joint_labels = {'Joint 1:', 'Joint 2:', 'Joint 3:'};
joint_edits = cell(1, 3);

for i = 1:3
    uicontrol(control_panel, 'Style', 'text', 'String', joint_labels{i}, ...
              'Position', [10, 160-30*i, 60, 20], 'HorizontalAlignment', 'left');
    
    joint_edits{i} = uicontrol(control_panel, 'Style', 'edit', ...
                               'String', '0', ...
                               'Position', [80, 160-30*i, 60, 20], ...
                               'Callback', @(src, event) updateJointAngle(src, event, i));
    
    uicontrol(control_panel, 'Style', 'pushbutton', 'String', 'Set', ...
              'Position', [150, 160-30*i, 40, 20], ...
              'Callback', @(src, event) setJointAngle(src, event, i));
end

% Forward kinematics display
fk_text = cell(1, 3);
fk_labels = {'X (m):', 'Y (m):', 'Z (m):'};
for i = 1:3
    uicontrol(fk_panel, 'Style', 'text', 'String', fk_labels{i}, ...
              'Position', [10, 100-25*i, 60, 20], 'HorizontalAlignment', 'left');
    
    fk_text{i} = uicontrol(fk_panel, 'Style', 'text', 'String', '0', ...
                           'Position', [80, 100-25*i, 60, 20], ...
                           'HorizontalAlignment', 'left', 'BackgroundColor', [1, 1, 1]);
end

% Calculate FK button
uicontrol(fk_panel, 'Style', 'pushbutton', 'String', 'Calculate FK', ...
          'Position', [150, 75, 100, 30], ...
          'Callback', @calculateFK);

% Jacobian display
uicontrol(jacobian_panel, 'Style', 'text', 'String', 'Jacobian Matrix:', ...
          'Position', [10, 110, 100, 20], 'HorizontalAlignment', 'left');

jacobian_text = uicontrol(jacobian_panel, 'Style', 'text', 'String', '', ...
                         'Position', [10, 40, 200, 70], 'HorizontalAlignment', 'left', ...
                         'BackgroundColor', [1, 1, 1]);

% Inverse Jacobian display
uicontrol(jacobian_panel, 'Style', 'text', 'String', 'Inverse Jacobian:', ...
          'Position', [220, 110, 100, 20], 'HorizontalAlignment', 'left');

inv_jacobian_text = uicontrol(jacobian_panel, 'Style', 'text', 'String', '', ...
                             'Position', [220, 40, 200, 70], 'HorizontalAlignment', 'left', ...
                             'BackgroundColor', [1, 1, 1]);

% Calculate Jacobian button
uicontrol(jacobian_panel, 'Style', 'pushbutton', 'String', 'Calculate Jacobian', ...
          'Position', [150, 10, 150, 30], ...
          'Callback', @calculateJacobian);

% Inverse Kinematics panel
ik_inputs = cell(1, 3);
ik_labels = {'Target X (m):', 'Target Y (m):', 'Target Z (m):'};
for i = 1:3
    uicontrol(ik_panel, 'Style', 'text', 'String', ik_labels{i}, ...
              'Position', [10, 300-30*i, 80, 20], 'HorizontalAlignment', 'left');
    
    ik_inputs{i} = uicontrol(ik_panel, 'Style', 'edit', 'String', '0', ...
                             'Position', [100, 300-30*i, 60, 20]);
end

uicontrol(ik_panel, 'Style', 'pushbutton', 'String', 'Solve IK', ...
          'Position', [40, 180, 100, 30], ...
          'Callback', @solveIK);

% Calculate IK results display
uicontrol(ik_panel, 'Style', 'text', 'String', 'IK Solution (rad):', ...
          'Position', [10, 140, 100, 20], 'HorizontalAlignment', 'left');

ik_result_text = cell(1, 3);
for i = 1:3
    uicontrol(ik_panel, 'Style', 'text', 'String', ['θ' num2str(i) ':'], ...
              'Position', [10, 140-25*i, 30, 20], 'HorizontalAlignment', 'left');
    
    ik_result_text{i} = uicontrol(ik_panel, 'Style', 'text', 'String', '0', ...
                                 'Position', [40, 140-25*i, 120, 20], ...
                                 'HorizontalAlignment', 'left', 'BackgroundColor', [1, 1, 1]);
end

% Initialize the visualization
updateVisualization();

% Callback functions
    function updateDH(src, ~, row, col)
        % Update DH parameters when user changes them
        value = str2double(get(src, 'String'));
        if ~isnan(value)
            default_dh(row, col) = value;
            updateVisualization();
        else
            % Reset to previous value if input is invalid
            set(src, 'String', num2str(default_dh(row, col)));
        end
    end

    function updateJointAngle(src, ~, joint_idx)
        % Update joint angle when user changes it in the edit box
        value = str2double(get(src, 'String'));
        if ~isnan(value)
            current_angles(joint_idx) = value;
        else
            % Reset to previous value if input is invalid
            set(src, 'String', num2str(current_angles(joint_idx)));
        end
    end

    function setJointAngle(~, ~, joint_idx)
        % Set joint angle and update visualization
        value = str2double(get(joint_edits{joint_idx}, 'String'));
        if ~isnan(value)
            current_angles(joint_idx) = value;
            updateVisualization();
            calculateFK();
        end
    end

    function calculateFK(~, ~)
        % Calculate forward kinematics
        T = forwardKinematics(default_dh, current_angles);
        
        % Extract position from transformation matrix
        position = T(1:3, 4);
        current_position = position';
        
        % Update position display
        for i = 1:3
            set(fk_text{i}, 'String', num2str(position(i), '%.4f'));
        end
    end

    function calculateJacobian(~, ~)
        % Calculate Jacobian and inverse Jacobian
        J = calculateJacobianMatrix(default_dh, current_angles);
        
        % Format Jacobian for display
        J_str = '';
        for i = 1:size(J, 1)
            row_str = '';
            for j = 1:size(J, 2)
                row_str = [row_str, sprintf('%.3f ', J(i, j))];
            end
            J_str = [J_str, row_str, '\n'];
        end
        set(jacobian_text, 'String', J_str);
        
        % Calculate and display inverse Jacobian
        try
            invJ = inv(J);
            invJ_str = '';
            for i = 1:size(invJ, 1)
                row_str = '';
                for j = 1:size(invJ, 2)
                    row_str = [row_str, sprintf('%.3f ', invJ(i, j))];
                end
                invJ_str = [invJ_str, row_str, '\n'];
            end
            set(inv_jacobian_text, 'String', invJ_str);
        catch
            set(inv_jacobian_text, 'String', 'Singular configuration!\nInverse Jacobian does not exist.');
        end
    end

    function solveIK(~, ~)
        % Get target position
        target_pos = zeros(1, 3);
        for i = 1:3
            target_pos(i) = str2double(get(ik_inputs{i}, 'String'));
        end
        
        % Solve inverse kinematics using Jacobian-based method
        [success, solution] = inverseKinematics(default_dh, current_angles, target_pos);
        
        % Update solution display
        if success
            for i = 1:3
                set(ik_result_text{i}, 'String', num2str(solution(i), '%.4f'));
            end
            
            % Update current angles with solution
            current_angles = solution;
            
            % Update joint angle inputs
            for i = 1:3
                set(joint_edits{i}, 'String', num2str(solution(i), '%.4f'));
            end
            
            % Update visualization
            updateVisualization();
            calculateFK();
        else
            for i = 1:3
                set(ik_result_text{i}, 'String', 'No solution');
            end
        end
    end

    function updateVisualization()
        % Clear the current axes
        cla(ax);
        
        % Calculate transformation matrices for each link
        T = eye(4);
        positions = zeros(4, 3);  % Store joint positions [base, joint1, joint2, end-effector]
        
        % Base position
        positions(1, :) = [0, 0, 0];
        
        % Calculate positions of each joint
        for i = 1:3
            dh_row = default_dh(i, :);
            a = dh_row(1);
            alpha = dh_row(2);
            d = dh_row(3);
            theta = dh_row(4) + current_angles(i);  % Add current joint angle
            
            % Calculate transformation matrix
            ct = cos(theta);
            st = sin(theta);
            ca = cos(alpha);
            sa = sin(alpha);
            
            A = [ct, -st*ca, st*sa, a*ct;
                 st, ct*ca, -ct*sa, a*st;
                 0, sa, ca, d;
                 0, 0, 0, 1];
            
            T = T * A;
            positions(i+1, :) = T(1:3, 4)';
        end
        
        % Plot robot links
        hold(ax, 'on');
        plot3(ax, positions(:, 1), positions(:, 2), positions(:, 3), 'k-', 'LineWidth', 2);
        
        % Plot joints as spheres
        for i = 1:4
            plotSphere(ax, positions(i, :), 0.02);
        end
        
        % Plot coordinate frames
        T = eye(4);
        plotFrame(ax, T, 0.05);  % Base frame
        
        for i = 1:3
            dh_row = default_dh(i, :);
            a = dh_row(1);
            alpha = dh_row(2);
            d = dh_row(3);
            theta = dh_row(4) + current_angles(i);
            
            ct = cos(theta);
            st = sin(theta);
            ca = cos(alpha);
            sa = sin(alpha);
            
            A = [ct, -st*ca, st*sa, a*ct;
                 st, ct*ca, -ct*sa, a*st;
                 0, sa, ca, d;
                 0, 0, 0, 1];
            
            T = T * A;
            plotFrame(ax, T, 0.05);
        end
        
        % Set axis properties
        axis(ax, 'equal');
        grid(ax, 'on');
        xlabel(ax, 'X (m)');
        ylabel(ax, 'Y (m)');
        zlabel(ax, 'Z (m)');
        title(ax, '3-DOF Robotic Arm');
        
        % Set view angle
        view(ax, 30, 20);
        
        % Set axis limits based on robot dimensions
        max_dim = max(sum(abs([default_dh(:, 1), default_dh(:, 3)]))) * 1.5;
        if max_dim < 1
            max_dim = 1;
        end
        axis(ax, [-max_dim, max_dim, -max_dim, max_dim, -max_dim, max_dim]);
    end
end

function T = forwardKinematics(dh_params, joint_angles)
    % Calculate forward kinematics using DH parameters
    % dh_params: [a, alpha, d, theta] for each link
    % joint_angles: current joint angles
    
    T = eye(4);
    
    for i = 1:size(dh_params, 1)
        a = dh_params(i, 1);
        alpha = dh_params(i, 2);
        d = dh_params(i, 3);
        theta = dh_params(i, 4) + joint_angles(i);
        
        ct = cos(theta);
        st = sin(theta);
        ca = cos(alpha);
        sa = sin(alpha);
        
        A = [ct, -st*ca, st*sa, a*ct;
             st, ct*ca, -ct*sa, a*st;
             0, sa, ca, d;
             0, 0, 0, 1];
         
        T = T * A;
    end
end

function J = calculateJacobianMatrix(dh_params, joint_angles)
    % Calculate Jacobian matrix for a 3-DOF robotic arm
    % The Jacobian relates joint velocities to end-effector velocities
    
    % Initialize Jacobian matrix (6x3 for 3-DOF robot)
    J = zeros(6, 3);
    
    % Calculate transformation matrices up to each joint
    T_current = eye(4);
    T_list = cell(1, 3);
    
    for i = 1:size(dh_params, 1)
        a = dh_params(i, 1);
        alpha = dh_params(i, 2);
        d = dh_params(i, 3);
        theta = dh_params(i, 4) + joint_angles(i);
        
        ct = cos(theta);
        st = sin(theta);
        ca = cos(alpha);
        sa = sin(alpha);
        
        A = [ct, -st*ca, st*sa, a*ct;
             st, ct*ca, -ct*sa, a*st;
             0, sa, ca, d;
             0, 0, 0, 1];
         
        T_current = T_current * A;
        T_list{i} = T_current;
    end
    
    % End-effector position
    p_end = T_current(1:3, 4);
    
    % Calculate Jacobian columns
    for i = 1:3
        if i == 1
            z_i_1 = [0; 0; 1];  % z-axis of base frame
            p_i_1 = [0; 0; 0];  % origin of base frame
        else
            z_i_1 = T_list{i-1}(1:3, 3);  % z-axis of previous frame
            p_i_1 = T_list{i-1}(1:3, 4);  % origin of previous frame
        end
        
        % For revolute joint: J_v = z_{i-1} × (p_end - p_{i-1})
        J_v = cross(z_i_1, p_end - p_i_1);
        
        % For revolute joint: J_w = z_{i-1}
        J_w = z_i_1;
        
        % Combine to form the column of the Jacobian
        J(:, i) = [J_v; J_w];
    end
end

function [success, solution] = inverseKinematics(dh_params, initial_angles, target_position)
    % Solve inverse kinematics using Jacobian-based iterative method
    % dh_params: DH parameters [a, alpha, d, theta]
    % initial_angles: starting joint angles
    % target_position: desired end-effector position [x, y, z]
    
    % Parameters for the iterative method
    max_iterations = 100;
    tolerance = 1e-4;
    alpha = 0.5;  % Step size
    
    % Initialize
    current_angles = initial_angles;
    
    for iter = 1:max_iterations
        % Calculate forward kinematics
        T = forwardKinematics(dh_params, current_angles);
        current_position = T(1:3, 4)';
        
        % Calculate error
        error = target_position - current_position;
        error_norm = norm(error);
        
        % Check if we're close enough to the target
        if error_norm < tolerance
            success = true;
            solution = current_angles;
            return;
        end
        
        % Calculate Jacobian
        J = calculateJacobianMatrix(dh_params, current_angles);
        J_pos = J(1:3, :);  % We only care about position, not orientation
        
        % Calculate joint angle increments using damped least squares
        lambda = 0.1;  % Damping factor
        delta_theta = J_pos' * inv(J_pos * J_pos' + lambda^2 * eye(3)) * error';
        
        % Update joint angles
        current_angles = current_angles + alpha * delta_theta';
        
        % Joint limits handling (if needed)
        % This is a simple example keeping angles in [-pi, pi]
        current_angles = mod(current_angles + pi, 2*pi) - pi;
    end
    
    % If we get here, we didn't converge
    success = false;
    solution = current_angles;
end

function plotSphere(ax, center, radius)
    % Helper function to plot a sphere
    [X, Y, Z] = sphere(10);
    X = X * radius + center(1);
    Y = Y * radius + center(2);
    Z = Z * radius + center(3);
    surf(ax, X, Y, Z, 'FaceColor', [0.3, 0.3, 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
end

function plotFrame(ax, T, scale)
    % Helper function to plot a coordinate frame
    origin = T(1:3, 4)';
    x_axis = origin + scale * T(1:3, 1)';
    y_axis = origin + scale * T(1:3, 2)';
    z_axis = origin + scale * T(1:3, 3)';
    
    % Plot axes
    line(ax, [origin(1), x_axis(1)], [origin(2), x_axis(2)], [origin(3), x_axis(3)], 'Color', 'r', 'LineWidth', 2);
    line(ax, [origin(1), y_axis(1)], [origin(2), y_axis(2)], [origin(3), y_axis(3)], 'Color', 'g', 'LineWidth', 2);
    line(ax, [origin(1), z_axis(1)], [origin(2), z_axis(2)], [origin(3), z_axis(3)], 'Color', 'b', 'LineWidth', 2);
end