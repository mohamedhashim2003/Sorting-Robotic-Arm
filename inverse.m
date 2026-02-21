disp('Please enter the desired end-effector position and link lengths when prompted.');

% Getting the link length from user
a1 = input('Enter link length a1 in meters: ');
a2 = input('Enter link length a2 in meters: ');
a3 = input('Enter link length a3 in meters: ');

%Getting the end effector information
x = input('Enter desired x-coordinate of the end-effector in meters: ');
y = input('Enter desired y-coordinate of the end-effector in meters: ');
phi_deg = input('Enter desired end-effector orientation (phi) in degrees: ');
phi = phi_deg * pi / 180; 
% We are calculating wrist position here
wx = x - a3 * cos(phi);
wy = y - a3 * sin(phi);

% Calculate the distance from base to wrist
r = sqrt(wx^2 + wy^2);

% Checking the reachability
if r > (a1 + a2) || r < abs(a1 - a2)
    disp('Error: The desired position is not reachable with the given link lengths.');
    return;
end

% Compute theta2
cos_theta2 = (r^2 - a1^2 - a2^2) / (2 * a1 * a2);
% To avoid numerical errors
cos_theta2 = max(min(cos_theta2, 1), -1);
theta2_elbow_up = acos(cos_theta2);  % Elbow up solution
theta2_elbow_down = -acos(cos_theta2); % Elbow down solution

% Compute theta1 (angle of link 1 relative to x-axis)
% theta1 = atan2(wy, wx) - angle to the wrist point
beta_elbow_up = atan2(a2 * sin(theta2_elbow_up), a1 + a2 * cos(theta2_elbow_up));
beta_elbow_down = atan2(a2 * sin(theta2_elbow_down), a1 + a2 * cos(theta2_elbow_down));
theta1_elbow_up = atan2(wy, wx) - beta_elbow_up;
theta1_elbow_down = atan2(wy, wx) - beta_elbow_down;

% Compute theta3 (Which adjusts orientation)
% phi = theta1 + theta2 + theta3, so theta3 = phi - theta1 - theta2
theta3_elbow_up = phi - theta1_elbow_up - theta2_elbow_up;
theta3_elbow_down = phi - theta1_elbow_down - theta2_elbow_down;

% Convert all angles to degrees
theta1_up_deg = theta1_elbow_up * 180 / pi;
theta2_up_deg = theta2_elbow_up * 180 / pi;
theta3_up_deg = theta3_elbow_up * 180 / pi;

theta1_down_deg = theta1_elbow_down * 180 / pi;
theta2_down_deg = theta2_elbow_down * 180 / pi;
theta3_down_deg = theta3_elbow_down * 180 / pi;

% Display the results
disp('Solution 1 (Elbow Up):');
disp(['theta1 = ' num2str(theta1_up_deg) ' degrees']);
disp(['theta2 = ' num2str(theta2_up_deg) ' degrees']);
disp(['theta3 = ' num2str(theta3_up_deg) ' degrees']);
disp(' ');

disp('Solution 2 (Elbow Down):');
disp(['theta1 = ' num2str(theta1_down_deg) ' degrees']);
disp(['theta2 = ' num2str(theta2_down_deg) ' degrees']);
disp(['theta3 = ' num2str(theta3_down_deg) ' degrees']);