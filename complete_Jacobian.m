disp('Please enter the link lengths and joint angles when prompted.');

% Input link lengths and joint angles
a1 = input('Enter link length a1 in meters: ');
a2 = input('Enter link length a2 in meters: ');
a3 = input('Enter link length a3 in meters: ');

theta1_deg = input('Enter theta1 in degrees: ');
theta2_deg = input('Enter theta2 in degrees: ');
theta3_deg = input('Enter theta3 in degrees: ');

% Convert angles to radians
theta1 = theta1_deg * pi / 180;
theta2 = theta2_deg * pi / 180;
theta3 = theta3_deg * pi / 180;

% Define rotation matrices as anonymous functions
Rot_z = @(theta) [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
Rot_y = @(theta) [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];

% Base frame
p0 = [0;0;0];
R0 = eye(3);
z0 = [0;0;1];  

% Joint 1: Base rotation around z
R1 = Rot_z(theta1);
p1 = p0 + R1 * [a1;0;0];
z1 = R1 * [0;1;0];  % Joint 2 rotates around y-axis in local frame

% Joint 2: Rotation around y after Joint 1
R2 = R1 * Rot_y(theta2);
p2 = p1 + R2 * [a2;0;0];
z2 = R2 * [0;0;1];  % Joint 3 rotates around z-axis in local frame

% End-effector: Rotation around z after Joint 2
R3 = R2 * Rot_z(theta3);
p3 = p2 + R3 * [a3;0;0];

% Build the 6x3 Jacobian
J = zeros(6,3);
% Linear velocity part
J(1:3,1) = cross(z0, p3 - p0);
J(1:3,2) = cross(z1, p3 - p1);
J(1:3,3) = cross(z2, p3 - p2);
% Angular velocity part
J(4:6,1) = z0;
J(4:6,2) = z1;
J(4:6,3) = z2;

% Display Jacobian
disp(' Jacobian matrix is  (6x3):');
disp(J);

% Compute and display inverse
J_inv = pinv(J);
disp('Inverse Jacobian matrix is (3x6):');
disp(J_inv);