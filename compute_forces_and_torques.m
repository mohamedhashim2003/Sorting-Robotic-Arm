clear all;
clc;
%% Step 1: Variables naming
syms theta1 theta2 theta3 real % Joint angles
syms dtheta1 dtheta2 dtheta3 real % Joint velocities 
syms ddtheta1 ddtheta2 ddtheta3 real % Joint accelerations 

%link parameters
syms a1 a2 a3 real % Link lengths
syms m1 m2 m3 real % Link masses
syms g real % Gravity constant

 
syms Ix1 Iy1 Iz1 Ix2 Iy2 Iz2 Ix3 Iy3 Iz3 real

% DH parameters table
DH = [theta1, 0, a1, 0;
      theta2, 0, a2, 0;
      theta3, 0, a3, 0];


%% Step 2: Calculating the transformation matrices for each link

A_i = @(theta, d, a, alpha) [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
                            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                            0, sin(alpha), cos(alpha), d;
                            0, 0, 0, 1];

A1 = A_i(theta1, DH(1,2), DH(1,3), DH(1,4));
A2 = A_i(theta2, DH(2,2), DH(2,3), DH(2,4));
A3 = A_i(theta3, DH(3,2), DH(3,3), DH(3,4));

T1 = A1;
T2 = T1 * A2;
T3 = T2 * A3;

% Display symbolic transformation matrices
fprintf('\nDisplaying symbolic transformation matrix T1:\n');
disp(T1);

fprintf('\nDisplaying symbolic transformation matrix T2:\n');
disp(T2);

fprintf('\nDisplaying symbolic transformation matrix T3:\n');
disp(T3);

%% Step 3: Calculate the positions of the centers of mass

% Assuming center of mass is at the middle of each link
r1_cm = [-a1/2; 0; 0; 1]; % COM link 1 frame 1
r2_cm = [-a2/2; 0; 0; 1]; % centre of mass link 2 frame 2
r3_cm = [-a3/2; 0; 0; 1]; % link 3 frame 3

% Calculate center of mass positions in the base frame
p1_cm = T1 * r1_cm;
p2_cm = T2 * r2_cm;
p3_cm = T3 * r3_cm;

% Extract the position vectors (first 3 components)
p1_pos = p1_cm(1:3);
p2_pos = p2_cm(1:3);
p3_pos = p3_cm(1:3);


%% Step 4: Calculate the velocities of the centers of mass

% Differentiate the position vectors with respect to time
v1_cm = jacobian(p1_pos, [theta1, theta2, theta3]) * [dtheta1; dtheta2; dtheta3];
v2_cm = jacobian(p2_pos, [theta1, theta2, theta3]) * [dtheta1; dtheta2; dtheta3];
v3_cm = jacobian(p3_pos, [theta1, theta2, theta3]) * [dtheta1; dtheta2; dtheta3];

% Calculate the squared velocities
v1_squared = simplify(v1_cm.' * v1_cm);
v2_squared = simplify(v2_cm.' * v2_cm);
v3_squared = simplify(v3_cm.' * v3_cm);


%% Step 5: Calculate kinetic and potential energies

%  we assume here the inertia matrix of each link as diagonal
% with principal moments of inertia about their local coordinate frames
I1 = diag([Ix1, Iy1, Iz1]);
I2 = diag([Ix2, Iy2, Iz2]);
I3 = diag([Ix3, Iy3, Iz3]);

% Angular velocities in the base frame
omega1 = [0; 0; dtheta1];
omega2 = [0; 0; dtheta1 + dtheta2];
omega3 = [0; 0; dtheta1 + dtheta2 + dtheta3];

% Rotational kinetic energy for each link
k1_rot = (1/2) * omega1.' * I1 * omega1;
k2_rot = (1/2) * omega2.' * I2 * omega2;
k3_rot = (1/2) * omega3.' * I3 * omega3;

% Translational kinetic energy for each link
k1_trans = (1/2) * m1 * v1_squared;
k2_trans = (1/2) * m2 * v2_squared;
k3_trans = (1/2) * m3 * v3_squared;

% Total kinetic energy
k1_total = simplify(k1_rot + k1_trans);
k2_total = simplify(k2_rot + k2_trans);
k3_total = simplify(k3_rot + k3_trans);
k_total = simplify(k1_total + k2_total + k3_total);

P1 = m1 * g * p1_pos(2);
P2 = m2 * g * p2_pos(2);
P3 = m3 * g * p3_pos(2);
P_total = simplify(P1 + P2 + P3);



%% Step 6: Form the Lagrangian

% Lagrangian 
L = simplify(k_total - P_total);


%% Step 7: Calculate the D coefficients and equations of motion

% D matrix (inertia matrix for the entire system)
D = sym(zeros(3,3));

% These are the coefficients of the acceleration terms (ddtheta)
D(1,1) = diff(diff(k_total, dtheta1), dtheta1);
D(1,2) = diff(diff(k_total, dtheta1), dtheta2);
D(1,3) = diff(diff(k_total, dtheta1), dtheta3);
D(2,1) = diff(diff(k_total, dtheta2), dtheta1);
D(2,2) = diff(diff(k_total, dtheta2), dtheta2);
D(2,3) = diff(diff(k_total, dtheta2), dtheta3);
D(3,1) = diff(diff(k_total, dtheta3), dtheta1);
D(3,2) = diff(diff(k_total, dtheta3), dtheta2);
D(3,3) = diff(diff(k_total, dtheta3), dtheta3);

% Simplify D matrix
D = simplify(D);

%% Step 8: Calculate individual D matrices for each link

% Calculate D1 matrix (inertia matrix for link 1)
D1 = sym(zeros(3,3));
D1(1,1) = diff(diff(k1_total, dtheta1), dtheta1);
D1(1,2) = diff(diff(k1_total, dtheta1), dtheta2);
D1(1,3) = diff(diff(k1_total, dtheta1), dtheta3);
D1(2,1) = diff(diff(k1_total, dtheta2), dtheta1);
D1(2,2) = diff(diff(k1_total, dtheta2), dtheta2);
D1(2,3) = diff(diff(k1_total, dtheta2), dtheta3);
D1(3,1) = diff(diff(k1_total, dtheta3), dtheta1);
D1(3,2) = diff(diff(k1_total, dtheta3), dtheta2);
D1(3,3) = diff(diff(k1_total, dtheta3), dtheta3);
D1 = simplify(D1);

% Calculate D2 matrix (inertia matrix for link 2)
D2 = sym(zeros(3,3));
D2(1,1) = diff(diff(k2_total, dtheta1), dtheta1);
D2(1,2) = diff(diff(k2_total, dtheta1), dtheta2);
D2(1,3) = diff(diff(k2_total, dtheta1), dtheta3);
D2(2,1) = diff(diff(k2_total, dtheta2), dtheta1);
D2(2,2) = diff(diff(k2_total, dtheta2), dtheta2);
D2(2,3) = diff(diff(k2_total, dtheta2), dtheta3);
D2(3,1) = diff(diff(k2_total, dtheta3), dtheta1);
D2(3,2) = diff(diff(k2_total, dtheta3), dtheta2);
D2(3,3) = diff(diff(k2_total, dtheta3), dtheta3);
D2 = simplify(D2);

% Calculate D3 matrix (inertia matrix for link 3)
D3 = sym(zeros(3,3));
D3(1,1) = diff(diff(k3_total, dtheta1), dtheta1);
D3(1,2) = diff(diff(k3_total, dtheta1), dtheta2);
D3(1,3) = diff(diff(k3_total, dtheta1), dtheta3);
D3(2,1) = diff(diff(k3_total, dtheta2), dtheta1);
D3(2,2) = diff(diff(k3_total, dtheta2), dtheta2);
D3(2,3) = diff(diff(k3_total, dtheta2), dtheta3);
D3(3,1) = diff(diff(k3_total, dtheta3), dtheta1);
D3(3,2) = diff(diff(k3_total, dtheta3), dtheta2);
D3(3,3) = diff(diff(k3_total, dtheta3), dtheta3);
D3 = simplify(D3);



% Display the D matrices
fprintf('\nDisplaying D1 matrix (inertia matrix for link 1):\n');
disp(D1);

fprintf('\nDisplaying D2 matrix (inertia matrix for link 2):\n');
disp(D2);

fprintf('\nDisplaying D3 matrix (inertia matrix for link 3):\n');
disp(D3);


%% Step 9: Calculate the h vector (Coriolis, centrifugal, and gravity terms) and torque outputs

h = sym(zeros(3,1));

% Calculate derivative of Lagrangian with respect to theta_i
dL_dtheta1 = diff(L, theta1);
dL_dtheta2 = diff(L, theta2);
dL_dtheta3 = diff(L, theta3);

% Calculate derivative of Lagrangian with respect to dtheta_i
dL_ddtheta1 = diff(L, dtheta1);
dL_ddtheta2 = diff(L, dtheta2);
dL_ddtheta3 = diff(L, dtheta3);

% Calculate the time derivative of dL_ddtheta_i
d_dt_dL_ddtheta1 = diff(dL_ddtheta1, theta1) * dtheta1 + ...
                   diff(dL_ddtheta1, theta2) * dtheta2 + ...
                   diff(dL_ddtheta1, theta3) * dtheta3 + ...
                   diff(dL_ddtheta1, dtheta1) * ddtheta1 + ...
                   diff(dL_ddtheta1, dtheta2) * ddtheta2 + ...
                   diff(dL_ddtheta1, dtheta3) * ddtheta3;

d_dt_dL_ddtheta2 = diff(dL_ddtheta2, theta1) * dtheta1 + ...
                   diff(dL_ddtheta2, theta2) * dtheta2 + ...
                   diff(dL_ddtheta2, theta3) * dtheta3 + ...
                   diff(dL_ddtheta2, dtheta1) * ddtheta1 + ...
                   diff(dL_ddtheta2, dtheta2) * ddtheta2 + ...
                   diff(dL_ddtheta2, dtheta3) * ddtheta3;

d_dt_dL_ddtheta3 = diff(dL_ddtheta3, theta1) * dtheta1 + ...
                   diff(dL_ddtheta3, theta2) * dtheta2 + ...
                   diff(dL_ddtheta3, theta3) * dtheta3 + ...
                   diff(dL_ddtheta3, dtheta1) * ddtheta1 + ...
                   diff(dL_ddtheta3, dtheta2) * ddtheta2 + ...
                   diff(dL_ddtheta3, dtheta3) * ddtheta3;

% Calculate the Euler-Lagrange equations (joint torques)
% T_i = d/dt(dL/d(dtheta_i)) - dL/dtheta_i
T1_expr = d_dt_dL_ddtheta1 - dL_dtheta1;
T2_expr = d_dt_dL_ddtheta2 - dL_dtheta2;
T3_expr = d_dt_dL_ddtheta3 - dL_dtheta3;

% Separate the terms with accelerations
T1 = collect(T1_expr, [ddtheta1, ddtheta2, ddtheta3]);
T2 = collect(T2_expr, [ddtheta1, ddtheta2, ddtheta3]);
T3 = collect(T3_expr, [ddtheta1, ddtheta2, ddtheta3]);

% Display the symbolic torque expressions
fprintf('\nDisplaying symbolic torque T1:\n');
disp(T1);

fprintf('\nDisplaying symbolic torque T2:\n');
disp(T2);

fprintf('\nDisplaying symbolic torque T3:\n');
disp(T3);

% Extract the h vector (all terms without accelerations)
h(1) = simplify(subs(T1, {ddtheta1, ddtheta2, ddtheta3}, {0, 0, 0}));
h(2) = simplify(subs(T2, {ddtheta1, ddtheta2, ddtheta3}, {0, 0, 0}));
h(3) = simplify(subs(T3, {ddtheta1, ddtheta2, ddtheta3}, {0, 0, 0}));

%% Step 10: Substitute numerical values for the D matrices

% Design dimensions of our project
a1_n = 0.12;  % link 1 length (m)
a2_n = 0.20;  % link 2 length (m)
a3_n = 0.30;  % link 3 length (m)
m1_n = 0.050; % mass of link 1 (kg)
m2_n = 0.050; % mass of link 2 (kg)
m3_n = 0.050; % mass of link 3 (kg)
g_n = 9.8;    % gravity (m/s^2)

% I = (m*L^2)/12
Ix1_n = (m1_n * a1_n^2) / 12;
Iy1_n = (m1_n * a1_n^2) / 12;
Iz1_n = (m1_n * a1_n^2) / 6;  % Assuming rotation around z-axis

Ix2_n = (m2_n * a2_n^2) / 12;
Iy2_n = (m2_n * a2_n^2) / 12;
Iz2_n = (m2_n * a2_n^2) / 6;

Ix3_n = (m3_n * a3_n^2) / 12;
Iy3_n = (m3_n * a3_n^2) / 12;
Iz3_n = (m3_n * a3_n^2) / 6;

theta1_n = pi/4;      
theta2_n = pi/3;      
theta3_n = pi/6;      


subs_params = {a1, a2, a3, m1, m2, m3, g, ...
             Ix1, Iy1, Iz1, Ix2, Iy2, Iz2, Ix3, Iy3, Iz3, ...
             theta1, theta2, theta3};
         
subs_params_vals = {a1_n, a2_n, a3_n, m1_n, m2_n, m3_n, g_n, ...
             Ix1_n, Iy1_n, Iz1_n, Ix2_n, Iy2_n, Iz2_n, Ix3_n, Iy3_n, Iz3_n, ...
             theta1_n, theta2_n, theta3_n};

% Substitute parameters into the D matrices
D1_params = subs(D1, subs_params, subs_params_vals);
D2_params = subs(D2, subs_params, subs_params_vals);
D3_params = subs(D3, subs_params, subs_params_vals);
D_params = subs(D, subs_params, subs_params_vals);

% Convert to numerical values for better readability
D1_num = double(D1_params);
D2_num = double(D2_params);
D3_num = double(D3_params);
D_num = double(D_params);

% Display the numerical D matrices
fprintf('\nNumerical D1 matrix:\n');
disp(D1_num);

fprintf('\nNumerical D2 matrix :\n');
disp(D2_num);

fprintf('\nNumerical D3 matrix:\n');
disp(D3_num);

%% Step 11: Display numerical values of transformation matrices and torques

% Substitute parameters into the transformation matrices
A1_params = subs(A1, subs_params, subs_params_vals);
A2_params = subs(A2, subs_params, subs_params_vals);
A3_params = subs(A3, subs_params, subs_params_vals);

% Substitute parameters into the torque expressions
% Assuming zero joint velocities and accelerations for numerical evaluation
dtheta1_n = 0;
dtheta2_n = 0;
dtheta3_n = 0;
ddtheta1_n = 0;
ddtheta2_n = 0;
ddtheta3_n = 0;

velocity_params = {dtheta1, dtheta2, dtheta3, ddtheta1, ddtheta2, ddtheta3};
velocity_vals = {dtheta1_n, dtheta2_n, dtheta3_n, ddtheta1_n, ddtheta2_n, ddtheta3_n};

% Substitute all parameters into torque expressions
T1_params = subs(T1, [subs_params, velocity_params], [subs_params_vals, velocity_vals]);
T2_params = subs(T2, [subs_params, velocity_params], [subs_params_vals, velocity_vals]);
T3_params = subs(T3, [subs_params, velocity_params], [subs_params_vals, velocity_vals]);

% Convert transformation matrices to numerical values for better readability
A1_num = double(A1_params);
A2_num = double(A2_params);
A3_num = double(A3_params);

% Convert torque expressions to numerical values
T1_num = double(T1_params);
T2_num = double(T2_params);
T3_num = double(T3_params);



% Display the numerical torque values at static equilibrium position
fprintf('\nNumerical torque T1 at equilibrium position :\n');
disp(T1_num);

fprintf('\nNumerical torque T2 at equilibrium position:\n');
disp(T2_num);

fprintf('\nNumerical torque T3 at equilibrium position:\n');
disp(T3_num);

% Now calculate with some velocity to demonstrate torque changes
dtheta1_n = 0.1;  % 0.1 rad/s
dtheta2_n = 0.2;  % 0.2 rad/s
dtheta3_n = 0.1;  % 0.1 rad/s
velocity_vals = {dtheta1_n, dtheta2_n, dtheta3_n, ddtheta1_n, ddtheta2_n, ddtheta3_n};

% Substitute all parameters into torque expressions with velocities
T1_vel_params = subs(T1, [subs_params, velocity_params], [subs_params_vals, velocity_vals]);
T2_vel_params = subs(T2, [subs_params, velocity_params], [subs_params_vals, velocity_vals]);
T3_vel_params = subs(T3, [subs_params, velocity_params], [subs_params_vals, velocity_vals]);

% Convert to numerical values
T1_vel_num = double(T1_vel_params);
T2_vel_num = double(T2_vel_params);
T3_vel_num = double(T3_vel_params);

% Display torques with joint velocities
fprintf('\nNumerical torque T1 with joint velocities :\n');
disp(T1_vel_num);

fprintf('\nNumerical torque T2 with joint velocities :\n');
disp(T2_vel_num);

fprintf('\nNumerical torque T3 with joint velocities:\n');
disp(T3_vel_num);