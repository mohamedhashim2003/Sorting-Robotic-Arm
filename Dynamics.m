% 3-DOF Robot Dynamics and Force Analysis
% Comprehensive Symbolic Computation of Equations of Motion and Forces

%% Initialization
clear all
clc
sympref('AbbreviateOutput', false);  % Disable output abbreviation

%% Symbolic Variables Declaration
% Geometric Parameters
syms l1 l2 l3 real        % Link lengths
syms d1 d2 d3 real        % Joint offsets

% Mass and Inertia Parameters
syms m1 m2 m3 real        % Link masses
syms I1xx I1yy I1zz real  % Link 1 moment of inertia
syms I2xx I2yy I2zz real  % Link 2 moment of inertia
syms I3xx I3yy I3zz real  % Link 3 moment of inertia

% Gravity and Dynamic Parameters
syms g real               % Gravity constant

% Joint Variables
syms theta1 theta2 theta3 real           % Joint angles
syms dtheta1 dtheta2 dtheta3 real        % Joint velocities
syms ddtheta1 ddtheta2 ddtheta3 real     % Joint accelerations

% Create vectors for easier manipulation
theta = [theta1; theta2; theta3];
dtheta = [dtheta1; dtheta2; dtheta3];
ddtheta = [ddtheta1; ddtheta2; ddtheta3];

%% Denavit-Hartenberg (DH) Parameters Table
% Format: [theta, d, a, alpha]
% Example configuration - adjust to match your specific robot
DH_params = [
    theta1,   d1,  l1,  0;    % Link 1
    theta2,   d2,  l2,  0;    % Link 2
    theta3,   d3,  l3,  0     % Link 3
];

%% Main Script Core

% Calculate transformations for each link
T01 = DH_transformation(theta1, d1, l1, 0);
T12 = DH_transformation(theta2, d2, l2, 0);
T23 = DH_transformation(theta3, d3, l3, 0);

% Compute cumulative transformations
T02 = T01 * T12;
T03 = T02 * T23;

% Extract positions for centers of mass (simplified for this example)
% Assuming center of mass at middle of each link
p0 = [0; 0; 0; 1];              % Origin
p1 = T01 * [l1/2; 0; 0; 1];     % COM of link 1
p2 = T02 * [l2/2; 0; 0; 1];     % COM of link 2
p3 = T03 * [l3/2; 0; 0; 1];     % COM of link 3

% Compute position Jacobians for velocity calculations
J1v = sym(zeros(3,3));
J2v = sym(zeros(3,3));
J3v = sym(zeros(3,3));

% For link 1 (only affected by joint 1)
J1v(:,1) = diff(p1(1:3), theta1);
% Other joints don't affect link 1's COM

% For link 2 (affected by joints 1 and 2)
J2v(:,1) = diff(p2(1:3), theta1);
J2v(:,2) = diff(p2(1:3), theta2);

% For link 3 (affected by all joints)
J3v(:,1) = diff(p3(1:3), theta1);
J3v(:,2) = diff(p3(1:3), theta2);
J3v(:,3) = diff(p3(1:3), theta3);

% Rotational Jacobians - simplified for planar case
J1w = [0; 0; 1; 0; 0; 0];
J2w = [0; 0; 1; 0; 0; 1];
J3w = [0; 0; 1; 0; 0; 1; 0; 0; 1];

% Compute kinetic energy
T = 0.5 * (m1 * (J1v(:,1)'*J1v(:,1)) * dtheta1^2 + ...
           m2 * (J2v*dtheta(1:2))'*(J2v*dtheta(1:2)) + ...
           m3 * (J3v*dtheta)'*(J3v*dtheta));

% Add rotational energy components
T = T + 0.5 * I1zz * dtheta1^2 + ...
        0.5 * I2zz * dtheta2^2 + ...
        0.5 * I3zz * dtheta3^2;

% Compute potential energy
V = m1 * g * p1(2) + m2 * g * p2(2) + m3 * g * p3(2);

% Compute Lagrangian
L = T - V;

%% Generalized Forces Computation
% Euler-Lagrange Equation: τ_i = d/dt(∂L/∂ḋ_i) - ∂L/∂q_i
% Compute torques for each joint
joint_torques = sym(zeros(3,1));

for i = 1:3
    % Get joint variable name
    q_var = ['theta' num2str(i)];
    dq_var = ['dtheta' num2str(i)];
    
    % Calculate partial derivatives
    dL_dqdot = diff(L, symvar(dq_var));
    
    % Compute time derivative of dL_dqdot
    dt_dL_dqdot = 0;
    for j = 1:3
        dt_dL_dqdot = dt_dL_dqdot + ...
            diff(dL_dqdot, theta(j)) * dtheta(j) + ...
            diff(dL_dqdot, dtheta(j)) * ddtheta(j);
    end
    
    % Compute partial derivative w.r.t. position
    dL_dq = diff(L, symvar(q_var));
    
    % Calculate torque using Euler-Lagrange equation
    joint_torques(i) = dt_dL_dqdot - dL_dq;
end

%% Mass Matrix (D) Computation
% Extract coefficients of acceleration terms
D_matrix = sym(zeros(3,3));
for i = 1:3
    for j = 1:3
        % Differentiate torque equation w.r.t. accelerations
        D_matrix(i,j) = diff(joint_torques(i), ddtheta(j));
    end
end

%% Coriolis and Centrifugal Forces (C)
% Compute h matrix (Coriolis and centrifugal terms)
h_vector = sym(zeros(3,1));
for i = 1:3
    % Symbolic computation of h vector elements
    h_expr = joint_torques(i);
    
    % Subtract D*ddtheta terms
    for j = 1:3
        h_expr = h_expr - D_matrix(i,j) * ddtheta(j);
    end
    h_vector(i) = h_expr;
end

% Compute Christoffel symbols and C matrix
C_matrix = sym(zeros(3,3));
for i = 1:3
    for j = 1:3
        % Calculate Coriolis matrix elements
        C_expr = 0;
        for k = 1:3
            % Use Christoffel symbols of the first kind
            C_expr = C_expr + ...
                0.5 * (diff(D_matrix(i,j), theta(k)) + ...
                       diff(D_matrix(i,k), theta(j)) - ...
                       diff(D_matrix(j,k), theta(i))) * dtheta(k);
        end
        C_matrix(i,j) = C_expr;
    end
end

%% Gravity Vector (G)
% Extract gravity terms from equations
G_vector = sym(zeros(3,1));
for i = 1:3
    G_vector(i) = diff(V, theta(i));
end

%% Complete Dynamic Equation
% The final form: τ = D(q)q̈ + C(q,q̇)q̇ + G(q)

%% Example Numeric Computation
% Define example parameter values
l1_val = 1.0;   % Link 1 length (m)
l2_val = 0.8;   % Link 2 length (m)
l3_val = 0.6;   % Link 3 length (m)

d1_val = 0.0;   % Link 1 offset (m)
d2_val = 0.0;   % Link 2 offset (m)
d3_val = 0.0;   % Link 3 offset (m)

m1_val = 5.0;   % Link 1 mass (kg)
m2_val = 4.0;   % Link 2 mass (kg)
m3_val = 3.0;   % Link 3 mass (kg)

% Moment of inertia values (kg⋅m²)
I1xx_val = 0.5; I1yy_val = 0.5; I1zz_val = 1.0;
I2xx_val = 0.4; I2yy_val = 0.4; I2zz_val = 0.8;
I3xx_val = 0.3; I3yy_val = 0.3; I3zz_val = 0.6;

g_val = 9.81;   % Gravity (m/s²)

% Example joint configuration
theta1_val = pi/4;    % 45 degrees
theta2_val = pi/3;    % 60 degrees
theta3_val = pi/6;    % 30 degrees

% Example joint velocities
dtheta1_val = 0.1;    % rad/s
dtheta2_val = 0.2;    % rad/s
dtheta3_val = 0.3;    % rad/s

% Example joint accelerations
ddtheta1_val = 0.01;  % rad/s²
ddtheta2_val = 0.02;  % rad/s²
ddtheta3_val = 0.03;  % rad/s²

% Create symbolic-to-numeric substitution list
sub_list = {l1, l2, l3, d1, d2, d3, ...
            m1, m2, m3, ...
            I1xx, I1yy, I1zz, I2xx, I2yy, I2zz, I3xx, I3yy, I3zz, ...
            g, ...
            theta1, theta2, theta3, ...
            dtheta1, dtheta2, dtheta3, ...
            ddtheta1, ddtheta2, ddtheta3};
val_list = {l1_val, l2_val, l3_val, d1_val, d2_val, d3_val, ...
            m1_val, m2_val, m3_val, ...
            I1xx_val, I1yy_val, I1zz_val, I2xx_val, I2yy_val, I2zz_val, I3xx_val, I3yy_val, I3zz_val, ...
            g_val, ...
            theta1_val, theta2_val, theta3_val, ...
            dtheta1_val, dtheta2_val, dtheta3_val, ...
            ddtheta1_val, ddtheta2_val, ddtheta3_val};

% Substitute numeric values into symbolic expressions
D_numeric = subs(D_matrix, sub_list, val_list);
C_numeric = subs(C_matrix, sub_list, val_list);
G_numeric = subs(G_vector, sub_list, val_list);

% Calculate joint torques for the example configuration
tau_numeric = subs(joint_torques, sub_list, val_list);

%% Display Results
disp('===== Robot Dynamics Analysis =====');
disp('1. Symbolic Joint Torques:');
disp(joint_torques);

disp('2. Symbolic Mass Matrix (D):');
disp(D_matrix);

disp('3. Symbolic Coriolis Matrix (C):');
disp(C_matrix);

disp('4. Symbolic Gravity Vector (G):');
disp(G_vector);

disp('5. Numeric Joint Torques for Example Configuration:');
disp(double(tau_numeric));

disp('6. Numeric Mass Matrix (D) for Example Configuration:');
disp(double(D_numeric));

disp('7. Numeric Coriolis Matrix (C) for Example Configuration:');
disp(double(C_numeric));

disp('8. Numeric Gravity Vector (G) for Example Configuration:');
disp(double(G_numeric));

%% Function Definitions
% All function definitions must be placed at the end of the script

function A = DH_transformation(theta, d, a, alpha)
    % Create Denavit-Hartenberg transformation matrix
    A = [
        cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
        0,           sin(alpha),             cos(alpha),            d;
        0,           0,                      0,                     1
    ];
end