clear; clc;
syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 real
syms g real
% Numerical values
L1 = 0.12; % Link 1 length in meters
L2 = 0.20; % Link 2 length in meters
L3 = 0.30; % Base diameter
m1 = 0.05; % Link 1 mass in kg
m2 = 0.05; % Link 2 mass in kg
m3 = 0.05; % Base mass in kg
I1 = (1/12)*m1*L1^2; % Moment of inertia of Link 1
I2 = (1/12)*m2*L2^2; % Moment of inertia of Link 2
I3 = (1/12)*m3*L3^2; % Moment of inertia of base
% Joint variables
q = [q1; q2; q3];
dq = [dq1; dq2; dq3];
ddq = [ddq1; ddq2; ddq3];
% Center of mass positions
r1 = [L1/2 * cos(q1); L1/2 * sin(q1)];
r2 = [L1 * cos(q1) + L2/2 * cos(q1 + q2);
 L1 * sin(q1) + L2/2 * sin(q1 + q2)];
r3 = [L1 * cos(q1) + L2 * cos(q1 + q2) + L3/2 * cos(q1 + q2 + q3);
 L1 * sin(q1) + L2 * sin(q1 + q2) + L3/2 * sin(q1 + q2 + q3)];
% Jacobians
J1 = jacobian(r1, q);
J2 = jacobian(r2, q);
J3 = jacobian(r3, q);
v1 = J1 * dq;
v2 = J2 * dq;
v3 = J3 * dq;
% Kinetic Energy
K1 = 0.5 * m1 * (v1.' * v1) + 0.5 * I1 * dq1^2;
K2 = 0.5 * m2 * (v2.' * v2) + 0.5 * I2 * (dq1 + dq2)^2;
K3 = 0.5 * m3 * (v3.' * v3) + 0.5 * I3 * (dq1 + dq2 + dq3)^2;
K = simplify(K1 + K2 + K3);
% Potential Energy
P1 = m1 * g * r1(2);
P2 = m2 * g * r2(2);
P3 = m3 * g * r3(2);
P = simplify(P1 + P2 + P3);
L = K - P;
% Euler-Lagrange Equations
E = sym(zeros(3,1));
for i = 1:3
 dL_dqi = diff(L, q(i));
 dL_ddqi = diff(L, dq(i));
 d_dt_dL_ddqi = diff(dL_ddqi, q1)*dq1 + diff(dL_ddqi, q2)*dq2 + diff(dL_ddqi, q3)*dq3 + ...
 diff(dL_ddqi, dq1)*ddq1 + diff(dL_ddqi, dq2)*ddq2 + diff(dL_ddqi, dq3)*ddq3;
 E(i) = simplify(d_dt_dL_ddqi - dL_dqi);
end
g_val = 9.81;
T1 = simplify(subs(E(1), g, g_val));
T2 = simplify(subs(E(2), g, g_val));
T3 = simplify(subs(E(3), g, g_val));
q1_val = pi/4; % 45 degrees
q2_val = pi/6; % 30 degrees
q3_val = pi/12; % 15 degrees
dq1_val = 0.1; % Joint 1 velocity (rad/s)
dq2_val = 0.2; % Joint 2 velocity (rad/s)
dq3_val = 0.05; % Joint 3 velocity (rad/s)
ddq1_val = 0.01; % Joint 1 acceleration (rad/s²)
ddq2_val = 0.02; % Joint 2 acceleration (rad/s²)
ddq3_val = 0.005;% Joint 3 acceleration (rad/s²)
% Substitute into the torque expressions
T1_num = double(subs(T1, {q1, q2, q3, dq1, dq2, dq3, ddq1, ddq2, ddq3}, ...
 {q1_val, q2_val, q3_val, dq1_val, dq2_val, dq3_val, ddq1_val, ddq2_val, ddq3_val}));
T2_num = double(subs(T2, {q1, q2, q3, dq1, dq2, dq3, ddq1, ddq2, ddq3}, ...
 {q1_val, q2_val, q3_val, dq1_val, dq2_val, dq3_val, ddq1_val, ddq2_val, ddq3_val}));
T3_num = double(subs(T3, {q1, q2, q3, dq1, dq2, dq3, ddq1, ddq2, ddq3}, ...
 {q1_val, q2_val, q3_val, dq1_val, dq2_val, dq3_val, ddq1_val, ddq2_val, ddq3_val}));
% Display numerical torques
disp('T1 (Joint 1 torque in Nm):');
disp(T1_num);
disp(' T2 (Joint 2 torque in Nm):');
disp(T2_num);
disp(' T3 (Joint 3 torque in Nm):');
disp(T3_num);