% final
% PUMA


% clc;
% clear all;


% DH Parameters
syms alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 real;
syms a1 a2 a3 a4 a5 a6 real;
syms d1 d2 d3 d4 d5 d6 real;
syms theta1 theta2 theta3 theta4 theta5 theta6 real;

% Derivatives
syms dt1 dt2 dt3 dt4 dt5 dt6 real;
syms ddt1 ddt2 ddt3 ddt4 ddt5 ddt6 real;

% Inertia elements
syms I_1_xx I_1_yy I_1_zz I_1_xy I_1_yz I_1_xz I_2_xx I_2_yy I_2_zz I_2_xy I_2_yz I_2_xz I_3_xx I_3_yy I_3_zz I_3_xy I_3_yz I_3_xz real;

%moment of inertia
syms Ic1 Ic2 Ic3 Ic4 Ic5 Ic6 real;

%gravity
syms grv;

%mass
syms m1 m2 m3 m4 m5 m6 real;

%position vector
syms P_0_1 P_1_2 P_2_3 P_3_4 P_4_5 P_5_6 real;

%center of mass 
syms COM1 COM2 COM3 COM4 COM5 COM6 real;
syms lg1 lg2 lg3 lg4 lg5 lg6 real; %position of length of COM
syms pcx1 pcx2 pcx3 pcx4 pcx5 pcx6 pcy1 pcy2 pcy3 pcy4 pcy5 pcy6 pcz1 pcz2 pcz3 pcz4 pcz5 pcz6 real;

% center of mass wrt base
syms p_0_c1 p_0_c2 p_0_c3

% link length
syms l1 l2 l3 l4 l5 l6;

% velocities wrt base
syms v_0_c1 v_0_c2 v_0_c3 v_0_c4 v_0_c5 v_0_c6 real;

% angular velocity
syms w_0_0 w_1_1 w_2_2 w_3_3 w_4_4 w_5_5 w_6_6 

% Lagrangian parameters
syms k1 k2 k3 k4 k5 k6 ktot real; % kinetic energy
syms p1 p2 p3 p4 p5 p6 ptot real; % potential energy
syms tau1 tau2 tau3 tau4 tau5 tau6 real; %torque
syms Ltot1 Ltot2 Ltot3 Ltot4 Ltot5 Ltot6 real;

% external forces / torque for Q
syms fx fy fz nx ny nz real;

%Differentiation
%time
syms t;

%Newton-Euler
syms f_3 n_3 dw_0_0 dw_1_1 dw_2_2 g dv_0_0 dv_1_1 dv_2_2;
syms dv_1_c1 dv_2_c2 F_1_1 F_2_2 F_3_3 N_1_1 N_2_2;

a2 = 0.4318;
a3 = 0.0191;
d3 = 0.1254;
d4 = 0.4318;

m1 = 0;
m2 = 17.4;
m3 = 4.8;

pcx1 = 0;
pcy1 = 0;
pcz1 = 0;

pcx2 = 0.068;
pcy2 = 0.006;
pcz2 = -0.016;

pcx3 = 0;
pcy3 = -0.070;
pcz3 = 0.014;

I_1_xx = 0;
I_1_yy = 0;
I_1_zz = 0.35;

I_2_xx = 0.13;
I_2_yy = 0.524;
I_2_zz = 0.539;

I_3_xx = 0.066;
I_3_yy = 0.0125;
I_3_zz = 0.066;


%DH Parameter Table ( Modified )
dh = [0 0 0 theta1; -pi/2 0 0 theta2;0 a2 d3 theta3; -pi/2 a3 d4 0];
    

for i = 1:4
    T(:, :, i) = [cos(dh(i, 4)), -sin(dh(i, 4)), 0, dh(i, 2); sin(dh(i, 4)) * cos(dh(i, 1)), cos(dh(i, 4)) * cos(dh(i, 1)), -sin(dh(i, 1)), -sin(dh(i, 1)) * dh(i, 3); sin(dh(i, 4)) * sin(dh(i, 1)), cos(dh(i, 4)) * sin(dh(i, 1)), cos(dh(i, 1)), cos(dh(i, 1)) * dh(i, 3); 0, 0, 0, 1];    
end

T_0_1 = T(:, :, 1);
T_1_2 = T(:, :, 2);
T_2_3 = T(:, :, 3);
T_3_4 = T(:, :, 4);

T_0_2 = T_0_1 * T_1_2;
T_0_3 = T_0_1 * T_1_2 * T_2_3;
T_0_4 = T_0_1 * T_1_2 * T_2_3 * T_3_4;

%Rotation Matrix
R_0_1 = T_0_1(1:3, 1:3);
R_1_2 = T_1_2(1:3, 1:3);
R_2_3 = T_2_3(1:3, 1:3);
R_3_4 = T_3_4(1:3, 1:3);

R_0_2 = T_0_2(1:3, 1:3);
R_0_3 = T_0_3(1:3, 1:3);
R_0_4 = T_0_4(1:3, 1:3);

R_4_0 = transpose(R_0_4);

%Position Vector
P_0_1 = T_0_1(1:3, 4);
P_1_2 = T_1_2(1:3, 4);
P_2_3 = T_2_3(1:3, 4);
P_3_4 = T_3_4(1:3, 4);

%Center of Mass (Length)
COM1 = [pcx1; pcy1; pcz1];
COM2 = [pcx2; pcy2; pcz2];
COM3 = [pcx3; pcy3; pcz3];

%COM wrt Base
p_0_c1 = T_0_1 * [COM1; 1];
p_0_c2 = T_0_2 * [COM2; 1];
p_0_c3 = T_0_3 * [COM3; 1];

p_0_c1 = p_0_c1(1:3);
p_0_c2 = p_0_c2(1:3);
p_0_c3 = p_0_c3(1:3);

%Inertia Tensor
Ic1 = [I_1_xx, 0, 0; 0, I_1_yy, 0; 0, 0, I_1_zz];
Ic2 = [I_2_xx, 0, 0; 0, I_2_yy, 0; 0, 0, I_2_zz];
Ic3 = [I_3_xx, 0, 0; 0, I_3_yy, 0; 0, 0, I_3_zz];



% Lagrangian

% velocities wrt COM
% need to change
v_0_c1 = dt1 * diff(p_0_c1, theta1);
v_0_c2 = dt1 * diff(p_0_c2, theta1) + dt2 * diff(p_0_c2, theta2);
v_0_c3 = dt1 * diff(p_0_c3, theta1) + dt2 * diff(p_0_c3, theta2) + dt3 * diff(p_0_c3, theta3);

%angular velocity
w_1_1 = [0; 0; dt1];
w_2_2 = transpose(R_1_2) * w_1_1 + [0; 0; dt2];
w_3_3 = transpose(R_2_3) * w_2_2 + [0; 0; dt3];

%kinetic energy

%inertia tensor wrt Base
I_0_c1 = R_0_1 * Ic1 * transpose(R_0_1);
I_0_c2 = R_0_2 * Ic2 * transpose(R_0_2);
I_0_c3 = R_0_3 * Ic3 * transpose(R_0_3);

k1 = (1/2) * m1 * transpose(v_0_c1) * v_0_c1 + (1/2) * transpose(w_1_1) * Ic1 * w_1_1;
k2 = (1/2) * m2 * transpose(v_0_c2) * v_0_c2 + (1/2) * transpose(w_2_2) * Ic2 * w_2_2;
k3 = (1/2) * m3 * transpose(v_0_c3) * v_0_c3 + (1/2) * transpose(w_3_3) * Ic3 * w_3_3;
ktot = k1 + k2 + k3;

simple_ktot = collect(simplify(ktot, 10), [dt1^2 dt2^2 dt3^2]);

%potential energy
g = [0; 0; grv];

p1 = m1 * transpose(g) * p_0_c1;
p2 = m2 * transpose(g) * p_0_c2;
p3 = m3 * transpose(g) * p_0_c3;
ptot = p1 + p2 + p3;

%lagrangian
L = ktot - ptot;
simple_L = collect(simplify(L, 10), [dt1^2 dt2^2 dt3^2]);

%derivative for tau

%derivative Lagrangian

%derivative L with q
dL1 = diff(L, theta1);
dL2 = diff(L, theta2);
dL3 = diff(L, theta3);

%derivative L with dq
ddL1 = diff(L, dt1);
ddL2 = diff(L, dt2);
ddL3 = diff(L, dt3);
simple_ddL1 = collect(simplify(ddL1, 10), [dt1 dt2 dt3]);

%derivate ddL with time

%definition with char
syms ddL1_sub dddL1_theta1 dddL1_theta2 dddL1_theta3 dddL1_dt1 dddL1_dt2 dddL1_dt3 dddL1;

temp_theta1 = [char(theta1), '(t)'];
temp_dt1 = [char(dt1), '(t)'];
temp_theta2 = [char(theta2), '(t)'];
temp_dt2 = [char(dt2), '(t)'];
temp_theta3 = [char(theta3), '(t)'];
temp_dt3 = [char(dt3), '(t)'];

ddL1_temp = ddL1;
ddL1_sub = subs(ddL1_temp, {theta1, dt1, theta2, dt2, theta3, dt3}, {temp_theta1, temp_dt1, temp_theta2, temp_dt2, temp_theta3, temp_dt3});
dddL1 = diff(ddL1_sub, t);
dddL1_sub = subs(dddL1, {'diff(theta1(t), t)', 'diff(theta2(t), t)', 'diff(theta3(t), t)', 'diff(dt1(t), t)', 'diff(dt2(t), t)', 'diff(dt3(t), t)', 'theta1(t)', 'theta2(t)', 'theta3(t)', 'dt1(t)', 'dt2(t)', 'dt3(t)'}, {'dt1', 'dt2', 'dt3', 'ddt1', 'ddt2', 'ddt3', 'theta1', 'theta2', 'theta3', 'dt1', 'dt2', 'dt3'});

simple_dddL1 = collect(simplify(dddL1_sub, 10), [ddt1 ddt2 ddt3 dt1 dt2 dt3]);

ddL2_temp = ddL2;
ddL2_sub = subs(ddL2_temp, {theta1, dt1, theta2, dt2, theta3, dt3}, {temp_theta1, temp_dt1, temp_theta2, temp_dt2, temp_theta3, temp_dt3});
dddL2 = diff(ddL2_sub, t);
dddL2_sub = subs(dddL2, {'diff(theta1(t), t)', 'diff(theta2(t), t)', 'diff(theta3(t), t)', 'diff(dt1(t), t)', 'diff(dt2(t), t)', 'diff(dt3(t), t)', 'theta1(t)', 'theta2(t)', 'theta3(t)', 'dt1(t)', 'dt2(t)', 'dt3(t)'}, {'dt1', 'dt2', 'dt3', 'ddt1', 'ddt2', 'ddt3', 'theta1', 'theta2', 'theta3', 'dt1', 'dt2', 'dt3'});

simple_dddL2 = collect(simplify(dddL2_sub, 10), [ddt1 ddt2 ddt3 dt1 dt2 dt3]);

ddL3_temp = ddL3;
ddL3_sub = subs(ddL3_temp, {theta1, dt1, theta2, dt2, theta3, dt3}, {temp_theta1, temp_dt1, temp_theta2, temp_dt2, temp_theta3, temp_dt3});
dddL3 = diff(ddL3_sub, t);
dddL3_sub = subs(dddL3, {'diff(theta1(t), t)', 'diff(theta2(t), t)', 'diff(theta3(t), t)', 'diff(dt1(t), t)', 'diff(dt2(t), t)', 'diff(dt3(t), t)', 'theta1(t)', 'theta2(t)', 'theta3(t)', 'dt1(t)', 'dt2(t)', 'dt3(t)'}, {'dt1', 'dt2', 'dt3', 'ddt1', 'ddt2', 'ddt3', 'theta1', 'theta2', 'theta3', 'dt1', 'dt2', 'dt3'});

% summary
Q1_L = dddL1_sub - dL1;
Q2_L = dddL2_sub - dL2;
Q3_L = dddL3_sub - dL3;

simple_Q1_L = collect(simplify(Q1_L, 10), [ddt1 ddt2 ddt3 dt1^2 dt2^2 dt3^2 dt1*dt2 dt2*dt3 dt1*dt3]);
simple_Q2_L = collect(simplify(Q2_L, 10), [ddt1 ddt2 ddt3 dt1^2 dt2^2 dt3^2 dt1*dt2 dt2*dt3 dt1*dt3]);
simple_Q3_L = collect(simplify(Q3_L, 10), [ddt1 ddt2 ddt3 dt1^2 dt2^2 dt3^2 dt1*dt2 dt2*dt3 dt1*dt3]);

%torque of Lagrangian
tauL = [Q1_L; Q2_L; Q3_L];
simple_tauL = [simple_Q1_L; simple_Q2_L; simple_Q3_L];

gravity_matrix_L = simplify([subs(simple_tauL, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 0, 0, 0, 0, 0, 0, 0})], 50);
mass_matrix_L = simplify([subs(simple_tauL, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {1, 0, 0, 0, 0, 0, 0, 0, 0}), subs(simple_tauL, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 1, 0, 0, 0, 0, 0, 0, 0}), subs(simple_tauL, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 1, 0, 0, 0, 0, 0, 0})] - [gravity_matrix_L gravity_matrix_L gravity_matrix_L] , 50);
cori_centri_matrix_L = simplify([subs(simple_tauL, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 0, 0, 0, 0, 1, 0, 0}), subs(simple_tauL, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 0, 0, 0, 0, 0, 1, 0}), subs(simple_tauL, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 0, 0, 0, 0, 0, 0, 1})] - [gravity_matrix_L gravity_matrix_L gravity_matrix_L], 50);

%torque of external forces



jacobian4 = [-d3 * cos(theta2 + theta3), a2*sin(theta3) - d4, -d4;...
    d4 * sin(theta2 + theta3) - a3 * cos(theta2 + theta3) - a2 * cos(theta2), 0, 0;...
    d3 * sin(theta2 + theta3), a3 + a2*cos(theta3), a3;...
    -sin(theta2 + theta3), 0, 0;...
    0, -1, -1;...
    -cos(theta2 + theta3), 0, 0];

% jacobian4 = simplify([R_4_0, zeros(3); zeros(3), R_4_0] * jacobian, 10);
% jacobian_V = [cross(R_0_1(1:3, 3), (p_0_c1 - P_0_1)), cross(R_0_2(1:3, 3), (p_0_c2 - P_0_2)), cross(R_0_3(1:3, 3), (p_0_c3 - P_0_3))];
% jacobian_W = [R_0_1(1:3, 3), R_0_2(1:3, 3), R_0_3(1:3, 3)];
% jacobian = simplify([jacobian_V; jacobian_W], 10);

Q = tauL + transpose( simplify(jacobian4)) * [fx; fy; fz; nx; ny; nz];
Q = collect(simplify(Q, 10), [ddt1 ddt2 ddt3 dt1^2 dt2^2 dt3^2 grv fx fy fz nx ny nz]);

if Q_n == Q
    disp('Lagrangian is same with Newton-Euler')
end


gravity_matrix_L = simplify([subs(Q, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 0, 0, 0, 0, 0, 0, 0})], 50);
mass_matrix_L = simplify([subs(Q, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {1, 0, 0, 0, 0, 0, 0, 0, 0}), subs(Q, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 1, 0, 0, 0, 0, 0, 0, 0}), subs(Q, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 1, 0, 0, 0, 0, 0, 0})] - [gravity_matrix_L gravity_matrix_L gravity_matrix_L] , 50);
cori_centri_matrix_L = simplify([subs(Q, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 0, 0, 0, 0, 1, 0, 0}), subs(Q, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 0, 0, 0, 0, 0, 1, 0}), subs(Q, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 0, 0, 0, 0, 0, 0, 1})] - [gravity_matrix_L gravity_matrix_L gravity_matrix_L], 50);

if gravity_matrix_N == gravity_matrix_L
    disp('Gravity Matrix of Lagrangian is same with Newton-Euler')
end

if mass_matrix_N == mass_matrix_L
    disp('Mass Matrix of Lagrangian is same with Newton-Euler')
end

if cori_centri_matrix_N == cori_centri_matrix_L
    disp('Centrifugal and Coriolis Matrix of Lagrangian is same with Newton-Euler')
end

