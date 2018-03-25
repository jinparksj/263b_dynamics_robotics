%final problem 1
%Newton Euler
%Puma

clc;
clear all;

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
syms f_e n_e;
syms fx fy fz nx ny nz real;

%Newton-Euler

%angular acceleration
syms dw_0_0 dw_1_1 dw_2_2 dw_3_3 dw_4_4 dw_5_5 dw_6_6 real;

%linear acceleration
syms dv_0_0 dv_1_1 dv_2_2 dv_3_3 dv_4_4 dv_5_5 dv_6_6 real;

%linear acceleration wrt COM
syms dv_0_c0 dv_1_c1 dv_2_c2 dv_3_c3 dv_4_c4 dv_5_c5 dv_6_c6 real;

%Forward F and N

%etc

syms F_1_1 F_2_2 F_3_3 N_1_1 N_2_2;

a2 = 0.4318;
a3 = 0.0191;
d3 = 0.1254;
d4 = 0.4318;

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

% iterative newton-euler / CM at middle of the link

%angular velocities
w_0_0 = 0; 
w_1_1 = [0; 0; dt1];
w_2_2 = transpose(R_1_2) * w_1_1 + [0; 0; dt2];
w_3_3 = transpose(R_2_3) * w_2_2 + [0; 0; dt3];

%angular acceleration

dw_0_0 = 0;
dw_1_1 = [0; 0; ddt1];
dw_2_2 = transpose(R_1_2) * dw_1_1 + cross(transpose(R_1_2) * w_1_1, [0; 0; dt2]) + [0; 0; ddt2];
dw_3_3 = transpose(R_2_3) * dw_2_2 + cross(transpose(R_2_3) * w_2_2, [0; 0; dt3]) + [0; 0; ddt3];

%linear acceleration
dv_0_0 = [0; 0; grv];
dv_1_1 = transpose(R_0_1) * dv_0_0;
dv_2_2 = transpose(R_1_2) * ( cross(dw_1_1, P_1_2) + cross(w_1_1, cross(w_1_1, P_1_2)) + dv_1_1);
dv_3_3 = transpose(R_2_3) * ( cross(dw_2_2, P_2_3) + cross(w_2_2, cross(w_2_2, P_2_3)) + dv_2_2);

%linear acceleration wrt COM
dv_1_c1 = cross(dw_1_1, COM1) + cross(w_1_1, cross(w_1_1, COM1)) + dv_1_1;
dv_2_c2 = cross(dw_2_2, COM2) + cross(w_2_2, cross(w_2_2, COM2)) + dv_2_2;
dv_3_c3 = cross(dw_3_3, COM3) + cross(w_3_3, cross(w_3_3, COM3)) + dv_3_3;

%Forward F and N

%F, Force
F_1_1 = m1*dv_1_c1;
F_2_2 = m2*dv_2_c2;
F_3_3 = m3*dv_3_c3;

%N, Torque
N_1_1 = Ic1 * dw_1_1 + cross(w_1_1, Ic1 * w_1_1);
N_2_2 = Ic2 * dw_2_2 + cross(w_2_2, Ic2 * w_2_2);
N_3_3 = Ic3 * dw_3_3 + cross(w_3_3, Ic3 * w_3_3);


%inward iteration
%external forces
f_e = [fx; fy; fz];
n_e = [nx; ny; nz];

%forces
f_3_3 = R_3_4 * f_e + F_3_3;
f_2_2 = R_2_3 * f_3_3 + F_2_2;
f_1_1 = R_1_2 * f_2_2 + F_1_1;

%torques
n_3_3 = N_3_3 + R_3_4 * n_e + cross(COM3, F_3_3) + cross(P_3_4, R_3_4 * f_e);
n_2_2 = N_2_2 + R_2_3 * n_3_3 + cross(COM2, F_2_2) + cross(P_2_3, R_2_3 * f_3_3);
n_1_1 = N_1_1 + R_1_2*n_2_2 + cross(COM1, F_1_1) + cross(P_1_2, R_1_2*f_2_2);

%torques

Q_3 = collect(simplify(n_3_3(3), 10), [ddt1 ddt2 ddt3 dt1^2 dt2^2 dt3^2 grv fx fy fz nx ny nz]);
Q_2 = collect(simplify(n_2_2(3), 10), [ddt1 ddt2 ddt3 dt1^2 dt2^2 dt3^2 grv fx fy fz nx ny nz]);
Q_1 = collect(simplify(n_1_1(3), 10), [ddt1 ddt2 ddt3 dt1^2 dt2^2 dt3^2 grv fx fy fz nx ny nz]);

Q_n = [Q_1; Q_2; Q_3];

gravity_matrix_N = simplify([subs(Q_n, [ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3], [0, 0, 0, 0, 0, 0, 0, 0, 0])], 50);
mass_matrix_N = simplify([subs(Q_n, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {1, 0, 0, 0, 0, 0, 0, 0, 0}), subs(Q_n, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 1, 0, 0, 0, 0, 0, 0, 0}), subs(Q_n, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 1, 0, 0, 0, 0, 0, 0})] - [gravity_matrix_N gravity_matrix_N gravity_matrix_N], 50);
cori_centri_matrix_N = simplify([subs(Q_n, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 0, 0, 0, 0, 1, 0, 0}), subs(Q_n, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 0, 0, 0, 0, 0, 1, 0}), subs(Q_n, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 0, 0, 0, 0, 0, 0, 1})] - [gravity_matrix_N gravity_matrix_N gravity_matrix_N], 50);






