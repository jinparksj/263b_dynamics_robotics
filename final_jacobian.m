%final jacobian
% final
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
syms fx fy fz nx ny nz real;

%Differentiation
%time
syms t;

%Newton-Euler
syms f_3 n_3 dw_0_0 dw_1_1 dw_2_2 g dv_0_0 dv_1_1 dv_2_2;
syms dv_1_c1 dv_2_c2 F_1_1 F_2_2 F_3_3 N_1_1 N_2_2;

syms th1(i) th2(i) th3(i) th4(i) th5(i) th6(i)
syms dth1(i) dth2(i) dth3(i) dth4(i) dth5(i) dth6(i) 



%DH Parameter Table ( Modified )
dh = [0 0 0 th1(i); -pi/2 0 0 th2(i);0 a2 d3 th3(i); -pi/2 a3 d4 th4(i); pi/2 0 0 th5(i); -pi/2 0 0 th6(i); 0, -0.1, 0.13625, 0];
    

for j = 1:7
    T(:, :, j) = [cos(dh(j, 4)), -sin(dh(j, 4)), 0, dh(j, 2); sin(dh(j, 4)) * cos(dh(j, 1)), cos(dh(j, 4)) * cos(dh(j, 1)), -sin(dh(j, 1)), -sin(dh(j, 1)) * dh(j, 3); sin(dh(j, 4)) * sin(dh(j, 1)), cos(dh(j, 4)) * sin(dh(j, 1)), cos(dh(j, 1)), cos(dh(j, 1)) * dh(j, 3); 0, 0, 0, 1];    
end

T_0_1 = T(:, :, 1);
T_1_2 = T(:, :, 2);
T_2_3 = T(:, :, 3);
T_3_4 = T(:, :, 4);
T_4_5 = T(:, :, 5);
T_5_6 = T(:, :, 6);
T_6_T = T(:, :, 7);


T_0_2 = T_0_1 * T_1_2;
T_0_3 = T_0_1 * T_1_2 * T_2_3;
T_0_4 = T_0_1 * T_1_2 * T_2_3 * T_3_4;
T_0_5 = T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_5;
T_0_6 = T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_5 * T_5_6;
T_0_T = T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_5 * T_5_6 * T_6_T;

%Rotation Matrix
R_0_1 = T_0_1(1:3, 1:3);
R_1_2 = T_1_2(1:3, 1:3);
R_2_3 = T_2_3(1:3, 1:3);
R_3_4 = T_3_4(1:3, 1:3);
R_4_5 = T_4_5(1:3, 1:3);
R_5_6 = T_5_6(1:3, 1:3);
R_6_T = T_6_T(1:3, 1:3);

R_0_2 = T_0_2(1:3, 1:3);
R_0_3 = T_0_3(1:3, 1:3);
R_0_4 = T_0_4(1:3, 1:3);
R_0_5 = T_0_5(1:3, 1:3);
R_0_6 = T_0_6(1:3, 1:3);
R_0_T = T_0_T(1:3, 1:3);

R_4_0 = transpose(R_0_4);

R_T_6 = transpose(R_6_T);
R_6_5 = transpose(R_5_6);
R_5_4 = transpose(R_4_5);
R_4_3 = transpose(R_3_4);
R_3_2 = transpose(R_2_3);
R_2_1 = transpose(R_1_2);
R_1_0 = transpose(R_0_1);

%Position Vector
P_0_1 = T_0_1(1:3, 4);
P_1_2 = T_1_2(1:3, 4);
P_2_3 = T_2_3(1:3, 4);
P_3_4 = T_3_4(1:3, 4);
P_4_5 = T_4_5(1:3, 4);
P_5_6 = T_5_6(1:3, 4);
P_6_T = T_6_T(1:3, 4);

w_0_0 = [0; 0; 0];
w_1_1 = R_1_0 * w_0_0 + [0; 0; dth1(i)];
w_2_2 = R_2_1 * w_1_1 + [0; 0; dth2(i)];
w_3_3 = R_3_2 * w_2_2 + [0; 0; dth3(i)];
w_4_4 = R_4_3 * w_3_3 + [0; 0; dth4(i)];
w_5_5 = R_5_4 * w_4_4 + [0; 0; dth5(i)];
w_6_6 = R_6_5 * w_5_5 + [0; 0; dth6(i)];
w_T_T = R_T_6 * w_6_6 + [0; 0; 0];
simple_w_T_T = collect(simplify(w_T_T, 10), [dth1(i) dth2(i) dth3(i) dth4(i) dth5(i) dth6(i)])

v_0_0 = [0;0;0];
v_1_1 = R_1_0 * ( cross(w_0_0, P_0_1)  + v_0_0);
v_2_2 = R_2_1 * ( cross(w_1_1, P_1_2)  + v_1_1);
v_3_3 = R_3_2 * ( cross(w_2_2, P_2_3)  + v_2_2);
v_4_4 = R_4_3 * ( cross(w_3_3, P_3_4)  + v_3_3);
v_5_5 = R_5_4 * ( cross(w_4_4, P_4_5)  + v_4_4);
v_6_6 = R_6_5 * ( cross(w_5_5, P_5_6)  + v_5_5);
v_T_T = R_T_6 * ( cross(w_6_6, P_6_T)  + v_6_6);
simple_v_T_T = collect(simplify(v_T_T, 10), [dth1(i) dth2(i) dth3(i) dth4(i) dth5(i) dth6(i)])


f_4_4 = [fx; fy; fz];
f_3_3 = R_3_4 * f_4_4;
f_2_2 = R_2_3 * f_3_3;
f_1_1 = R_1_2 * f_2_2;
simple_f_1_1 = collect(simplify(f_1_1, 10), [fx fy fz]);


n_4_4 = [nx; ny; nz];
n_3_3 = R_3_4 * n_4_4 + cross(P_3_4, f_3_3);
n_2_2 = R_2_3 * n_3_3 + cross(P_2_3, f_2_2);
n_1_1 = R_1_2 * n_2_2 + cross(P_1_2, f_1_1);
simple_n_1_1 = collect(simplify(n_1_1, 10), [fx, fy, fz, nx, ny, nz]);

