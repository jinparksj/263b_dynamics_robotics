%final3

%with final 2 trajectory generation
%interpolation at the joint space


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

cori_centri_matrix_N_2 = simplify([subs(Q_n, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 0, 1, 0, 0, 0, 0, 0}), subs(Q_n, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 0, 0, 1, 0, 0, 0, 0}), subs(Q_n, {ddt1, ddt2, ddt3, dt1^2, dt2^2, dt3^2, dt1*dt2, dt2*dt3, dt1*dt3}, {0, 0, 0, 0, 0, 1, 0, 0, 0})] - [gravity_matrix_N gravity_matrix_N gravity_matrix_N], 50);


syms alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 real;
syms a1 a2 a3 a4 a5 a6 real;
syms d1 d2 d3 d4 d5 d6 real;
syms theta1 theta2 theta3 theta4 theta5 theta6 real;

% Derivatives
syms dt1 dt2 dt3 dt4 dt5 dt6 real;
syms ddt1 ddt2 ddt3 ddt4 ddt5 ddt6 real;

a2 = 0.4318;
a3 = 0.0191;
d3 = 0.1254;
d4 = 0.4318;

theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;
theta5 = 0;
theta6 = 0;

%standard DH parameters

L_puma(1) = Link([theta1, 0, 0, -pi/2],'standard');
L_puma(2) = Link([theta2, 0, a2, 0],'standard');
L_puma(3) = Link([theta3, d3, a3, -pi/2],'standard');
L_puma(4) = Link([theta4, d4, 0, pi/2],'standard');
L_puma(5) = Link([theta5, 0, 0, -pi/2],'standard');
L_puma(6) = Link([theta6, 0, 0, 0],'standard');
tool = transl(-0.1, 0, 0.13625);

puma = SerialLink(L_puma, 'name', 'final puma', 'tool', tool);

angles = [theta1 theta2 theta3 theta4 theta5 theta6];

L = 0.8636;

s1 = sqrt(0.3^2 + 0.15^2) * L;
s2 = pi * 0.075 * L;
s3 = pi * 0.075 * L;
s4 = 0.15 * L;

totaldistance = s1 + s2 + s3 + s4;
totaltime = 20;
totalsteps = 2000;

averagevelocity = totaldistance/totaltime;

%initial conditions
drawingplain = 0; %0.2956
startingx = 0.4318;
startingy = 0.1254;
startingz = drawingplain; %0.2956

startpoint= [startingx; startingy; startingz];
nstep = 2000;

%viapoint (vp) 3 sections each segment
viasize_15 = (0.15 / 3) * L;
viasize_s1 = (s1 / 3);
viasize_30 = (0.3 / 3) * L;
viasize_s2 = s2/3;

ymove = [0; 1; 0];
ymoveneg = [0; -1; 0];
xmove = [1; 0; 0];
xmoveneg = [-1; 0; 0];

%s1
vp1 = startpoint + ymove * viasize_30 + xmove * viasize_15;
vp2 = vp1 + ymove * viasize_30 + xmove * viasize_15;
vp3 = vp2 + ymove * viasize_30 + xmove * viasize_15;

%s2
vp4 = vp3 + (0.075 * L * [cos(pi * 4 / 10); -(1-sin(pi * 4 / 10)); 0]);
vp5 = vp3 + (0.075 * L * [cos(pi * 3 / 10); -(1-sin(pi * 3 / 10)); 0]);
vp6 = vp3 + (0.075 * L * [cos(pi * 2 / 10); -(1-sin(pi * 2 / 10)); 0]);
vp7 = vp3 + (0.075 * L * [cos(pi * 1 / 10); -(1-sin(pi * 1 / 10)); 0]);
vp8 = vp3 + (0.075 * L * [1; -1; 0]);
vp9 = vp3 + (0.075 * L * [cos(pi * 1 / 10); -(1+sin(pi * 1 / 10)); 0]);
vp10 = vp3 + (0.075 * L * [cos(pi * 2 / 10); -(1+sin(pi * 2 / 10)); 0]);
vp11 = vp3 + (0.075 * L * [cos(pi * 3 / 10); -(1+sin(pi * 3 / 10)); 0]);
vp12 = vp3 + (0.075 * L * [cos(pi * 4 / 10); -(1+sin(pi * 4 / 10)); 0]);
vp13 = vp3 + (0.075 * L * [0; -2; 0]);

%s3
vp14 = vp13 + (0.075 * L * [cos(pi * 6 / 10); -(1-sin(pi * 4 / 10)); 0]);
vp15 = vp13 + (0.075 * L * [cos(pi * 7 / 10); -(1-sin(pi * 3 / 10)); 0]);
vp16 = vp13 + (0.075 * L * [cos(pi * 8 / 10); -(1-sin(pi * 2 / 10)); 0]);
vp17 = vp13 + (0.075 * L * [cos(pi * 9 / 10); -(1-sin(pi * 1 / 10)); 0]);
vp18 = vp13 + (0.075 * L * [-1; -1; 0]);
vp19 = vp13 + (0.075 * L * [cos(pi * 11 / 10); -(1+sin(pi * 1 / 10)); 0]);
vp20 = vp13 + (0.075 * L * [cos(pi * 12 / 10); -(1+sin(pi * 2 / 10)); 0]);
vp21 = vp13 + (0.075 * L * [cos(pi * 13 / 10); -(1+sin(pi * 3 / 10)); 0]);
vp22 = vp13 + (0.075 * L * [cos(pi * 14 / 10); -(1+sin(pi * 4 / 10)); 0]);
vp23 = vp13 + (0.075 * L * [0; -2; 0]);

%s4
vp24 = vp23 + viasize_15 * xmoveneg;
vp25 = vp24 + viasize_15 * xmoveneg;
vp26 = vp25 + viasize_15 * xmoveneg;

%remark point

remarkpoint = [startpoint vp1 vp2 vp3 vp5 vp7 vp9 vp11 vp13 vp15 vp17 vp19 vp21 vp23 vp24 vp25 vp26];
%startpoint vp1 vp2 vp3 vp4 vp5 vp6 vp7 vp8 vp9 vp10 vp11 vp12 vp13 vp14 vp15 vp16 vp17 vp18 vp19 vp20 vp21 vp22 vp23 vp24 vp25 vp26
remarkpoint_x = remarkpoint(1, 1:end);
remarkpoint_y = remarkpoint(2, 1:end);

number_remarkpoint = size(remarkpoint, 2);



%resizing viapoint size
stepsize = nstep * 5 / 20; %500
vs_15 = (0.15 * L) / stepsize;
vs_s1 = s1 / stepsize;
vs_30 = (0.3 * L) / stepsize;
vs_s2 = s2/stepsize;

count = 1;

%s1 - 1~501
via(:, 1) = startpoint;
for i = 1:1:stepsize
    count = count + 1;
    via(:, count) = via(:, (count - 1)) + ymove * vs_30 + xmove * vs_15;
end

half_stepsize = stepsize / 2;

%s2 - 501~1001
ccount = count;
for i=1:1:half_stepsize
    count = count + 1;
    via(:, count) = via(:, (ccount)) + (0.075 * L * [cos(pi*(half_stepsize - i) / (2*half_stepsize) ); -(1 - sin(pi*(half_stepsize - i) / (2 * half_stepsize))); 0]);
end

for i = 1: 1: half_stepsize
    count = count + 1;
    via(:, count) = via(:, (ccount)) + ( 0.075 * L * [cos(pi * i / (2 * half_stepsize) ); - (1 + sin(pi * i / (2 * half_stepsize))); 0]); 
end

%s3 - 1001 ~ 1501
cccount = count;
for i=1:1:half_stepsize
    count = count + 1;
    via(:, count) = via(:, (cccount)) + ( 0.075 * L * [cos(pi*(half_stepsize + i) / (2*half_stepsize) ); -(1 - sin(pi*(half_stepsize - i) / (2 * half_stepsize))); 0]);
end

for i = 1: 1: half_stepsize
    count = count + 1;
    via(:, count) = via(:, (cccount)) + ( 0.075 * L * [cos(pi * (((2 * half_stepsize) + i) / (2 * half_stepsize))); - (1 + sin(pi * i / (2 * half_stepsize))); 0]); 
end


%s4 - 1501 ~ 2001

for i = 1: 1: (stepsize + 1)
    count = count + 1;
    via(:, count) = via(:, (count - 1)) + xmoveneg * vs_15;
end

line_via_x = via(1, 1:(nstep+1));
line_via_y = via(2, 1:(nstep+1));

%plotting
figure(1);
plot(line_via_x, line_via_y); axis equal; hold on;
plot(remarkpoint_x', remarkpoint_y', 'r.'); axis equal; grid;
xlabel('x(m)');
ylabel('y(m)');
xlim([0.3 0.8]);
ylim([0.1 0.4]);
title('Cartesian Trajectory Map');

%joint inverse kinematics joint theta1, theta2, theta3 theta4 theta5 theta6
for i = 1:1:200
    joint(:, i) = wrapToPi(puma.ikine([0, 0, 1, (via(1,10*i));0, -1, 0, (via(2, 10*i));1, 0, 0, (via(3, 10*i)); 0, 0, 0, 1]));
end
joint(:, 201) = wrapToPi(puma.ikine([0, 0, 1, (via(1,2001));0, -1, 0, (via(2, i));1, 0, 0, (via(3, i)); 0, 0, 0, 1]));


%add 10% margin for finishing within 20s
ave_speed_s1 = (s1 / 5) * 1.1; %average speed during s1 for 5s
ave_speed_s2 = 1.1 * (s2 / 5); %average speed during s1 for 5s
ave_speed_s3 = 1.1* (s3 / 5); %average speed during s1 for 5s
ave_speed_s4 = 1.1* (s4 / 5); %average speed during s1 for 5s



%s1: viapoints 1 - 51

for i = 1:1:6
    q0_s1(i) = joint(i, 1);
    qf_s1(i) = joint(i, 51);
    tf_s1 = 5; %0 ~ 5 second
    t_s1 = [0: 0.1:tf_s1]; %100Hz 

    q_s1(i,:) = q0_s1(i) + 3/tf_s1^2 * (qf_s1(i) - q0_s1(i))*t_s1.^2 - 2/tf_s1^3 * (qf_s1(i) - q0_s1(i)) * t_s1.^3;
    dq_s1(i,:) = 6/tf_s1^2 * (qf_s1(i) - q0_s1(i)) * t_s1 - 6 / tf_s1^3 * (qf_s1(i) - q0_s1(i)) * t_s1.^2;
    ddq_s1(i,:) = 6/tf_s1^2 * (qf_s1(i) - q0_s1(i)) - 12/tf_s1^3 * (qf_s1(i) - q0_s1(i)) * t_s1;    
end



%s2 - s3: viapoints 51 - 151

for i = 1:1:100
    for j = 1:1:6
        initial_theta(j) = joint(j, i + 50);
        final_theta(j) = joint(j, i + 51);
        initial_velocity = 0;
        final_velocity = 0;

        tf_s23 = 0.2; 
        t_s23 = [0.1: 0.1:0.2]; %100Hz

        a_0 = joint(j, i + 50);
        a_1 = initial_velocity;
        a_2 = (3/tf_s23^2) * (final_theta(j) - initial_theta(j)) - (2/tf_s23) * initial_velocity - final_velocity;
        a_3 = (-2/tf_s23^3) * (final_theta(j) - initial_theta(j)) + (2/tf_s23^2) * (final_velocity - initial_velocity);
        
        q_s23(j,i,:) = a_0 + a_1 * t_s23 + a_2 * t_s23.^2 + a_3 * t_s23.^3;
        dq_s23(j,i,:) = a_1 + 2 * a_2 * t_s23 + 3 * a_3 * t_s23.^3;
        ddq_s23(j,i,:) = 2 * a_2 + 6 * a_3 * t_s23;
    end
end



%s4: viapoints 151 - 201

for i = 1:1:6
    q0_s4(i) = joint(i, 151);
    qf_s4(i) = joint(i, 201);
    tf_s4 = 5; %0 ~ 5 second
    t_s4 = [0: 0.1:tf_s4]; %100Hz 

    q_s4(i,:) = q0_s4(i) + 3/tf_s4^2 * (qf_s4(i) - q0_s4(i))*t_s4.^2 - 2/tf_s4^3 * (qf_s4(i) - q0_s4(i)) * t_s4.^3;
    dq_s4(i,:) = 6/tf_s4^2 * (qf_s4(i) - q0_s4(i)) * t_s4 - 6 / tf_s4^3 * (qf_s4(i) - q0_s4(i)) * t_s4.^2;
    ddq_s4(i,:) = 6/tf_s4^2 * (qf_s4(i) - q0_s4(i)) - 12/tf_s4^3 * (qf_s4(i) - q0_s4(i)) * t_s4;    
end



for i= 1:1:6
    total_q(i,:) = [q_s1(i,:), q_s23(i,:,1), q_s4(i,:)];
    total_dq(i,:) = [dq_s1(i,:), dq_s23(i,:,1), dq_s4(i,:)];
    total_ddq(i,:) = [ddq_s1(i,:),ddq_s23(i,:,1),ddq_s4(i,:)];
end

total_t = [0: 0.1: 20.1];

% syms th1(i) th2(i) th3(i);
% subs(cori_centri_matrix_N, {'theta1', 'theta2', 'theta3'}, {'th1(i)', 'th2(i)', 'th3(i)'})
% subs(cori_centri_matrix_N_2, {'theta1', 'theta2', 'theta3'}, {'th1(i)', 'th2(i)', 'th3(i)'})
% subs(gravity_matrix_N, {'fx', 'fy', 'fz', 'nx', 'ny', 'nz','theta1','theta2', 'theta3'}, {0, 0, 0, 0, 0, 0, 'th1(i)', 'th2(i)','th3(i)'})

for i = 1:1:size(total_q,2)
    th1(i) = total_q(1, i);
    th2(i) = total_q(2, i);
    th3(i) = total_q(3, i);
    ddtheta1(i) = total_ddq(1, i);
    ddtheta2(i) = total_ddq(2, i);    
    ddtheta3(i) = total_ddq(3, i);
    dth1(i) = total_dq(1, i);
    dth2(i) = total_dq(2, i);
    dth3(i) = total_dq(3, i);
end

% a) M

for i = 1:1:size(total_q,2)
    M_matrix11 = (45339*sin(th3(i)))/312500 - (4437*sin(2*th2(i)))/625000 + (45339*sin(2*th2(i) + th3(i)))/312500 - (42774911*sin(th2(i))^2)/31250000 + (3851*sin(th2(i) + th3(i))^2)/50000 + 12247833/6250000;
    M_matrix12 = (4218669*sin(th2(i)))/15625000 - (261*cos(th2(i)))/156250 - (14637*cos(th2(i) + th3(i)))/312500;
    M_matrix13 = -(14637*cos(th2(i) + th3(i)))/312500;
    M_matrix21 = (4218669*sin(th2(i)))/15625000 - (261*cos(th2(i)))/156250 - (14637*cos(th2(i) + th3(i)))/312500;
    M_matrix22 = (45339*sin(th3(i)))/156250 + 50142811/31250000;
    M_matrix23 = (45339*sin(th3(i)))/312500 + 1119/12500;
    M_matrix31 = -(14637*cos(th2(i) + th3(i)))/312500;
    M_matrix32 = (45339*sin(th3(i)))/312500 + 1119/12500;
    M_matrix33 = 1119/12500;
    M1(i) = M_matrix11 * ddtheta1(i) + M_matrix12 * ddtheta2(i) + M_matrix13 * ddtheta3(i);
    M2(i) = M_matrix21 * ddtheta1(i) + M_matrix22 * ddtheta2(i) + M_matrix23 * ddtheta3(i);
    M3(i) = M_matrix31 * ddtheta1(i) + M_matrix32 * ddtheta2(i) + M_matrix33 * ddtheta3(i);
end

% b) V

for i = 1:1:size(total_q,2)
    V_matrix11_mul = (3851*sin(2*th2(i) + 2*th3(i)))/50000 - (4437*cos(2*th2(i)))/312500 - (42774911*sin(2*th2(i)))/31250000 + (45339*cos(2*th2(i) + th3(i)))/156250;
    V_matrix12_mul = (14637*sin(th2(i) + th3(i)))/156250;
    V_matrix13_mul = (45339*cos(th3(i)))/312500 + (3851*sin(2*th2(i) + 2*th3(i)))/50000 + (45339*cos(2*th2(i) + th3(i)))/312500;
    V_matrix21_mul = 0;
    V_matrix22_mul = (45339*cos(th3(i)))/156250;
    V_matrix23_mul = 0;
    V_matrix31_mul = 0;
    V_matrix32_mul = 0;
    V_matrix33_mul = 0;
    
    V_matrix11_sq = 0;
    V_matrix12_sq = (4218669*cos(th2(i)))/15625000 + (261*sin(th2(i)))/156250 + (14637*sin(th2(i) + th3(i)))/312500;
    V_matrix13_sq = (14637*sin(th2(i) + th3(i)))/312500;
    V_matrix21_sq = (4437*cos(2*th2(i)))/625000 - (3851*sin(2*th2(i) + 2*th3(i)))/100000 + (42774911*sin(2*th2(i)))/62500000 - (45339*cos(2*th2(i) + th3(i)))/312500;
    V_matrix22_sq = 0;
    V_matrix23_sq = (45339*cos(th3(i)))/312500;
    V_matrix31_sq = - (45339*cos(th3(i)))/625000 - (3851*sin(2*th2(i) + 2*th3(i)))/100000 - (45339*cos(2*th2(i) + th3(i)))/625000;
    V_matrix32_sq =  -(45339*cos(th3(i)))/312500;
    V_matrix33_sq = 0;
    
    V1_mul(i) = V_matrix11_mul * dth1(i) * dth2(i) + V_matrix12_mul * dth2(i) * dth3(i) + V_matrix13_mul * dth1(i) * dth3(i);
    V2_mul(i) = V_matrix21_mul * dth1(i) * dth2(i) + V_matrix22_mul * dth2(i) * dth3(i) + V_matrix23_mul * dth1(i) * dth3(i);
    V3_mul(i) = V_matrix31_mul * dth1(i) * dth2(i) + V_matrix32_mul * dth2(i) * dth3(i) + V_matrix33_mul * dth1(i) * dth3(i);
    
    V1_sq(i) = V_matrix11_sq * dth1(i)^2 + V_matrix12_sq * dth2(i)^2 + V_matrix13_sq * dth1(i)^2;
    V2_sq(i) = V_matrix21_sq * dth1(i)^2 + V_matrix22_sq * dth2(i)^2 + V_matrix23_sq * dth1(i)^2;
    V3_sq(i) = V_matrix31_sq * dth1(i)^2 + V_matrix32_sq * dth2(i)^2 + V_matrix33_sq * dth1(i)^2;
    
    V1(i) = V1_mul(i) + V1_sq(i);
    V2(i) = V2_mul(i) + V2_sq(i);
    V3(i) = V3_mul(i) + V3_sq(i);

end

% c) G

grv = 9.8

for i = 1:1:size(total_q,2)
    
    
    G1(i) = 0;
    G2(i) = -grv*((20349*cos(th2(i)))/6250 - (261*sin(th2(i)))/2500 + (42*sin(th2(i) + th3(i)))/125);
    G3(i) = -(42*grv*sin(th2(i) + th3(i)))/125;
    
end


% d) tau

for i = 1:1:size(total_q,2)
    
    tau_final1(i) = M1(i) + V1(i) + G1(i);
    tau_final2(i) = M2(i) + V2(i) + G2(i);
    tau_final3(i) = M3(i) + V3(i) + G3(i);
    
end


%a) M plot

figure(1);
%plotting
subplot(311);
plot(total_t, M1);
title('a) M(theta)*ddtheta');
ylabel('Mass 1');


subplot(312);
plot(total_t, M2);
ylabel('Mass 2');

subplot(313)
plot(total_t, M3);
ylabel('Mass 3');
xlabel('time (t)');


% b) V plot


figure(2);
%plotting
subplot(311);
plot(total_t, V1);
title('b) V(theta)');
ylabel('V 1');


subplot(312);
plot(total_t, V2);
ylabel('V 2');

subplot(313)
plot(total_t, V3);
ylabel('V 3');
xlabel('time (t)');

% c) G plot


figure(3);
%plotting
subplot(311);
plot(total_t, G1);
title('c) G(theta)');
ylabel('G 1');


subplot(312);
plot(total_t, G2);
ylabel('G 2');

subplot(313)
plot(total_t, G3);
ylabel('G 3');
xlabel('time (t)');


% d) Tau plot


figure(4);
%plotting
subplot(311);
plot(total_t, tau_final1);
title('d) Tau');
ylabel('tau 1');


subplot(312);
plot(total_t, tau_final2);
ylabel('tau 2');

subplot(313)
plot(total_t, tau_final2);
ylabel('tau 3');
xlabel('time (t)');











