%hw1
%3D - 2 - RRP

clear all;
clc;

syms l1 l2 l3 theta1 theta2 theta3 d3;


L4_mod(1) = Link([theta1, 0, 0, 0], 'modified');
L4_mod(2) = Link([theta2, l2, 0, pi/2 ], 'modified');
L4_mod(3) = Link([0, d3, 0, -pi/2, 1], 'modified');


RRP_3D_mod = SerialLink(L4_mod, 'name', '3D - 2 - RRP_mod');


T_0_1 = L4_mod(1).A(theta1);
T_1_2 = L4_mod(2).A(theta2);
T_2_3 = L4_mod(3).A(d3);

R = t2r(T_0_1) * t2r(T_1_2) * t2r(T_2_3);
J_4_4 = [l2*cos(theta2), -d3, 0; -d3 * sin(theta2), 0, 0 ; -l2 * sin(theta2), 1, 1; sin(theta2), 0, 0; 0, -1, 0; cos(theta2), 0, 0];
br = [R, zeros(3); zeros(3), R];
simplify(br * J_4_4)



RRP_3D_mod.fkine([theta1 theta2 d3]);

% L4_std(1) = Link([theta1, 0, 0, pi/2], 'standard');
% L4_std(2) = Link([theta2, l2, 0, -pi/2], 'standard');
% L4_std(3) = Link([0, d3, 0, 0, 1], 'standard');
% RRP_3D_std = SerialLink(L4_std, 'name', '3D - 2 - RRP_std')
% RRP_3D_std.fkine([theta1 theta2 d3])


L4_std(1) = Link([theta1, l2, 0, pi/2], 'standard');
L4_std(2) = Link([theta2, 0, 0, -pi/2], 'standard');
L4_std(3) = Link([0, d3, 0, 0, 1], 'standard');
RRP_3D_std = SerialLink(L4_std, 'name', '3D - 2 - RRP_std');
RRP_3D_std.fkine([theta1 theta2 d3]);


% L4_std(1) = Link([0, 0, 0, pi/2], 'standard');
% L4_std(2) = Link([0, 1, 0, -pi/2], 'standard');
% L4_std(3) = Link([0, 2, 0, 0, 1], 'standard');
% 
% 
% RRP_3D_std = SerialLink(L4_std, 'name', '3D - 2 - RRP_std');
% figure();
% 
% RRP_3D_std.plot([0 0 1], 'workspace', [-3, 3, -3, 3, -3, 3]);


