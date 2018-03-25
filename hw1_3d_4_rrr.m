%hw1
%3D - 4 - RRR

clear all;
clc;

syms l1 l2 l3 theta1 theta2 theta3 d2 d3 theta4 theta5 theta6;


L6_mod(1) = Link([theta4, 0, 0, 0], 'modified');
L6_mod(2) = Link([theta5, 0, 0, pi/2], 'modified');
L6_mod(3) = Link([theta6, 0, 0, -pi/2], 'modified');

RRR_3D_mod = SerialLink(L6_mod, 'name', '3D - 4 - RRR_mod');

L6_mod(1).A(theta4);
L6_mod(2).A(theta5);
L6_mod(3).A(theta6);

L6_mod(1).A(theta4) * L6_mod(2).A(theta5) * L6_mod(3).A(theta6)

RRR_3D_mod.fkine([theta4 theta5 theta6]);

L6_std(1) = Link([theta4, 0, 0, pi/2], 'standard');
L6_std(2) = Link([theta5, 0, 0, -pi/2], 'standard');
L6_std(3) = Link([theta6, 0, 0, 0], 'standard');

RRR_3D_std = SerialLink(L6_std, 'name', '3D - 4 - RRR_std');
RRR_3D_std.fkine([theta4 theta5 theta6]);

% 
% 
% L6_std(1) = Link([0, 0, 0, pi/2], 'standard');
% L6_std(2) = Link([0, 0, 0, -pi/2], 'standard');
% L6_std(3) = Link([0, 0, 0, 0], 'standard');
% 
% 
% RRR_3D_std = SerialLink(L6_std, 'name', '3D - 4 - RRR_std');
% figure();
% 
% RRR_3D_std.plot([0 pi/2 0], 'workspace', [-1, 1, -1, 1, -1, 1]);


