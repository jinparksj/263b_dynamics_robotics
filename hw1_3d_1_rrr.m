%hw1
%3D - 1 - RRR

clear all;
clc;

syms l1 l2 l3 theta1 theta2 theta3;


L3_mod(1) = Link([theta1, 0, 0, 0], 'modified');
L3_mod(2) = Link([theta2, 0, 0, -pi/2 ], 'modified');
L3_mod(3) = Link([theta3, 0, l2, 0], 'modified');
L3_mod(4) = Link([0, 0, l3, 0], 'modified');

RRR_3D_mod = SerialLink(L3_mod, 'name', '3D - 1 - RRR_mod')
L3_mod(1).A(theta1)
L3_mod(2).A(theta2)
L3_mod(3).A(theta3);
L3_mod(4).A(0);

 
RRR_3D_mod.fkine([theta1 theta2 theta3 0]);
% 
% L3_std(1) = Link([theta1, 0, 0, -pi/2], 'standard');
% L3_std(2) = Link([theta2, 0, l2, 0], 'standard');
% L3_std(3) = Link([theta3, 0, l3, 0], 'standard');
% 
% RRR_3D_std = SerialLink(L3_std, 'name', '3D - 1 - RRR_std')
% RRR_3D_std.fkine([theta1 theta2 theta3])
% 

% 
% L3_std(1) = Link([0, 0, 0, -pi/2], 'standard');
% L3_std(2) = Link([0, 0, 1, 0], 'standard');
% L3_std(3) = Link([0, 0, 1, 0], 'standard');
% 
% 
% RRR_3D_std = SerialLink(L3_std, 'name', '3D - 1 - RRR_std');
% 
% 
%  figure();
%  RRR_3D_std.plot([0 0 pi/2], 'workspace', [-3 3 -3 3 -3 3]);

 
