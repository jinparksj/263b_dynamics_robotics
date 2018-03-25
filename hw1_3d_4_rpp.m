%hw1
%3D - 4 - RPP

clear all;
clc;

syms l1 l2 l3 theta1 theta2 theta3 d2 d3;


L5_mod(1) = Link([theta1, 0, 0, 0], 'modified');
L5_mod(2) = Link([pi/2, d2, 0, 0, 1 ], 'modified');
L5_mod(3) = Link([0, d3, 0, pi/2, 1], 'modified');

RPP_3D_mod = SerialLink(L5_mod, 'name', '3D - 4 - RPP_mod')

L5_mod(1).A(theta1) * L5_mod(2).A(d2) * L5_mod(3).A(d3)


RPP_3D_mod.fkine([theta1 d2 d3]);

L5_std(1) = Link([theta1, 0, 0, 0], 'standard');
L5_std(2) = Link([pi/2, d2, 0, pi/2, 1], 'standard');
L5_std(3) = Link([0, d3, 0, 0, 1], 'standard');

RPP_3D_std = SerialLink(L5_std, 'name', '3D - 4 - RPP_std')
RPP_3D_std.fkine([theta1 d2 d3]);

% 
% 
% L5_std(1) = Link([0, 0, 0, 0], 'standard');
% L5_std(2) = Link([pi/2, 1, 0, pi/2, 1], 'standard');
% L5_std(3) = Link([0, 1, 0, 0, 1], 'standard');
% 
% 
% RPP_3D_std = SerialLink(L5_std, 'name', '3D - 4 - RPP_std');
% figure();
% 
% RPP_3D_std.plot([0 1 1], 'workspace', [-3, 3, -3, 3, -3, 3]);
% 




