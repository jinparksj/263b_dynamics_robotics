%hw1
%3D - 6 - PRRR

clear all;
clc;

syms l1 l2 l3 l4 theta1 theta2 theta3 d1 d2 d3 d4 theta4 theta5 theta6;


L8_mod(1) = Link([0, d1, 0, 0, 1], 'modified');
L8_mod(2) = Link([theta2, l1, 0, 0], 'modified');
L8_mod(3) = Link([theta3, 0, l2, 0], 'modified');
L8_mod(4) = Link([theta4, 0, l3, 0], 'modified');
L8_mod(5) = Link([0, -l4, 0, 0], 'modified');

PRRR_3D_mod = SerialLink(L8_mod, 'name', '3D - 6 - PRRR_mod');

PRRR_3D_mod.fkine([d1, theta2, theta3, theta4, 0])


L8_mod(1).A(d1);
L8_mod(2).A(theta2);
L8_mod(3).A(theta3);
L8_mod(4).A(theta4);
L8_mod(5).A(0)
% 
% L8_std(1) = Link([0, d1+l1, 0, 0, 1], 'standard');
% L8_std(2) = Link([theta2, 0, l2, 0], 'standard');
% L8_std(3) = Link([theta3, 0, l3, 0], 'standard');
% L8_std(4) = Link([theta4, -l4, 0, 0], 'standard');
% % 
% PRRR_3D_std = SerialLink(L8_std, 'name', '3D - 6 - PRRR_std')
% PRRR_3D_std.fkine([d1 + l1, theta2, theta3, theta4])

% L8_std(1) = Link([0, 0, 0, 0, 1, 1], 'standard');
% L8_std(2) = Link([0, 0, 1, 0], 'standard');
% L8_std(3) = Link([0, 0, 1, 0], 'standard');
% L8_std(4) = Link([0, -1, 0, 0], 'standard');
% 
% PRRR_3D_std = SerialLink(L8_std, 'name', '3D - 6 - PRRR_std')
% figure();
% 
% PRRR_3D_std.plot([0 ,0 ,0 ,0], 'workspace', [-3, 3, -3, 3, -3, 3]);


