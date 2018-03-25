%hw1
%3D - 5 - RRRP

clear all;
clc;

syms l1 l2 l3 theta1 theta2 theta3 d2 d3 d4 theta4 theta5 theta6;


L7_mod(1) = Link([theta1, 0, 0, 0], 'modified');
L7_mod(2) = Link([theta2, 0, l1, 0], 'modified');
L7_mod(3) = Link([theta3, 0, l2, 0], 'modified');
L7_mod(4) = Link([0, -d4, 0, 0, 1], 'modified');

RRRP_3D_mod = SerialLink(L7_mod, 'name', '3D - 5 - RRRP_mod');


L7_mod(1).A(theta1);
L7_mod(2).A(theta2);
L7_mod(3).A(theta3);
L7_mod(4).A(-d4);


L7_mod(1).A(theta1) * L7_mod(2).A(theta2) * L7_mod(3).A(theta3)



RRRP_3D_mod.fkine([theta1, theta2, theta3, -d4]);
% % 
% L7_std(1) = Link([theta1, 0, l1, 0], 'standard');
% L7_std(2) = Link([theta2, 0, l2, 0], 'standard');
% L7_std(3) = Link([theta3, 0, 0, 0], 'standard');
% L7_std(4) = Link([0, -d4, 0, 0, 1], 'standard');
% 
% RRRP_3D_std = SerialLink(L7_std, 'name', '3D - 5 - RRRP_std')
% RRRP_3D_std.fkine([theta1, theta2, theta3, -d4])

% L7_std(1) = Link([0, 0, 1, 0], 'standard');
% L7_std(2) = Link([0, 0, 1, 0], 'standard');
% L7_std(3) = Link([0, 0, 0, 0], 'standard');
% L7_std(4) = Link([0, -1, 0, 0, 1], 'standard');
% 
% RRRP_3D_std = SerialLink(L7_std, 'name', '3D - 5 - RRRP_std');
% figure();
% 
% RRRP_3D_std.plot([0 ,0 ,0 , 0], 'workspace', [-3, 3, -3, 3, -3, 3]);


