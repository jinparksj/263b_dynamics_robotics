%hw1
%2D - 2 - RRR

clear all;
clc;

syms l1 l2 theta1 theta2 theta3;


L2_mod(1) = Link([theta1, 0, 0, 0], 'modified');
L2_mod(2) = Link([theta2, 0, l1, 0], 'modified');
L2_mod(3) = Link([theta3, 0, l2, 0], 'modified');

RRR_2D_mod = SerialLink(L2_mod, 'name', '2D - 2 - RRR_mod')
L2_mod(1).A(theta1)
L2_mod(2).A(theta2)
L2_mod(3).A(theta3)

RRR_2D_mod.fkine([theta1 theta2 theta3]);
% 
L2_std(1) = Link([theta1, 0, l1, 0], 'standard');
L2_std(2) = Link([theta2, 0, l2, 0], 'standard');
L2_std(3) = Link([theta3, 0, 0, 0], 'standard');

% L2_std(1) = Link([0, 0, 1, 0], 'standard');
% L2_std(2) = Link([0, 0, 1, 0], 'standard');
% L2_std(3) = Link([0, 0, 0, 0], 'standard');


RRR_2D_std = SerialLink(L2_std, 'name', '2D - 2 - RRR_std')
RRR_2D_std.fkine([theta1 theta2 theta3])
% 
% 
% figure();
% RRR_2D_std.plot([0 0 0], 'workspace', [-3 3 -3 3 -3 3]);


