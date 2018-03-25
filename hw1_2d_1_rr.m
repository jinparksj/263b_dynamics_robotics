%hw1
%2D - 1 - RR

clear all;
clc;

syms l1 l2 theta1 theta2 theta3;


L1_mod(1) = Link([theta1, 0, 0, 0], 'modified');
L1_mod(2) = Link([theta2, 0, l1, 0], 'modified');
L1_mod(3) = Link([0, 0, l2, 0], 'modified');

RR_2D_mod = SerialLink(L1_mod, 'name', '2D - 1 - RR_mod');


H_T = L1_mod(2).A(theta2)



% RR_2D_mod.fkine([theta1 theta2 0]);
% 
% L1_std(1) = Link([theta1, 0, l1, 0], 'standard');
% L1_std(2) = Link([theta2, 0, l2, 0], 'standard');

% L1_std(1) = Link([0, 0, 1, 0], 'standard');
% L1_std(2) = Link([0, 0, 1, 0], 'standard');
% 
% RR_2D_std = SerialLink(L1_std, 'name', '2D - 1 - RR_std')
% sol = RR_2D_std.ikine_sym(2);
% length(sol)
% s1 = sol{1};
% q1 = s1(1);
% q2 = s1(2);
% 
% s2 = sol{2};
% q2_1 = s2(1)
% q2_2 = s2(2)


% 
% figure();
% RR_2D_std.plot([0 0], 'workspace', [-3 3 -3 3 -3 3]);

(l2*cos(theta1 + theta2) + l1*cos(theta1))^2 + (l2*sin(theta1 + theta2) + l1*sin(theta1))^2;
