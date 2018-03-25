%midterm - kuka KR 6 Agilus R900 Sixx

clear all;
clc;

syms theta1 theta2 theta3 theta4 theta5 theta6 X Y Z dt1 dt2 dt3 dt4 dt5 dt6 fx fy fz nx ny nz;


%Forward Kinematics

a1 = 25;
a2 = 455;
a3 = 35;
d1 = 400;
d4 = 420;

kuka_mod(1) = Link([theta1, 400, 0, 0], 'modified');
kuka_mod(2) = Link([theta2, 0, 25, (-pi/2)], 'modified');
kuka_mod(3) = Link([theta3, 0, 455, 0], 'modified');
kuka_mod(4) = Link([theta4, 420, 35, (-pi/2)], 'modified');
kuka_mod(5) = Link([theta5, 0, 0, pi/2], 'modified');
kuka_mod(6) = Link([theta6, 0, 0, (-pi/2)], 'modified');

kuka_r900_mod = SerialLink(kuka_mod, 'name', 'Kuka KR 6 R900_mod');

kuka_std(1) = Link([theta1, 400, 25, (-pi/2)], 'standard');
kuka_std(2) = Link([theta2, 0,  455, 0], 'standard');
kuka_std(3) = Link([theta3, 0, 35, (-pi/2)], 'standard');
kuka_std(4) = Link([theta4, 420, 0, pi/2], 'standard');
kuka_std(5) = Link([theta5, 0, 0, (-pi/2)], 'standard');
kuka_std(6) = Link([theta6, 0, 0, 0], 'standard');


kuka_r900_std = SerialLink(kuka_std, 'name', 'Kuka KR 6 R900_std');

kuka_r900_mod.fkine([theta1 theta2 theta3 theta4 theta5 theta6]);
kuka_r900_std.fkine([theta1 theta2 theta3 theta4 theta5 theta6]);

T_0_1 = kuka_mod(1).A(theta1);
T_1_2 = kuka_mod(2).A(theta2);
T_2_3 = kuka_mod(3).A(theta3);
T_3_4 = kuka_mod(4).A(theta4);
T_4_5 = kuka_mod(5).A(theta5);
T_5_6 = kuka_mod(6).A(theta6);

T_0_1_std = kuka_std(1).A(theta1);
T_1_2_std = kuka_std(2).A(theta2);
T_2_3_std = kuka_std(3).A(theta3);
T_3_4_std = kuka_std(4).A(theta4);
T_4_5_std = kuka_std(5).A(theta5);
T_5_6_std = kuka_std(6).A(theta6);

HT_0_6 = T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_5 * T_5_6;

%plot

kuka_std_plot(1) = Link([0, 400, 25, (-pi/2)], 'standard');
kuka_std_plot(2) = Link([0, 0,  455, 0], 'standard');
kuka_std_plot(3) = Link([0, 0, 35, (-pi/2)], 'standard');
kuka_std_plot(4) = Link([0, 420, 0, pi/2], 'standard');
kuka_std_plot(5) = Link([0, 0, 0, (-pi/2)], 'standard');
kuka_std_plot(6) = Link([0, 0, 0, 0], 'standard');

figure(1);
kuka_r900_std_plot = SerialLink(kuka_std_plot, 'name', 'Kuka KR 6 R900_std_plot');
kuka_r900_std_plot.plot([0, 0, 0, 0, pi/2, 0],'workspace', [-2000,2000, -2000,2000, -2000,2000]);


kuka_mod_plot(1) = Link([0, 400, 0, 0], 'modified');
kuka_mod_plot(2) = Link([0, 0, 25, (-pi/2)], 'modified');
kuka_mod_plot(3) = Link([0, 0, 455, 0], 'modified');
kuka_mod_plot(4) = Link([0, 420, 35, (-pi/2)], 'modified');
kuka_mod_plot(5) = Link([0, 0, 0, pi/2], 'modified');
kuka_mod_plot(6) = Link([0, 0, 0, (-pi/2)], 'modified');

figure(2);
kuka_r900_mod_plot = SerialLink(kuka_mod_plot, 'name', 'Kuka KR 6 R900_mod');
kuka_r900_mod_plot.plot([0, 0, 0, 0, pi/2, 0],'workspace', [-2000,2000, -2000,2000, -2000,2000]);


% %inverse kinematics

sol = kuka_r900_std.ikine_sym(6)
length(sol);

s1 = sol{1}
s2 = sol{2}
s3 = sol{3}
s4 = sol{4}
s5 = sol{5}
s6 = sol{6}

%Jacobian(Explicit Method)

X = 25*cos(theta1) + 455*cos(theta1)*cos(theta2) - 35*cos(theta1)*sin(theta2)*sin(theta3) + 35*cos(theta1)*cos(theta2)*cos(theta3) - 420*cos(theta1)*cos(theta2)*sin(theta3) - 420*cos(theta1)*cos(theta3)*sin(theta2);
Y = 25*sin(theta1) + 455*cos(theta2)*sin(theta1) - 420*cos(theta2)*sin(theta1)*sin(theta3) - 420*cos(theta3)*sin(theta1)*sin(theta2) - 35*sin(theta1)*sin(theta2)*sin(theta3) + 35*cos(theta2)*cos(theta3)*sin(theta1);
Z = 420*sin(theta2)*sin(theta3) - 420*cos(theta2)*cos(theta3) - 35*cos(theta2)*sin(theta3) - 35*cos(theta3)*sin(theta2) - 455*sin(theta2) + 400;

J_V = jacobian([X Y Z], [theta1, theta2, theta3]);
J_V_Mat = [J_V, zeros(3)]


J_W = cell(1, 6); 
temp_J_W = cell(1, 6);

temp_J_W{1} = t2r(T_0_1);
J_W{1} = temp_J_W{1}(:, 3);

temp_J_W{2} = t2r(T_0_1 * T_1_2);
J_W{2} = temp_J_W{2}(:, 3);

temp_J_W{3} = t2r(T_0_1 * T_1_2 * T_2_3);
J_W{3} = temp_J_W{3}(:, 3);

temp_J_W{4} = t2r(T_0_1 * T_1_2 * T_2_3 * T_3_4);
J_W{4} = temp_J_W{4}(:, 3);

temp_J_W{5} = t2r(T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_5);
J_W{5} = temp_J_W{5}(:, 3);

temp_J_W{6} = t2r(T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_5 * T_5_6);
J_W{6} = temp_J_W{6}(:, 3);

J_W_Mat = [J_W{1}, J_W{2}, J_W{3}, J_W{4}, J_W{5}, J_W{6}]

Total_Jacobian = [J_V_Mat ; J_W_Mat]


% %jacobian: velocity propagation
% %angular velocity
w_1_1 = [0; 0; dt1];

T_1_2 = kuka_mod(2).A(theta2);
R_1_2 = t2r(T_1_2);
w_2_2 = transpose(R_1_2) * [0; 0; dt1] + [0; 0; dt2];

T_2_3 = kuka_mod(3).A(theta3);
R_2_3 = t2r(T_2_3);
w_3_3 = transpose(R_2_3) * w_2_2 + [0; 0; dt3];
simplify(w_3_3);

T_3_4 = kuka_mod(4).A(theta4);
R_3_4 = t2r(T_3_4);
w_4_4 = transpose(R_3_4) * w_3_3 + [0; 0; dt4];
simplify(w_4_4);

T_4_5 = kuka_mod(5).A(theta5);
R_4_5 = t2r(T_4_5);
w_5_5 = transpose(R_4_5) * w_4_4 + [0; 0; dt5];
simplify(w_5_5);

T_5_6 = kuka_mod(6).A(theta6);
R_5_6 = t2r(T_5_6);
w_6_6 = transpose(R_5_6) * w_5_5 + [0; 0; dt6];
collect(simplify(w_6_6), [dt1, dt2, dt3, dt4, dt5, dt6]);

% %linear veolocity
p_1_2 = [25; 0; 0];
v_1_1 = [0; 0; 0];
v_2_2 = transpose(R_1_2) * (cross(w_1_1,p_1_2) + v_1_1);

p_2_3 = [455; 0; 0];
v_3_3 = transpose(R_2_3) * (cross(w_2_2,p_2_3) + v_2_2);

p_3_4 = [35; 420; 0];
v_4_4 = transpose(R_3_4) * (cross(w_3_3,p_3_4) + v_3_3);
simplify(v_4_4);

p_4_5 = [0; 0; 0];
v_5_5 = transpose(R_4_5) * (cross(w_4_4,p_4_5) + v_4_4);
simplify(v_5_5);


p_5_6 = [0; 0; 0];
v_6_6 = transpose(R_5_6) * (cross(w_5_5,p_5_6) + v_5_5);
collect(simplify(v_6_6), [dt1 dt2 dt3]);

R_0_6 = t2r(HT_0_6);
TM_J = [R_0_6, zeros(3); zeros(3), R_0_6]
J_6 = [(- 5*cos(theta4)*sin(theta6)*(7*cos(theta2 + theta3) - 84*sin(theta2 + theta3) + 91*cos(theta2) + 5) - 5*cos(theta5)*cos(theta6)*sin(theta4)*(7*cos(theta2 + theta3) - 84*sin(theta2 + theta3) + 91*cos(theta2) + 5)), (cos(theta6)*(sin(theta5)*(455*cos(theta3) + 35) + cos(theta4)*cos(theta5)*(455*sin(theta3) - 420)) - sin(theta4)*sin(theta6)*(455*sin(theta3) - 420)), (420*sin(theta4)*sin(theta6) + cos(theta6)*(35*sin(theta5) - 420*cos(theta4)*cos(theta5))), 0, 0, 0;(5*cos(theta5)*sin(theta4)*sin(theta6)*(7*cos(theta2 + theta3) - 84*sin(theta2 + theta3) + 91*cos(theta2) + 5) - 5*cos(theta4)*cos(theta6)*(7*cos(theta2 + theta3) - 84*sin(theta2 + theta3) + 91*cos(theta2) + 5)), (- sin(theta6)*(sin(theta5)*(455*cos(theta3) + 35) + cos(theta4)*cos(theta5)*(455*sin(theta3) - 420)) - cos(theta6)*sin(theta4)*(455*sin(theta3) - 420)), (420*cos(theta6)*sin(theta4) - sin(theta6)*(35*sin(theta5) - 420*cos(theta4)*cos(theta5))), 0, 0, 0; 5*sin(theta4)*sin(theta5)*(7*cos(theta2 + theta3) - 84*sin(theta2 + theta3) + 91*cos(theta2) + 5), (cos(theta5)*(455*cos(theta3) + 35) - cos(theta4)*sin(theta5)*(455*sin(theta3) - 420)), (35*cos(theta5) + 420*cos(theta4)*sin(theta5)), 0, 0, 0;(sin(theta2 + theta3)*sin(theta4)*sin(theta6) - cos(theta6)*(cos(theta2 + theta3)*sin(theta5) + sin(theta2 + theta3)*cos(theta4)*cos(theta5))), (- cos(theta4)*sin(theta6) - cos(theta5)*cos(theta6)*sin(theta4)), (- cos(theta4)*sin(theta6) - cos(theta5)*cos(theta6)*sin(theta4)), cos(theta6)*sin(theta5), (-sin(theta6)), 0; (sin(theta6)*(cos(theta2 + theta3)*sin(theta5) + sin(theta2 + theta3)*cos(theta4)*cos(theta5)) + sin(theta2 + theta3)*cos(theta6)*sin(theta4)), (cos(theta5)*sin(theta4)*sin(theta6) - cos(theta4)*cos(theta6)), (cos(theta5)*sin(theta4)*sin(theta6) - cos(theta4)*cos(theta6)), (-sin(theta5)*sin(theta6)), (-cos(theta6)), 0; (sin(theta2 + theta3)*cos(theta4)*sin(theta5) - cos(theta2 + theta3)*cos(theta5)), sin(theta4)*sin(theta5), sin(theta4)*sin(theta5), cos(theta5), 0, 1];
J_0 = TM_J *  J_6;
simplify(J_0);

% %jacobian: backward propagation 
% %force / torque
% % force
f_6_6 = [fx; fy; fz];

f_5_5 = R_5_6 * f_6_6;

f_4_4 = R_4_5 * f_5_5;
collect(simplify(f_4_4), [fx, fy, fz]);


f_3_3 = R_3_4 * f_4_4;
collect(simplify(f_3_3), [fx, fy, fz]);

f_2_2 = R_2_3 * f_3_3;
collect(simplify(f_2_2), [fx, fy, fz]);

f_1_1 = R_1_2 * f_2_2;
collect(simplify(f_1_1), [fx, fy, fz]);

%torque
n_6_6 = [nx; ny; nz];

n_5_5 = R_5_6 * n_6_6 + cross(p_5_6, f_5_5);

n_4_4 = R_4_5 * n_5_5 + cross(p_4_5, f_4_4);
collect(simplify(n_4_4), [nx, ny, nz]);

n_3_3 = R_3_4 * n_4_4 + cross(p_3_4, f_3_3);
collect(simplify(n_3_3), [fx, fy, fz, nx, ny, nz]);

n_2_2 = R_2_3 * n_3_3 + cross(p_2_3, f_2_2);
collect(simplify(n_2_2), [fx, fy, fz, nx, ny, nz]);

n_1_1 = R_1_2 * n_2_2 + cross(p_1_2, f_1_1);
collect(simplify(n_1_1), [fx, fy, fz, nx, ny, nz]);



T1_6 = T_1_2 * T_2_3 * T_3_4 *T_4_5 * T_5_6;

expand((cos(theta1) * X + sin(theta1) * Y -25)^2);

T_3_6 = T_3_4 *T_4_5 * T_5_6;

T_0_3_inv = transpose(T_0_1 * T_1_2 * T_2_3);

A3 = cos(theta1)*cos(theta2+theta3)*X-sin(theta2+theta3)*cos(theta1)*Y-sin(theta1)*Z+5*cos(theta1)*(91*sin(theta2)+5);
D4 = sin(theta1)*cos(theta2+theta3)*X-sin(theta1)*sin(theta2+theta3)*Y+cos(theta1)*Z+5*sin(theta1)*(91*cos(theta2)+5);
simplify(expand(A3^ + D4));

T_0_4_inv = simplify(transpose(T_0_1 * T_1_2 * T_2_3 * T_3_4));
T_4_6 = T_4_5 * T_5_6;

simplify(sin(theta1)*sin(theta4) - cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)));
simplify(cos(theta4)*sin(theta1) + sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)));
simplify( - cos(theta1)*cos(theta2)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2));
simplify(-cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)));

T_0_5_inv = transpose(T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_5);
simplify(- sin(theta5)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - cos(theta4)*cos(theta5)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)));
simplify(cos(theta4)*sin(theta5)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) - cos(theta5)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)));
simplify(-sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)));

% %jacobian

X = simplify(25*cos(theta1) + 455*cos(theta1)*cos(theta2) - 35*cos(theta1)*sin(theta2)*sin(theta3) + 35*cos(theta1)*cos(theta2)*cos(theta3) - 420*cos(theta1)*cos(theta2)*sin(theta3) - 420*cos(theta1)*cos(theta3)*sin(theta2));
Y = simplify(25*sin(theta1) + 455*cos(theta2)*sin(theta1) - 420*cos(theta2)*sin(theta1)*sin(theta3) - 420*cos(theta3)*sin(theta1)*sin(theta2) - 35*sin(theta1)*sin(theta2)*sin(theta3) + 35*cos(theta2)*cos(theta3)*sin(theta1))
Z = simplify(420*sin(theta2)*sin(theta3) - 420*cos(theta2)*cos(theta3) - 35*cos(theta2)*sin(theta3) - 35*cos(theta3)*sin(theta2) - 455*sin(theta2) + 400);


simplify(- cos(theta1)*cos(theta2)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2))
simplify(- cos(theta2)*sin(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2))
simplify(sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3))


simplify(- cos(theta4)*sin(theta1) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))
simplify(cos(theta1)*cos(theta4) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))
simplify(-sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))

simplify(- sin(theta5)*(sin(theta1)*sin(theta4) - cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))) - cos(theta5)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))







