%final 2 trajectory generation
%interpolation at the joint space


clc;
clear all;

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
figure(7);
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


for i = 1:1:6
    str = ['cubic trajectory with joint ', num2str(i)];
    figure(i);
    %plotting
    subplot(311);
    plot(total_t, total_q(i,:));
    title(str);
    ylabel('position q(t)');
%     ylim([-3 3]);

    subplot(312);
    plot(total_t, total_dq(i,:));
    ylabel('velocity dq(t)');
%     ylim([-3 3]);

    subplot(313)
    plot(total_t, total_ddq(i,:));
    ylabel('acceleration ddq(t)');
    xlabel('time (t)');
%     ylim([-3 3]);
end






