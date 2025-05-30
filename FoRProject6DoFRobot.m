clc;
clear variables;

syms theta th1 th2 th3 th4 th5 th6 th7 d1 d2 d3 d4 d5 d6 d7 a1 a2 a3 a4 a5 a6 a7 alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 alpha7

% Defining DH Table
DHtable = [
    th1, d1, a1, alpha1;
    th2, d2, a2, alpha2;
    th3, d3, a3, alpha3;
    th4, d4, a4, alpha4;
    th5, d5, a5, alpha5;
    th6, d6, a6, alpha6;
    th7, d7, a7, alpha7];

% Home Configuration (q vector = 0)
% Change values here to change joint angles (theta in degrees, distance in meters)
q1 = 20;     q2 = 20;     q3 = 20;     q4 = 20;     q5 = 20;     q6 = 20;

% DH Parameters
the1 = 0+q1;    dis1 = 0.78;    l1 = 0.41;      alph1 = -90;
the2 = -90+q2;  dis2 = 0;       l2 = 1.075;     alph2 = 0;
the3 = 0+q3;    dis3 = 0;       l3 = 0.165;     alph3 = -90;
the4 = 0+q4;    dis4 = 1.056;   l4 = 0;         alph4 = 90;
the5 = 0+q5;    dis5 = 0;       l5 = 0;         alph5 = -90;
the6 = 180+q6;  dis6 = 0.25;    l6 = 0;         alph6 = 0;
the7 = 0;       dis7 = 0;       l7 = 2;         alph7 = 0;

% Substituting inputs into symbols in DHtable function
DHtablevalues = subs(DHtable,{th1 th2 th3 th4 th5 th6 th7 ...
    d1 d2 d3 d4 d5 d6 d7 ...
    a1 a2 a3 a4 a5 a6 a7 ...
    alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 alpha7}, ...
    {the1 the2 the3 the4 the5 the6 the7 ...
    dis1 dis2 dis3 dis4 dis5 dis6 dis7 ...
    l1 l2 l3 l4 l5 l6 l7 ...
    alph1 alph2 alph3 alph4 alph5 alph6 alph7});

disp('DH Table');
disp(eval(DHtablevalues));
disp('  ');

% Forward Kinematics
% Calculating Transformation Matrix
T = eye(4);
for i = 1:size(DHtable, 1)
    % Extract DH parameters for the current joint
    a_i = DHtablevalues(i, 3);
    alpha_i = DHtablevalues(i, 4);
    d_i = DHtablevalues(i, 2);
    theta_i = DHtablevalues(i, 1);
    
    % Compute transformation matrix for the current joint
    A_i = [
        cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), a_i*cos(theta_i);
        sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), a_i*sin(theta_i);
        0, sin(alpha_i), cos(alpha_i), d_i;
        0, 0, 0, 1];

    % Update overall transformation matrix
    T = T * A_i;

    % Save the current transformation matrix in the cell array
    T_array{i} = A_i;
end

% Formulating Transformation Matrix with respect to base frame (0)
T01 = eval(T_array{1});
T02 = eval(T_array{1})*eval(T_array{2});
T03 = eval(T_array{1})*eval(T_array{2})*eval(T_array{3});
T04 = eval(T_array{1})*eval(T_array{2})*eval(T_array{3})*eval(T_array{4});
T05 = eval(T_array{1})*eval(T_array{2})*eval(T_array{3})*eval(T_array{4})*eval(T_array{5});
T06 = eval(T_array{1})*eval(T_array{2})*eval(T_array{3})*eval(T_array{4})*eval(T_array{5})*eval(T_array{6});
T07 = eval(T_array{1})*eval(T_array{2})*eval(T_array{3})*eval(T_array{4})*eval(T_array{5})*eval(T_array{6})*eval(T_array{7});

% Display the final Transformation Matrix
disp('  ');
disp('Forward Kinematics:');
Tvalue = eval(T);
disp(Tvalue);

% Inverse Kinematics
% Extract rotation matrix and translation vector from the final Transformation Matrix
R = Tvalue(1:3, 1:3);
P = Tvalue(1:3, 4);

% Calculating joint angles
theta1_val = rad2deg(atan2(P(2), P(1)));
theta2_val = rad2deg(atan2(sqrt((P(1))^2 + (P(2))^2) - l1, P(3) - dis1));
theta3_val = rad2deg(atan2(R(3, 2), R(3, 1)));
theta4_val = rad2deg(atan2(R(2, 3), -R(1, 3)));
theta5_val = rad2deg(acos(R(3, 3)));
theta6_val = rad2deg(atan2(R(2, 3), R(1, 3)));

% Display joint angles
disp('  ');
disp('Inverse Kinematics:');
disp('Theta 1:');
disp(theta1_val);
disp('  ');
disp('Theta 2:');
disp(theta2_val);
disp('  ');
disp('Theta 3:');
disp(theta3_val);
disp('  ');
disp('Theta 4:');
disp(theta4_val);
disp('  ');
disp('Theta 5:');
disp(theta5_val);
disp('  ');
disp('Theta 6:');
disp(theta6_val);

% Visualization
% Extracting joint origin's positions
L0xpos = 0;        L0ypos = 0;        L0zpos = 0;
L1xpos = T01(1,4); L1ypos = T01(2,4); L1zpos = T01(3,4);
L2xpos = T02(1,4); L2ypos = T02(2,4); L2zpos = T02(3,4);
L3xpos = T03(1,4); L3ypos = T03(2,4); L3zpos = T03(3,4);
L4xpos = T04(1,4); L4ypos = T04(2,4); L4zpos = T04(3,4);
L5xpos = T05(1,4); L5ypos = T05(2,4); L5zpos = T05(3,4);
L6xpos = T06(1,4); L6ypos = T06(2,4); L6zpos = T06(3,4);
L7xpos = T07(1,4); L7ypos = T07(2,4); L7zpos = T07(3,4);

% Plot 3D stick figure of the robot
plot_robot_stick_figure(L0xpos, L0ypos, L0zpos, L1xpos, L1ypos, L1zpos, L2xpos, L2ypos, L2zpos, L3xpos, L3ypos, L3zpos, L4xpos, L4ypos, L4zpos, L5xpos, L5ypos, L5zpos, L6xpos, L6ypos, L6zpos, L7xpos, L7ypos, L7zpos);

% Inverse Dynamics - Jacobian
% Assume external wrench applied at the end effector
% Specify force and moment components
force = [1; 2; 3]; % Example force vector [Fx; Fy; Fz]
moment = [0.1; 0.2; 0.3]; % Example moment vector [Mx; My; Mz]

% Combine force and moment into a wrench vector
wrench = [force; moment];

% Calculate the Jacobian matrix
Jacobian = zeros(6, 6);
for i = 1:size(DHtable, 1)
    % Extract DH parameters for the current joint
    a_i = DHtablevalues(i, 3);
    alpha_i = DHtablevalues(i, 4);
    d_i = DHtablevalues(i, 2);
    theta_i = DHtablevalues(i, 1);

    % Compute transformation matrix for the current joint
    A_i = [
        cos(theta_i), -sin(theta_i) * cos(alpha_i), sin(theta_i) * sin(alpha_i), a_i * cos(theta_i);
        sin(theta_i), cos(theta_i) * cos(alpha_i), -cos(theta_i) * sin(alpha_i), a_i * sin(theta_i);
        0, sin(alpha_i), cos(alpha_i), d_i;
        0, 0, 0, 1
    ];

    % Calculate the joint axis in the base frame
    z_i = A_i(1:3, 3);

    % Calculate the position vector from the joint i to the end effector in the base frame
    p_i = Tvalue(1:3, 4) - A_i(1:3, 4);

    % Calculate the cross product to obtain the rotational component of the Jacobian
    J_angular = cross(z_i, p_i);

    % Populate the Jacobian matrix
    Jacobian(:, i) = [z_i; J_angular];
end

% Calculate joint torques using Inverse Dynamics
joint_torques = Jacobian' * wrench;

% Display joint torques
disp('  ');
disp('Inverse Dynamics - Joint Torques:');
disp(joint_torques);

% Function to plot the 3D stick figure of the robot
function plot_robot_stick_figure(L0xpos, L0ypos, L0zpos, L1xpos, L1ypos, L1zpos, L2xpos, L2ypos, L2zpos, L3xpos, L3ypos, L3zpos, L4xpos, L4ypos, L4zpos, L5xpos, L5ypos, L5zpos, L6xpos, L6ypos, L6zpos, L7xpos, L7ypos, L7zpos)

    figure;
    hold on;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(3);

    % Plot joints
    plot3(L0xpos, L0ypos, L0zpos, 'ro', 'MarkerSize', 8);
    plot3(L1xpos, L1ypos, L1zpos, 'ro', 'MarkerSize', 8);
    plot3(L2xpos, L2ypos, L2zpos, 'ro', 'MarkerSize', 8);
    plot3(L3xpos, L3ypos, L3zpos, 'ro', 'MarkerSize', 8);
    plot3(L4xpos, L4ypos, L4zpos, 'ro', 'MarkerSize', 8);
    plot3(L5xpos, L5ypos, L5zpos, 'ro', 'MarkerSize', 8);
    plot3(L6xpos, L6ypos, L6zpos, 'ro', 'MarkerSize', 8);
    plot3(L7xpos, L7ypos, L7zpos, 'ro', 'MarkerSize', 8);

    % Plot links
    plot3([L0xpos, L1xpos], [L0ypos, L1ypos], [L0zpos, L1zpos], 'LineWidth', 2);
    plot3([L1xpos, L2xpos], [L1ypos, L2ypos], [L1zpos, L2zpos], 'LineWidth', 2);
    plot3([L2xpos, L3xpos], [L2ypos, L3ypos], [L2zpos, L3zpos], 'LineWidth', 2);
    plot3([L3xpos, L4xpos], [L3ypos, L4ypos], [L3zpos, L4zpos], 'LineWidth', 2);
    plot3([L4xpos, L5xpos], [L4ypos, L5ypos], [L4zpos, L5zpos], 'LineWidth', 2);
    plot3([L5xpos, L6xpos], [L5ypos, L6ypos], [L5zpos, L6zpos], 'LineWidth', 2);
    plot3([L6xpos, L7xpos], [L6ypos, L7ypos], [L6zpos, L7zpos], 'LineWidth', 2);

    % Set axis limits
    axis equal;
    xlim([-2, 4]);
    ylim([-2, 4]);
    zlim([-2, 4]);

    % Display the plot
    hold off;
end