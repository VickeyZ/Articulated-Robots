close all; clear; clc;
%% inital state (angle [0.5 0.3 0.8 0.1 0.2 0.5]
L_1 = 284/1000; L_2 = 225/1000; L_3 = 228.9/1000; d_6 = 55/1000;
% the position of the orgain point of link6
D_pos = [0.076483, 0.0479149, 0.688358]';
% the RPY of the orgain point of lin6
D_sta = [2.6550221, -0.9675468, -0.5583089]';
r11 = cos(D_sta(2))*cos(D_sta(3));
r12 = sin(D_sta(1))*sin(D_sta(2))*cos(D_sta(3)) - cos(D_sta(1))*sin(D_sta(3));
r13 = sin(D_sta(1))*sin(D_sta(3)) + cos(D_sta(1))*sin(D_sta(2))*cos(D_sta(3));
r21 = cos(D_sta(2))*sin(D_sta(3));
r22 = cos(D_sta(1))*cos(D_sta(3)) + sin(D_sta(1))*sin(D_sta(2))*sin(D_sta(3));
r23 = cos(D_sta(1))*sin(D_sta(2))*sin(D_sta(3)) - cos(D_sta(3))*sin(D_sta(1));
r31 = -sin(D_sta(2));
r32 = cos(D_sta(2))*sin(D_sta(1));
r33 = cos(D_sta(1))*cos(D_sta(2));
R0_7 = [r11, r12, r13; r21, r22, r23; r31, r32, r33];
T0_7 = [R0_7, D_pos; 0 0 0 1];
% *************key point************* 
T6_7 = eye(4);
T6_7(2,4) = -d_6;
T0_6 = T0_7*inv(T6_7);
R0_6 = T0_6(1:3,1:3);
dx = T0_6(1,4); dy = T0_6(2,4); dz = T0_6(3,4);

%% solve theta_1 (right)
% reflect on XOY
theta_1 = atan2(dy,dx);    % (-pi, pi]

%% solve theta_3 (right)
% in XOZ plane (needn't consider the inital angle, for example pi/2
% L_3*cos(theta_2+theta_3) - L_2*sin(theta_2) = X/cos(theta_1)  _________ 1
% L_3*sin(theta_2+theta_3) + L_2*cos(theta_2) = Z - L_1         _________ 2
% (1)^2 + (2)^2
% sin(theta_2) = ((X/cos(theta_1))^2 + (Z-L_1)^2 - (L_2)^2 -
% (L_3)^2)/(2*L_2*L_3)
sinthe_3 = (dx^2+dy^2 + (dz-L_1)^2 - (L_2)^2 - (L_3)^2)/(2*L_2*L_3);
theta_3 = asin(sinthe_3);
theta_3 = atan2(sin(theta_3),cos(theta_3)); % ******

%% solve theta_2  (right)
% L_3*(cos(theta_2)cos(theta_3)-sin(theta_2)*sin(theta_3)) -
% L_2*sin(theta_2) = X/cos(theta_1)   ___________________ 1
% L_3*(sin(theta_2)*cos(theta_3)+cos(theta_2)*sin(theta_3)) +
% L_2*cos(theta_2) = Z - L_1          ___________________ 2
A = [L_3*cos(theta_3), -(L_3*sin(theta_3)+L_2); (L_3*sin(theta_3)+L_2), L_3*cos(theta_3)];
B = [sqrt(dx^2+dy^2); dz-L_1];
% d = L_3*cos(theta_3);
% f = L_3*sin(theta_3)+L_2;
% g = D_pos(1)/cos(theta_1);
% h = D_pos(3) - L_1;
% sinthe2 = (g*f - h*d) / (d^2 - f^2);
% costhe2 = (g*d - h*f) / (d^2 - f^2);

% result = [cos;sin];
result = inv(A)*B;
theta_2 = atan2(result(2),result(1));
%theta_2 = theta_2+pi/2;
%% solve theta_4 theta_5 theta_6 
% m11 = cos(theta_1)*cos(theta_2+theta_3);
% m12 = sin(theta_1);
% m13 = cos(theta_1)*sin(theta_2+theta_3);
% m21 = sin(theta_1)*cos(theta_2+theta_3);
% m22 = -cos(theta_1);
% m23 = sin(theta_1)*sin(theta_2+theta_3);
% m31 = sin(theta_2+theta_3);
% m32 = 0;
% m33 = -cos(theta_2+theta_3);
m11 = -cos(theta_1)*sin(theta_2+theta_3);
m12 = sin(theta_1);
m13 = cos(theta_1)*cos(theta_2+theta_3);
m21 = -sin(theta_1)*sin(theta_2+theta_3);
m22 = -cos(theta_1);
m23 = sin(theta_1)*cos(theta_2+theta_3);
m31 = cos(theta_2+theta_3);
m32 = 0;
m33 = sin(theta_2+theta_3);
R0_3 = [m11, m12, m13; m21, m22, m23; m31, m32, m33];
R3_6 = (R0_3')*R0_6;
theta_4 = atan2(R3_6(2,2),R3_6(1,2))-pi;
theta_42 = theta_4+pi;
theta_5 = atan2(sqrt(R3_6(1,2)^2+R3_6(2,2)^2), R3_6(3,2))-pi/2;
theta_52 = theta_5+pi;
theta_6 = atan2(R3_6(3,3), -R3_6(3,1));
theta_62 = theta_6+pi;

%% answer
theta = [theta_1 theta_2 theta_3 theta_4 theta_5 theta_6;
    theta_1 theta_2 theta_3 theta_42 theta_5 theta_6;
    theta_1 theta_2 theta_3 theta_4 theta_52 theta_6;
    theta_1 theta_2 theta_3 theta_4 theta_5 theta_62;
    theta_1 theta_2 theta_3 theta_42 theta_52 theta_6;
    theta_1 theta_2 theta_3 theta_42 theta_5 theta_62;
    theta_1 theta_2 theta_3 theta_4 theta_52 theta_62;
    theta_1 theta_2 theta_3 theta_42 theta_52 theta_62];