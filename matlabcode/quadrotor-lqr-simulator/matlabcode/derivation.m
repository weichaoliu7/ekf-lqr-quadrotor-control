clear all;
clc;
syms roll pitch yaw
syms angular_velocity_x angular_velocity_y angular_velocity_z
% syms velocity_i velocity_j velocity_k
% syms position_x position_y position_z
syms ft torque_roll torque_pitch torque_yaw
syms m g I_x I_y I_z
    
f1 = angular_velocity_x + angular_velocity_y * sin(roll) * tan(pitch) + angular_velocity_z * cos(roll) * tan(pitch);
f2 = angular_velocity_y * cos(roll) - angular_velocity_z * sin(roll);
f3 = angular_velocity_y * sin(roll) / cos(pitch) + angular_velocity_z * cos(roll) / cos(pitch);

f4 = (I_y-I_z) / I_x * angular_velocity_y * angular_velocity_z + torque_roll/I_x;
f5 = (I_z-I_x) / I_y * angular_velocity_x * angular_velocity_z + torque_pitch/I_y;
f6 = (I_x-I_y) / I_z * angular_velocity_x * angular_velocity_y + torque_yaw/I_z;

% f7 = angular_velocity_z * velocity_j - angular_velocity_y * velocity_k - g * sin(pitch);
% f8 = angular_velocity_x * velocity_k - angular_velocity_z * velocity_i + g * cos(pitch) * sin(roll);
% f9 = angular_velocity_y * velocity_i - angular_velocity_x * velocity_j + g * cos(pitch) * cos(roll) - ft/m;
% 
% f10 = velocity_i * cos(pitch) * cos(yaw) + velocity_j * (sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw)) + velocity_k *  (cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw));
% f11 = velocity_i * cos(pitch) * sin(yaw) + velocity_j * (sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw)) + velocity_k *  (cos(roll)*sin(pitch)*sin(yaw) + sin(roll)*cos(yaw));
% f12 = velocity_i * (-sin(pitch)) + velocity_j * sin(roll)*cos(pitch) + velocity_k *  cos(roll)*cos(pitch);

% x = [roll pitch yaw angular_velocity_x angular_velocity_y angular_velocity_z velocity_i velocity_j velocity_k position_x position_y position_z]';
x = [roll pitch yaw angular_velocity_x angular_velocity_y angular_velocity_z]';
f = [f1 f2 f3 f4 f5 f6]';
A = jacobian(f',x');
disp("A = ");disp(A);

A_e = subs(A,[roll pitch yaw angular_velocity_x angular_velocity_y angular_velocity_z g],[0,0,0,0,0,0,9.81]);
A_e = double(A_e);
disp("A_e = ");disp(A_e);

u =[ft torque_roll torque_pitch torque_yaw]';

B = jacobian(f',u');
disp("B = ");disp(B);
B_e = subs(B,[I_x I_y I_z m],[0.022, 0.022, 0.03, 0.895]);
B_e = double(B_e);
disp("B_e = ");disp(B_e);

