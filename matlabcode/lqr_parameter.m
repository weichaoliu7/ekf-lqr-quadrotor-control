%% attitude control
clear all;
clc;
syms roll pitch yaw
syms angular_velocity_x angular_velocity_y angular_velocity_z
syms roll_angular_velocity pitch_angular_velocity yaw_angular_velocity
syms velocity_i velocity_j velocity_k
syms velocity_x velocity_y velocity_z
syms I_x I_y I_z
syms torque_roll torque_pitch torque_yaw
syms g

f1 = angular_velocity_x + angular_velocity_y * sin(roll) * tan(pitch) + angular_velocity_z * cos(roll) * tan(pitch);
f2 = angular_velocity_y * cos(roll) - angular_velocity_z * sin(roll);
f3 = angular_velocity_y * sin(roll) / cos(pitch) + angular_velocity_z * cos(roll) / cos(pitch);

f4 = (I_y-I_z) / I_x * angular_velocity_y * angular_velocity_z + torque_roll/I_x;
f5 = (I_z-I_x) / I_y * angular_velocity_x * angular_velocity_z + torque_pitch/I_y;
f6 = (I_x-I_y) / I_z * angular_velocity_x * angular_velocity_y + torque_yaw/I_z;

f7 = angular_velocity_z * velocity_y - angular_velocity_y * velocity_z - g * sin(pitch);
f8 = angular_velocity_x * velocity_z - angular_velocity_z * velocity_x + g * cos(pitch) * sin(roll);
A = jacobian([f1;f2;f3;f4;f5;f6;f7;f8],[roll;pitch;yaw;angular_velocity_x;angular_velocity_y;angular_velocity_z;velocity_x;velocity_y]);

A_e = subs(A, [roll, pitch, yaw, roll_angular_velocity, pitch_angular_velocity, yaw_angular_velocity, angular_velocity_x, angular_velocity_y, angular_velocity_z, velocity_x, velocity_y, velocity_z, g], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9.81]);
A_e = double(A_e);
disp("A_e = ");disp(A_e);

B = jacobian([f1;f2;f3;f4;f5;f6;f7;f8],[torque_roll torque_pitch torque_yaw]);
B_e = subs(B, [I_x, I_y, I_z], [0.022 0.022 0.03]);
B_e = double(B_e);
disp("B_e = ");disp(B_e);

Q_e = diag([1 1 1 1.5 1.5 1.5 0.5 0.5]);
R_e = diag([16 16 25]);

K_e = lqr(A_e,B_e,Q_e,R_e);
disp("K_e = ");disp(K_e);

%% altitude control
clear all;
clc;
syms position_z
syms velocity_z

f1 = velocity_z;

f2 = angular_velocity_y * cos(roll) - angular_velocity_z * sin(roll);
f3 = angular_velocity_y * sin(roll) / cos(pitch) + angular_velocity_z * cos(roll) / cos(pitch);

f4 = (I_y-I_z) / I_x * angular_velocity_y * angular_velocity_z + torque_roll/I_x;
f5 = (I_z-I_x) / I_y * angular_velocity_x * angular_velocity_z + torque_pitch/I_y;
f6 = (I_x-I_y) / I_z * angular_velocity_x * angular_velocity_y + torque_yaw/I_z;

f7 = angular_velocity_z * velocity_y - angular_velocity_y * velocity_z - g * sin(pitch);
f8 = angular_velocity_x * velocity_z - angular_velocity_z * velocity_x + g * cos(pitch) * sin(roll);
A = jacobian([f1;f2;f3;f4;f5;f6;f7;f8],[roll;pitch;yaw;angular_velocity_x;angular_velocity_y;angular_velocity_z;velocity_x;velocity_y]);

A_e = subs(A, [roll, pitch, yaw, roll_angular_velocity, pitch_angular_velocity, yaw_angular_velocity, angular_velocity_x, angular_velocity_y, angular_velocity_z, velocity_x, velocity_y, velocity_z, g], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9.81]);
A_e = double(A_e);
disp("A_e = ");disp(A_e);

B = jacobian([f1;f2;f3;f4;f5;f6;f7;f8],[torque_roll torque_pitch torque_yaw]);
B_e = subs(B, [I_x, I_y, I_z], [0.022 0.022 0.03]);
B_e = double(B_e);
disp("B_e = ");disp(B_e);

Q_e = diag([1 1 1 1.5 1.5 1.5 0.5 0.5]);
R_e = diag([16 16 25]);

K_e = lqr(A_e,B_e,Q_e,R_e);
disp("K_e = ");disp(K_e);