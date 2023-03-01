[theta1_rad, theta2_rad, theta3_rad, theta4_rad] = InverseKinematics(0.1, 0, 0.1, deg2rad(0))
% constant = - atan2(0.024,0.128) + pi/2;
%theta2_rad = constant + theta2_rad;
%theta3_rad = - constant + theta3_rad;
theta1 = normal_convert(theta1_rad);
theta2 = normal_convert(theta2_rad);
theta3 = theta3_convert(theta3_rad);
theta4 = theta4_convert(theta4_rad);
%function theta1 = final_convert1(theta1_rad)
%    theta1 = normal_convert(theta1_rad);
%end

%function theta2= final_convert2(theta2_rad)
%    theta2 = normal_convert(theta2_rad);
%end
%function theta3 = final_convert3(theta3_rad)
%    theta3 = theta3_convert(theta3_rad);
%end
%function theta4 = final_convert4(theta4_rad)
%    theta4 = theta4_convert(theta4_rad);
%end