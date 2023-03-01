[theta1_rad, theta2_rad, theta3_rad, theta4_rad] = InverseKinematics(0.1, 0.1, 0.1, deg2rad(0))
% constant = - atan2(0.024,0.128) + pi/2;
theta2_rad = constant + theta2_rad;
theta3_rad = - constant + theta3_rad;

theta1 = radians_to_encoder_position(theta1_rad)
theta2 = radians_to_encoder_position(theta2_rad)
theta3 = radians_to_encoder_position(theta3_rad)
theta4 = radians_to_encoder_position(theta4_rad)

