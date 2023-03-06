[theta1_rad, theta2_rad, theta3_rad, theta4_rad] = InverseKinematics( 0.1, 0.1, 0.1, deg2rad(-90));

rad2deg(theta1_rad)
rad2deg(theta2_rad)
rad2deg(theta3_rad)
rad2deg(theta4_rad)
%output same as inverseKin

% %Take into account offset angle (done in IK now)
% theta2_rad = constant + theta2_rad;
% theta3_rad = - constant + theta3_rad;
% constant = - atan2(0.024,0.128) + pi/2;
%Default Pose 0.2740,0.0000,0.2048

theta1 = radians_to_encoder_position(theta1_rad)
theta2 = radians_to_encoder_position(-theta2_rad)
theta3 = radians_to_encoder_position(-theta3_rad)
theta4 = radians_to_encoder_position(-theta4_rad)
