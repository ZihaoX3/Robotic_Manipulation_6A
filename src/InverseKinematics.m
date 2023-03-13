function [theta1,theta2,theta3,theta4] = InverseKinematics(x,y,z,gripper_orientation)

    % link lengths
    l1 = 0.077;
    l2 = 0.13;
    l3 = 0.124;
    l4 = 0.126;
    
    
%     gripper_orientation = deg2rad(90);% orientation of gripper with respect to world frame
    
    r3 = sqrt(x^2+y^2);% distance of end effector from origin in x-y plane
    z3 = z - l1;% height of end effector
    
    % calculating coordinates of wrist joint
    r2 = r3 - l4*cos(gripper_orientation);
    z2 = z3 - l4*sin(gripper_orientation);
    
    %Inverse Kinematics
    theta1 = atan2(y,x);
    
    % theta3
    arg = ( ((r2^2 + z2^2) - (l2^2 + l3^2)) / (2*l2*l3) );
    
    theta3 = -acos(arg);% two sols + and - when feasible? 
    
    % theta2
    theta2_denom = (((l2 + l3*cos(theta3))*z2) - ((l3*sin(theta3))*r2))/ (r2^2 + z2^2);
    
    theta2_num = (((l2 + l3*cos(theta3))*r2) + ((l3*sin(theta3))*z2))/ (r2^2 + z2^2);
    
    theta2 = atan2(theta2_num,theta2_denom); %derived by defining wrist joint using basic trig and theta2 and theta3, then rearranged
    
    % theta4 
    theta4 = gripper_orientation - theta3 - pi/2 + theta2;% gripper orientation with respect to world frame is sum of total rotations of links in z-r plane
    
    % Adjust angles to match the way we defined them in the DH table
    %     constant = pi/2 - gamma;  
    % theta2 =  - (theta2 - (deg2rad(90) - constant));% -theta2 +79 deg
    gamma = atan(0.024/0.128);

    
    theta2 =  - (theta2 -gamma);
    theta3 = theta3 + deg2rad(90) - gamma;


    
    % disp("theta1 "+ rad2deg(theta1))
    % disp("theta2 "+ rad2deg(theta2))
    % disp("theta3 "+ rad2deg(theta3))
    % disp("theta4 "+ rad2deg(theta4))
end



