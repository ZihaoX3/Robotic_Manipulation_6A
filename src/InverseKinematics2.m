function [theta] = InverseKinematics2(x,y,z,gripper_orientation)

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
    theta(1) = atan2(y,x); 
    
    % theta(3)
    arg = ( ((r2^2 + z2^2) - (l2^2 + l3^2)) / (2*l2*l3) ); %from T1_2*T2_3 

    
    theta3 = - acos(arg); % -ve is elbow up
    
    %change to elbow down if violates joint limits (joint limits probably wrong)
%     if ((rad2deg(theta3) > 150) || -(rad2deg(theta3) > 90))
%         theta3 = acos(arg);
%         rad2deg(theta3)
%         disp("changing to elbow down")
%     end
          
    if isreal(theta3)
            theta(3)= theta3;
    else
        disp("invalid theta3")
        return
    end
    
    % theta(2)
    theta2_denom = (((l2 + l3*cos(theta(3)))*z2) - ((l3*sin(theta(3)))*r2))/ (r2^2 + z2^2);
    
    theta2_num = (((l2 + l3*cos(theta(3)))*r2) + ((l3*sin(theta(3)))*z2))/ (r2^2 + z2^2);

    theta2 = atan(theta2_num/theta2_denom);

    if isreal(theta2)
            theta(2)= atan2(theta2_num,theta2_denom);
    else
        disp("invalid theta2")
        return
    end

   % theta(4)
    theta(4) = gripper_orientation - theta(3) - pi/2 + theta(2);% gripper orientation with respect to world frame is sum of total rotations of links in z-r plane

    % Adjust angles for robot
    gamma = atan(0.024/0.128);

    
    theta(2) =  - (theta(2) -gamma);
    theta(3) = theta(3) + deg2rad(90) - gamma;


    
    % disp("theta(1) "+ rad2deg(theta(1)))
    % disp("theta(2) "+ rad2deg(theta(2)))
    % disp("theta(3) "+ rad2deg(theta(3)))
    % disp("theta(4) "+ rad2deg(theta(4)))
end



