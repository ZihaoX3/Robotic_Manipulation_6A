function [pos_points1,pos_points2,pos_points3,pos_points4, pos_points5] = cubicInterp(points, number_of_intermediate_points)

        pos_points1 = [];
        pos_points2 = [];
        pos_points3 = [];
        pos_points4 = [];
        pos_points5 = [];

    
        j = 1;
    while  j < size(points,1)
        
        currentPoint = points(j, 1:3);
        nextPoint = points(j+1, 1:3);
        current_thetaG_pos = points(j,4);
        next_thetaG_pos = points(j+1, 4);
        gripper_angle = points(j, 5);
        [theta] = InverseKinematics2(currentPoint(1),currentPoint(2),currentPoint(3),current_thetaG_pos);
        [thetaf] = InverseKinematics2(nextPoint(1),nextPoint(2),nextPoint(3),next_thetaG_pos);

%         theta = [theta1, theta2,theta3,theta4];
%         thetaf = [thetaf1, thetaf2,thetaf3,thetaf4];
    
        n = number_of_intermediate_points;
        
        [q1, qd1, qdd1] = cubicTrajectoryPlanning(n, theta(1), thetaf(1), 0, 0);
        [q2, qd2, qdd2] = cubicTrajectoryPlanning(n, theta(2), thetaf(2), 0, 0);
        [q3, qd3, qdd3] = cubicTrajectoryPlanning(n, theta(3), thetaf(3), 0, 0);
        [q4, qd4, qdd4] = cubicTrajectoryPlanning(n, theta(4), thetaf(4), 0, 0);

        pos_points1 = [pos_points1, q1];
        pos_points2 = [pos_points2, q2];
        pos_points3 = [pos_points3, q3];
        pos_points4 = [pos_points4, q4];

        for i = 1:n
            pos_points5 = [pos_points5, gripper_angle];
        end

       

        j = j+1; 

    end
    
end