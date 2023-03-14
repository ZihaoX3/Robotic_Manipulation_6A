function [pos_points1,pos_points2,pos_points3,pos_points4, pos_points5] = cubicInterp_cartesian(points, number_of_intermediate_points)

        pos_points1 = [];
        pos_points2 = [];
        pos_points3 = [];
        pos_points4 = [];
        pos_points5 = [];

    
        j = 1;
    while  j < size(points,1)
        j
        currentPoint = points(j, 1:3);
        nextPoint = points(j+1, 1:3);
        current_thetaG_pos = points(j,4);
        next_thetaG_pos = points(j+1, 4);
        gripper_angle = points(j, 5);
        next_gripper_angle = points(j+1, 5);
            
        n = number_of_intermediate_points;
        
        [q1, qd1, qdd1] = cubicTrajectoryPlanning(n, currentPoint(1), nextPoint(1), 0, 0);%x
        [q2, qd2, qdd2] = cubicTrajectoryPlanning(n, currentPoint(2), nextPoint(2), 0, 0);%y
        [q3, qd3, qdd3] = cubicTrajectoryPlanning(n, currentPoint(3), nextPoint(3), 0, 0);%z
        [q4, qd4, qdd4] = cubicTrajectoryPlanning(n, current_thetaG_pos, next_thetaG_pos, 0, 0);%gripper orientation
        
        
        for i = 1:length(q1)
            [theta] = InverseKinematics2(q1(i),q2(i),q3(i),q4(i));
       
        pos_points1 = [pos_points1, theta(1)];
        pos_points2 = [pos_points2, theta(2)];
        pos_points3 = [pos_points3, theta(3)];
        pos_points4 = [pos_points4, theta(4)];

            if i > length(q1)/1.1
                pos_points5 = [pos_points5, next_gripper_angle];
            else
                pos_points5 = [pos_points5, gripper_angle];
            end   
        end

        j = j+1; 
        
    end
end