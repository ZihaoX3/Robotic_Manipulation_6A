function jointLimitsOk = withinJointLimits(point)
        
        [theta] = InverseKinematics2(point(1),point(2),point(3),point(4));
        theta5 = point(5);
        

        % check if solution is within joint limits 
    if ((rad2deg(theta(1)) > 100) || (rad2deg(theta(1)) < -100))
        jointLimitsOk = false;

    elseif  ((rad2deg(theta(2)) > 100) || (rad2deg(theta(2)) < -120))
        jointLimitsOk = false;

    elseif ((rad2deg(theta(3)) > 90) || (rad2deg(theta(3)) < -120))
        jointLimitsOk = false;

    elseif ((rad2deg(theta(4)) > 120) || (rad2deg(theta(4)) < -110))
        jointLimitsOk = false;

%     elseif ((rad2deg(theta5) > 232) || (rad2deg(theta5) < 90))
%         jointLimitsOk = false;

    else
        jointLimitsOk = true;
    end

end

    
