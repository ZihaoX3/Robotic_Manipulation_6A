function invalidIK = isIKInvalid(point)
    % since we append angles to theta in inverse kinematics, simply check if list is long enough

    [theta] = InverseKinematics2(point(1),point(2),point(3),point(4));

    if length(theta) < 4
        invalidIK = true;

    else
        invalidIK = false;
    end

end
