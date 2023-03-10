function invalidIK = isIKInvalid(point)

    [theta] = InverseKinematics2(point(1),point(2),point(3),point(4));

    if length(theta) < 4
        invalidIK = true;

    else
        invalidIK = false;
    end

end