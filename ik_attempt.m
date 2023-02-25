%link lengths
l1 = 0.077;
l2 = 0.13;
l3 = 0.124;
l4 = 0.126;

%goal coords
x = 0.0631;
y = 0.000;
z = -0.0724;

gripper_orientation = pi/2;

end_effec_length = sqrt(x^2+y^2);
end_effec_height = z - 0.077;

%calculate wrist position
rx = end_effec_height - l4*cos(gripper_orientation);
ry = end_effec_length - l4*sin(gripper_orientation);

c = sqrt(rx^2+ry^2);%+rz^2);
constant = atan(0.024/0.128);

theta1 = atan2(y,x);
theta2 = asin(sqrt(rx^2+ry^2)/c)- acos((l2^2+c^2-l3^2)/2*l2*c);
theta3 = pi - acos((l2^2+l3^2-c^2)/2*l2*l3);
theta4 = gripper_orientation - theta2- theta3;

disp("theta2 "+ rad2deg(theta2))
disp("theta3 "+ rad2deg(theta3))
disp("theta4 "+ rad2deg(theta4))

