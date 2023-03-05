clc
%clear all
close all

% constant = - atan2(0.024,0.128) + pi/2;% offset angle between link 2 and 3
gamma = atan2(0.024,0.128);% offset angle between link 2 and 3

% theta1 = deg2rad(0);
% theta2 = constant + deg2rad(0);
% theta3 = - constant + deg2rad(0);
% theta4 = deg2rad(0);

alpha1 = pi/2;
alpha2 = 0;
alpha3 = 0;
alpha4 = 0;

a1 = 0;
a2 = 0.13;
a3 = 0.124;
a4 = 0.126;

d1 = 0.077;
d2 = 0;
d3 = 0;
d4 = 0;

plot3([0 0], [0 0], [0 0])
xlabel('X')
ylabel('Y')
zlabel('Z')

title("Inverse Kinematics")

grid on
hold on
xlim([-0.3 0.3])
ylim([-0.3 0.3])
zlim([0 0.4])


divisions = 20;
%10cm square in x-y plane
% x = [linspace(0.0, 0.1, divisions), zeros(1,divisions)+0.1, linspace(0.1-0.1/divisions, 0.0, divisions), zeros(1,divisions)];
% y = [zeros(1,divisions)+0.1 , linspace(0.1+0.1/divisions, 0.2, divisions), zeros(1,divisions)+0.2, linspace(0.2-0.1/divisions, 0.1, divisions)];
% z = zeros(1,divisions*4)+0.1;

% %10cm square in z-x plane
% x = [linspace(0.0, 0.1, divisions), zeros(1,divisions)+0.1, linspace(0.1-0.1/divisions, 0.0, divisions), zeros(1,divisions)];
% z = [zeros(1,divisions)+0.1 , linspace(0.1+0.1/divisions, 0.2, divisions), zeros(1,divisions)+0.2, linspace(0.2-0.1/divisions, 0.1, divisions)];
% y = zeros(1,divisions*4)+0.2;

% %10cm square in z-y plane
y = [linspace(0.0, 0.1, divisions), zeros(1,divisions)+0.1, linspace(0.1-0.1/divisions, 0.0, divisions), zeros(1,divisions)];
z = [zeros(1,divisions)+0.1 , linspace(0.1+0.1/divisions, 0.2, divisions), zeros(1,divisions)+0.2, linspace(0.2-0.1/divisions, 0.1, divisions)];
x = zeros(1,divisions*4)+0.2;

b=1;
while (b <= divisions*4)

    %IK
    [theta1, theta2, theta3, theta4] = InverseKinematics(x(b), y(b), z(b), deg2rad(0));
  

    dh_theta2 = theta2 - gamma + pi/2;
    dh_theta3 = theta3 + gamma - pi/2;
    
    %fk for links
    T0_1 = Transform(alpha1, a1, d1, theta1);
    T1_2 = Transform(alpha2, a2, d2, dh_theta2);
    T2_3 = Transform(alpha3, a3, d3, dh_theta3);
    T3_4 = Transform(alpha4, a4, d4, theta4);
    
    T0_4 = T0_1 * T1_2 * T2_3 * T3_4;
    T0_3 = T0_1 * T1_2 * T2_3;
    T0_2 = T0_1 * T1_2;
    
    link1end = T0_1(1:3,4);
    link2end = T0_2(1:3,4);
    link3end = T0_3(1:3,4);
    end_effector_pos = T0_4(1:3,4);
    end_effector_orientation = T0_4(1:3,1:3);
    


    %remove robot from previous iteration
    if b ~=1
        delete(link1)
        delete(link2)
        delete(link3)
        delete(link4)
        delete(frame1)
        delete(frame2)
        delete(frame3)
        delete(frame4)
    end
    
    %plot robot links
    link1 = drawLink([0,0,0], link1end, 2, 'black');
    link2 = drawLink(link1end, link2end, 2, 'black');
    link3 = drawLink(link2end, link3end, 2, 'black');
    link4 = drawLink(link3end, end_effector_pos, 2, 'black');

    %plot coordinate frames
    frame1 = plotCoordFrame(T0_1);
    frame2 = plotCoordFrame(T0_2);
    frame3 = plotCoordFrame(T0_3);
    frame4 = plotCoordFrame(T0_4);
    
    %draw square
%     if b == 1
%         drawLink([x(1),y(1),z(1)], [x(b),y(b),z(b)],2,'magenta');
%     else
%         drawLink([x(b-1),y(b-1),z(b-1)], [x(b),y(b),z(b)],2,'magenta');
%     end
    scatter3(end_effector_pos(1), end_effector_pos(2), end_effector_pos(3), 'MarkerEdgeColor', [0 .75 .75], 'MarkerFaceColor',[0 .75 .75], 'Marker', 'o');

    view(3);
    %view ([0 -90 0]); %X Z Plane
    %view ([0 0 -90 ]); 
    b = b + 1;
    pause(0.1)   
end





function T = Transform(alpha, a, d, theta)
    
T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*cos(alpha) a*sin(theta);
    0 sin(alpha) cos(alpha) d;
    0 0 0 1];
end


function frame = plotCoordFrame(transform)

    x_dir = transform(1:3,1)*0.015;
    y_dir = transform(1:3,2)*0.015;

    z_dir = cross(x_dir,y_dir) *70;
    start = transform(:,4);

    frame(1) = plot3([start(1) start(1)+x_dir(1)],[start(2) start(2)+x_dir(2)],[start(3) start(3)+x_dir(3)],'LineWidth',2,'Color','red');
    frame(2) = plot3([start(1) start(1)+y_dir(1)],[start(2) start(2)+y_dir(2)],[start(3) start(3)+y_dir(3)],'LineWidth',2,'Color','green');
    frame(3) = plot3([start(1) start(1)+z_dir(1)],[start(2) start(2)+z_dir(2)],[start(3) start(3)+z_dir(3)],'LineWidth',2,'Color','blue');
end

function link = drawLink (start, endpoint, width, colour)
    link = line([start(1) endpoint(1)],[start(2) endpoint(2)],[start(3) endpoint(3)],'LineWidth',width,'Color',colour);
end







