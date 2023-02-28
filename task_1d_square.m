clc
%clear all
close all

constant = - atan2(0.024,0.128) + pi/2;% offset angle between link 2 and 3

theta1 = deg2rad(0);
theta2 = constant + deg2rad(0);
theta3 = - constant + deg2rad(0);
theta4 = deg2rad(0);

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
xlim([-0.2 0.2])
ylim([-0.2 0.2])
zlim([0 0.4])


divisions = 20;
%10cm square in x-y plane
x = [linspace(0.0, 0.1, divisions), zeros(1,divisions)+0.1, linspace(0.1-0.1/divisions, 0.0, divisions), zeros(1,divisions)];
y = [zeros(1,divisions)+0.1 , linspace(0.1+0.1/divisions, 0.2, divisions), zeros(1,divisions)+0.2, linspace(0.2-0.1/divisions, 0.1, divisions)];
z = zeros(1,divisions*4)+0.2;

% %10cm square in z-x plane
% x = [linspace(0.0, 0.1, divisions), zeros(1,divisions)+0.1, linspace(0.1-0.1/divisions, 0.0, divisions), zeros(1,divisions)];
% z = [zeros(1,divisions)+0.1 , linspace(0.1+0.1/divisions, 0.2, divisions), zeros(1,divisions)+0.2, linspace(0.2-0.1/divisions, 0.1, divisions)];
% y = zeros(1,divisions*4)+0.2;

% %10cm square in z-y plane
% y = [linspace(0.0, 0.1, divisions), zeros(1,divisions)+0.1, linspace(0.1-0.1/divisions, 0.0, divisions), zeros(1,divisions)];
% z = [zeros(1,divisions)+0.1 , linspace(0.1+0.1/divisions, 0.2, divisions), zeros(1,divisions)+0.2, linspace(0.2-0.1/divisions, 0.1, divisions)];
% x = zeros(1,divisions*4)+0.2;

b=1;
while (b <= divisions*4)

    %IK
    [theta1, theta2, theta3, theta4] = InverseKinematics(x(b), y(b), z(b), deg2rad(0));
  
    theta2 = constant + theta2;
    theta3 = - constant + theta3;
    
    %fk for links
    T0_1 = Transform(alpha1, a1, d1, theta1);
    T1_2 = Transform(alpha2, a2, d2, theta2);
    T2_3 = Transform(alpha3, a3, d3, theta3);
    T3_4 = Transform(alpha4, a4, d4, theta4);
    
    T0_4 = T0_1 * T1_2 * T2_3 * T3_4;
    T0_3 = T0_1 * T1_2 * T2_3;
    T0_2 = T0_1 * T1_2;
    
    link1end = T0_1(1:3,4);
    link2end = T0_2(1:3,4);
    link3end = T0_3(1:3,4);
    end_effector_pos = T0_4(1:3,4);
    end_effector_orientation = T0_4(1:3,1:3);
    
%     %plot coordinate frames
%     frame = plotCoordFrame(T0_1, link1end);
%     frame1 =plotCoordFrame(T0_2, link2end);
%     frame2 =plotCoordFrame(T0_3, link3end);
%     frame3 =plotCoordFrame(T0_4, end_effector_pos);

    %remove robot from previous iteration
    if b ~=1
        delete(link1)
        delete(link2)
        delete(link3)
        delete(link4)
%         delete(frame)
%         delete(frame1)
%         delete(frame2)
%         delete(frame3)
    end
    
    %plot robot links
    link1 = drawLink([0,0,0], link1end, 2, 'black');
    link2 = drawLink(link1end, link2end, 2, 'black');
    link3 = drawLink(link2end, link3end, 2, 'black');
    link4 = drawLink(link3end, end_effector_pos, 2, 'black');
    
    %draw square
    if b == 1
        drawLink([x(1),y(1),z(1)], [x(b),y(b),z(b)],2,'magenta')
    else
        drawLink([x(b-1),y(b-1),z(b-1)], [x(b),y(b),z(b)],2,'magenta')
    end

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




function R = RotationZ(theta)
 R = [   cos(theta)                       -sin(theta)               0                  0;
                sin(theta)                   cos(theta)             0                  0;
                0                                 0                 1                  0;
                0                                 0                 0                  1
         ];
end

function frame = plotCoordFrame(transform, start)

%     X = [  [1,0,0,0.02];
%             [0,1,0,0]; 
%             [0,0,1,0];
%             [0, 0, 0,1]];
% 
%     Y = [[1,0,0,0];
%           [0,1,0,0.02];
%           [0,0,1,0];
%           [0, 0, 0,1]];
% 
%     Z = [[1,0,0,0];
%           [0,1,0,0]; 
%           [0,0,1,0.02];
%           [0, 0, 0,1]];
% 
%     Xcoord = transform * X;
%     Xcoord = Xcoord(1:3, 4);
% 
%     Ycoord = transform * Y;
%     Ycoord = Ycoord(1:3, 4);
% 
%     Zcoord = transform * Z;
%     Zcoord = Zcoord(1:3, 4); 
%     
%     drawLink(start,Xcoord,1,'red')
%     drawLink(start,Ycoord,1,'blue')
%     drawLink(start,Zcoord,1,'green')


    x_dir = transform(1:3,1)*0.03;
    y_dir = transform(1:3,2)*0.03;
    z_dir = transform(1:3,3)*0.03;
    start = transform(:,4);

%     line([start(1) start(1)+x_dir(1)],[start(2) start(2)+x_dir(2)],[start(3) start(3)+x_dir(3)],'LineWidth',1,'Color','red');
%     line([start(1) start(1)+y_dir(1)],[start(2) start(2)+y_dir(2)],[start(3) start(3)+y_dir(3)],'LineWidth',1,'Color','blue');
%     line([start(1) start(1)+z_dir(1)],[start(2) start(2)+z_dir(2)],[start(3) start(3)+z_dir(3)],'LineWidth',1,'Color','green');
    frame_x = plot3([start(1) start(1)+x_dir(1)],[start(2) start(2)+x_dir(2)],[start(3) start(3)+x_dir(3)],'LineWidth',1,'Color','red');
    frame_y = plot3([start(1) start(1)+y_dir(1)],[start(2) start(2)+y_dir(2)],[start(3) start(3)+y_dir(3)],'LineWidth',1,'Color','blue');
    frame_z = plot3([start(1) start(1)+z_dir(1)],[start(2) start(2)+z_dir(2)],[start(3) start(3)+z_dir(3)],'LineWidth',1,'Color','green');

    frame = [frame_x, frame_y, frame_z];



end

function link = drawLink (start, endpoint, width, colour)
    link = line([start(1) endpoint(1)],[start(2) endpoint(2)],[start(3) endpoint(3)],'LineWidth',width,'Color',colour);
end







