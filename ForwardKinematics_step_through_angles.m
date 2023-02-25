clc
%clear all
close all

%syms theta1 theta2 theta3 theta4

theta1 = 0;
theta2 = - atan2(0.024,0.128) + pi/2;
theta3 = - pi/2 + atan2(0.024,0.128);
theta4 = 0;

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

b=0;
while (b <= 5)
    
    %fk for links
    T0_1 = Transform(alpha1, a1, d1, theta1);
    T1_2 = Transform(alpha2, a2, d2, theta2);
    T2_3 = Transform(alpha3, a3, d3, theta3);
    T3_4 = Transform(alpha4, a4, d4, theta4);
    
    T0_4 = T0_1 * T1_2 * T2_3 * T3_4;
    T0_3 = T0_1 * T1_2 * T2_3;
    T0_2 = T0_1 * T1_2;
    % simplify(T0_4)
    
    link1end = T0_1(1:3,4);
    link2end = T0_2(1:3,4);
    link3end = T0_3(1:3,4);
    end_effector_pos = T0_4(1:3,4);
    end_effector_orientation = T0_4(1:3,1:3);
    


    %frame 1
    X1 = [[eye(3);[0,0,0]] [0.02,0,0,1]'];
    Y1 = [[eye(3);[0,0,0]] [0,0.02,0,1]'];
    Z1 = [[T0_1(1:3,1:3);[0,0,0]] [0,0,0.02,1]'];
    
    X1trans = T0_1 * X1;
    X1coord = X1trans(1:3, 4);
    
    Y1trans = T0_1 * Y1;
    Y1coord = Y1trans(1:3, 4);
    
    Z1trans = T0_1 * Z1;
    Z1coord = Z1trans(1:3, 4);

   
    %frame 2 
    X2 = [[eye(3);[0,0,0]] [0.02,0,0,1]'];
    Y2 = [[eye(3);[0,0,0]] [0,0.02,0,1]'];
    Z2 = [[eye(3);[0,0,0]] [0,0,0.02,1]'];
    
    Xtrans = T0_2 * X2;
    Xcoord = Xtrans(1:3, 4);
    
    Ytrans = T0_2 * Y2;
    Ycoord = Ytrans(1:3, 4);
    
    Ztrans = T0_2 * Z2;
    Zcoord = Ztrans(1:3, 4);

    %frame 3
    X3 = [[eye(3);[0,0,0]] [0.02,0,0,1]'];
    Y3 = [[eye(3);[0,0,0]] [0,0.02,0,1]'];
    Z3 = [[eye(3);[0,0,0]] [0,0,0.02,1]'];
    
    X3trans = T0_3 * X3;
    X3coord = X3trans(1:3, 4);
    
    Y3trans = T0_3 * Y3;
    Y3coord = Y3trans(1:3, 4);
    
    Z3trans = T0_3 * Z3;
    Z3coord = Z3trans(1:3, 4);

    %frame 4
    X4 = [[eye(3);[0,0,0]] [0.02,0,0,1]'];
    Y4 = [[eye(3);[0,0,0]] [0,0.02,0,1]'];
    Z4 = [[eye(3);[0,0,0]] [0,0,0.02,1]'];
    
    X4trans = T0_4 * X4;
    X4coord = X4trans(1:3, 4);
    
    Y4trans = T0_4 * Y4;
    Y4coord = Y4trans(1:3, 4);
    
    Z4trans = T0_4 * Z4;
    Z4coord = Z4trans(1:3, 4);

    plot3(0,0,0)


    %frame 0
    X0 = [  [1,0,0,0.02];
            [0,1,0,0]; 
            [0,0,1,0];
            [0, 0, 0,1]];

    Y0 = [[1,0,0,0];
          [0,1,0,0.02];
          [0,0,1,0];
          [0, 0, 0,1]];

    Z0 = [[1,0,0,0];
          [0,1,0,0]; 
          [0,0,1,0.02];
          [0, 0, 0,1]];

    frame_0_X = RotationZ(theta1)*X0;
    frame_0_X = frame_0_X(1:3, 4);
    frame_0_Y = RotationZ(theta1)*Y0;
    frame_0_Y = frame_0_Y(1:3, 4);
    frame_0_Z = RotationZ(theta1)*Z0;
    frame_0_Z = frame_0_Z(1:3, 4);
    

    %plot frames%

    %plot origin frame
    line([0 frame_0_X(1)], [0 frame_0_X(2)], [0 frame_0_X(3)],'LineWidth',1,'Color','red');
    line([0 frame_0_Y(1)], [0 frame_0_Y(2)], [0 frame_0_Y(3)],'LineWidth',1,'Color','blue');
    line([0 frame_0_Z(1)], [0 frame_0_Z(2)], [0 frame_0_Z(3)],'LineWidth',3,'Color','green');

    %plot frame 1
    line([link1end(1) X1coord(1)],[link1end(2) X1coord(2)],[link1end(3) X1coord(3)],'LineWidth',1,'Color','red');
    line([link1end(1) Y1coord(1)],[link1end(2) Y1coord(2)],[link1end(3) Y1coord(3)],'LineWidth',3,'Color','green');
    line([link1end(1) Z1coord(1)],[link1end(2) Z1coord(2)],[link1end(3) Z1coord(3)],'LineWidth',1,'Color','blue');
    
    %plot frame 2
    line([link2end(1) Xcoord(1)],[link2end(2) Xcoord(2)],[link2end(3) Xcoord(3)],'LineWidth',1,'Color','red');
    line([link2end(1) Ycoord(1)],[link2end(2) Ycoord(2)],[link2end(3) Ycoord(3)],'LineWidth',1,'Color','green');
    line([link2end(1) Zcoord(1)],[link2end(2) Zcoord(2)],[link2end(3) Zcoord(3)],'LineWidth',1,'Color','blue');

    %plot frame 3
    line([link3end(1) X3coord(1)],[link3end(2) X3coord(2)],[link3end(3) X3coord(3)],'LineWidth',1,'Color','red');
    line([link3end(1) Y3coord(1)],[link3end(2) Y3coord(2)],[link3end(3) Y3coord(3)],'LineWidth',1,'Color','green');
    line([link3end(1) Z3coord(1)],[link3end(2) Z3coord(2)],[link3end(3) Z3coord(3)],'LineWidth',1,'Color','blue');

    %plot frame 4
    line([end_effector_pos(1) X4coord(1)],[end_effector_pos(2) X4coord(2)],[end_effector_pos(3) X4coord(3)],'LineWidth',1,'Color','red');
    line([end_effector_pos(1) Y4coord(1)],[end_effector_pos(2) Y4coord(2)],[end_effector_pos(3) Y4coord(3)],'LineWidth',1,'Color','green');
    line([end_effector_pos(1) Z4coord(1)],[end_effector_pos(2) Z4coord(2)],[end_effector_pos(3) Z4coord(3)],'LineWidth',1,'Color','blue');

    %plot robot links
    line([0 link1end(1)],[0 link1end(2)],[0 link1end(3)],'LineWidth',2,'Color','black');%link 1
    line([link1end(1) link2end(1)], [link1end(2) link2end(2)], [link1end(3) link2end(3)], 'LineWidth',2,'Color','black');%link 2
    line([link2end(1) link3end(1)], [link2end(2) link3end(2)], [link2end(3) link3end(3)], 'LineWidth',2,'Color','black'); %link 3
    line([link3end(1) end_effector_pos(1)], [link3end(2) end_effector_pos(2)], [link3end(3) end_effector_pos(3)], 'LineWidth',2,'Color','black');%link 4

    xlabel('x');
    ylabel('y');
    zlabel('z');
    axis('equal');
    grid on;
    
    xlim([-0.1 0.3]); 
    ylim([-0.1 0.3]);
    zlim([0 0.5]);
    
    view(3);
    %view ([0 -90 0]); %X Z Plane
    %view ([0 0 -90 ]); 
    b = b+1;
    theta3 = theta3 + pi/16;
    theta4 = theta4 + pi/16;
    %theta1 = theta1 - pi/4;
    theta2 = theta2 + pi/64;


    pause(1.5)
    
end
% close all
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
                0                                 0                 0                         1
         ];
end








