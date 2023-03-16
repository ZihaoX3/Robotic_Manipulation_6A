% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates


clc;
clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% ---- Control Table Addresses ---- %%

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;
ADDR_PRO_VELOCITY            = 112; 
ADDR_PRO_PROFILE_ACCELERATION = 108;
ADDR_PRO_PROFILE_VELOCITY     = 112;


%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID1_BASE                     = 11;            % Dynamixel ID: 1
DXL_ID2_SHOULDER                 = 12;            % Dynamixel ID: 2
DXL_ID3_ELBOW                    = 13;            % Dynamixel ID: 3
DXL_ID4_WRIST                    = 14;            % Dynamixel ID: 4
DXL_ID5_GRIPPER                  = 15;            % Dynamixel ID: 5
BAUDRATE                    = 115200;
DEVICENAME                  = 'COM9';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% ------------------ %%

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position

% ----- SET MOTION LIMITS ----------- %
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
MAX_POS = 3400;
MIN_POS = 600;
% Set max position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BASE, ADDR_MAX_POS, MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_SHOULDER, ADDR_MAX_POS, MAX_POS);
% Set min position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BASE, ADDR_MIN_POS, MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_SHOULDER, ADDR_MIN_POS, MIN_POS);
% ---------------------------------- %

% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Put actuator into Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BASE, ADDR_PRO_OPERATING_MODE, 3);




% Enable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BASE, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_SHOULDER, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3_ELBOW, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4_WRIST, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5_GRIPPER, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);


% Set velocity and acceleration
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BASE, ADDR_PRO_PROFILE_VELOCITY, 30);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_SHOULDER, ADDR_PRO_PROFILE_VELOCITY, 30);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3_ELBOW, ADDR_PRO_PROFILE_VELOCITY, 30);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4_WRIST, ADDR_PRO_PROFILE_VELOCITY, 30);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5_GRIPPER, ADDR_PRO_PROFILE_VELOCITY, 30);

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BASE, ADDR_PRO_PROFILE_ACCELERATION, 30);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_SHOULDER, ADDR_PRO_PROFILE_ACCELERATION, 30);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3_ELBOW, ADDR_PRO_PROFILE_ACCELERATION, 30);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4_WRIST, ADDR_PRO_PROFILE_ACCELERATION, 30);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5_GRIPPER, ADDR_PRO_PROFILE_ACCELERATION, 30);


dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% i = 0;


%     j = 0;
%     while (j<200)
%         j = j+1;
%         
%         % Read present position
%         dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BASE, ADDR_PRO_PRESENT_POSITION);
%         dx2_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_SHOULDER, ADDR_PRO_PRESENT_POSITION);
%         dx3_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3_ELBOW, ADDR_PRO_PRESENT_POSITION);
%         dx4_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4_WRIST, ADDR_PRO_PRESENT_POSITION);
%         dx5_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5_GRIPPER, ADDR_PRO_PRESENT_POSITION);
%         
%         dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
%         dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
%         
%         if dxl_comm_result ~= COMM_SUCCESS
%             fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
%         elseif dxl_error ~= 0
%             fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
%         end

%         fprintf('[ID:%03d] Base Angle: %03d\n', DXL_ID1_BASE, typecast(uint32(encoder_position_to_radians(dxl_present_position)), 'int32'));
%         fprintf('[ID:%03d] Shoulder Angle: %03d\n', DXL_ID2_SHOULDER, typecast(uint32(encoder_position_to_radians(dx2_present_position)), 'int32'));
%         fprintf('[ID:%03d] Elbow Angle: %03d\n', DXL_ID3_ELBOW, typecast(uint32(encoder_position_to_radians(dx3_present_position)), 'int32'));
%         fprintf('[ID:%03d] Wrist Angle: %03d\n', DXL_ID4_WRIST, typecast(uint32(encoder_position_to_radians(dx4_present_position)), 'int32'));
%         fprintf('[ID:%03d] Gripper Angle: %03d\n', DXL_ID5_GRIPPER, typecast(uint32(encoder_position_to_radians(dx5_present_position)), 'int32'));

%         if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
%             break
%         end
   



%% TASK2b Test %%
 [theta1_list, theta2_list, theta3_list, theta4_list, gripperList] = task3robot_de();
%     [theta1_list, theta2_list, theta3_list, theta4_list, gripperList] =  task3robot();

    for i=1:size(theta1_list,2) 
    
        theta1 = theta1_list(i);
        theta2 = theta2_list(i);
        theta3 = theta3_list(i);
        theta4 = theta4_list(i);
        gripper_angle = gripperList(i);
    
        theta1_arm = radians_to_encoder_position(theta1);%flip x axis since i defined coords wrong...(we are facing robot)
        theta2_arm = radians_to_encoder_position(-theta2);
        theta3_arm = radians_to_encoder_position(-theta3);
        theta4_arm = radians_to_encoder_position(-theta4);
        
        
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5_GRIPPER, ADDR_PRO_GOAL_POSITION, gripper_angle);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4_WRIST, ADDR_PRO_GOAL_POSITION, theta4_arm);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BASE, ADDR_PRO_GOAL_POSITION, theta1_arm);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_SHOULDER, ADDR_PRO_GOAL_POSITION, theta2_arm);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3_ELBOW, ADDR_PRO_GOAL_POSITION, theta3_arm);
      
        pause(0.01)%0.007
    end      



%% Set one position %%
% %       %Default Pose 0.2740,0.0000,0.2048 225,0,40,0,deg2rad(212)
%         [theta1_rad, theta2_rad, theta3_rad, theta4_rad] = InverseKinematics(0.274,0,0.2048, deg2rad(0));
%         
%         %our angles are defined as left positive, robot is right positive i think
% %         theta1 = radians_to_encoder_position(theta1_rad);
%         theta2 = radians_to_encoder_position(-theta2_rad);
%         theta3 = radians_to_encoder_position(-theta3_rad);
%         theta4 = radians_to_encoder_position(-theta4_rad);
%         theta1 = theta1_convert(theta1_rad);
%         %write position
% 
%         write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BASE, ADDR_PRO_GOAL_POSITION, theta1);
%         write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_SHOULDER, ADDR_PRO_GOAL_POSITION, theta2);
%         write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3_ELBOW, ADDR_PRO_GOAL_POSITION, theta3);
%         write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4_WRIST, ADDR_PRO_GOAL_POSITION, theta4);
% %     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5_GRIPPER, ADDR_PRO_GOAL_POSITION, theta5);
%         pause(50) 

%     end


% Disable Dynamixel Torque
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BASE, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
% setTorqueAll(port_num, PROTOCOL_VERSION, DXL_ID1_BASE, ADDR_PRO_TORQUE_ENABLE,TORQUE_DISABLE);
%     write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BASE, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
%     write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_SHOULDER, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
%     write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3_ELBOW, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
%     write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4_WRIST, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
%     write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5_GRIPPER, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;
