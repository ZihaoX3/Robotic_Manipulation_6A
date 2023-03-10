function pointsList = task2acoordlist(start, endpoint, safe_dist, thetaG, cube_grab_z, open_value, close_value)

    pointsList = [];
    pointsList = [pointsList; [start, start, safe_dist, thetaG, open_value]];
    % Grab cube
    pointsList = [pointsList; [start, start, cube_grab_z, thetaG, close_value]];
    %pick cube up
    pointsList = [pointsList; [start, start, safe_dist, thetaG, close_value]];
    %move to end point
    pointsList = [pointsList; [endpoint, endpoint, safe_dist, thetaG, close_value]];
    %put cube down
    pointsList = [pointsList; [endpoint, endpoint, cube_grab_z, thetaG, open_value]];
    %move back to safe distance
    pointsList = [pointsList; [endpoint, endpoint, safe_dist, thetaG, open_value]];

%     pointsList = [];
%     % Go to safe distance above cube 
%     point = [start, start, safe_dist, thetaG, open_value];
% 
% %     jointLimitsOk = withinJointLimits(point);
% %     if ~jointLimitsOk
% %         point(4) = thetaG;
% %         thetaG = thetaG;
% %     end
%     
%     pointsList = [pointsList; point];
% 
%     % Grab cube
%     point = [start, start, cube_grab_z, thetaG, close_value];
% 
% %     jointLimitsOk = withinJointLimits(point);
% %     if ~jointLimitsOk
% %         point(4) = thetaG;
% %         thetaG = thetaG;
% %         point(3) = cube_grab_z;
% %     end
%     pointsList = [pointsList; point];
% 
%     %pick cube up
%     point = [start, start, safe_dist, thetaG, close_value];
% 
% %     jointLimitsOk = withinJointLimits(point);
% %     if ~jointLimitsOk
% %         point(4) = thetaG;
% %         thetaG = thetaG;
% %     end
%     pointsList = [pointsList; point];
% 
%     
%     %move to end point
%     point = [endpoint, endpoint, safe_dist, thetaG, close_value];
% %     jointLimitsOk = withinJointLimits(point);
% %     if ~jointLimitsOk
% %         point(4) = thetaG;
% %         thetaG = thetaG;
% %     end
%     pointsList = [pointsList; point];
% 
% 
%     %put cube down
%     point = [endpoint, endpoint, cube_grab_z, thetaG, open_value];
% %     jointLimitsOk = withinJointLimits(point);
% %     if ~jointLimitsOk
% %         point(4) = thetaG;
% %         thetaG = thetaG;
% %         point(3) = cube_grab_z;
% %     end
%     pointsList = [pointsList; point];
% 
% 
%     %move back to safe distance
%     point = [endpoint, endpoint, safe_dist, thetaG, open_value];
%     pointsList = [pointsList; point];
end