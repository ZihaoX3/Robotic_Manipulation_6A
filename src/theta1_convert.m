function encoder = theta1_convert(rad_angle)
%     proportion = rad_angle/(2*pi);
%     encoder = int32(proportion * 4096 + 2048);% add 2048 so that angle = 0 is at 
%    RAD_IN_ENCODE = (2*pi) / 4096;
%    encoder = int32(double(rad_angle) / RAD_IN_ENCODE + 2048);

     encoder = 2048*(rad_angle/(pi));
     encoder = encoder + 1024;
     %encoder = rem(encoder,3096);
end
