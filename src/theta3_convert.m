function encoder = theta3_convert(rad_angle)
     encoder = 2048*(rad_angle/(pi));
     encoder = 3096-encoder ;
end