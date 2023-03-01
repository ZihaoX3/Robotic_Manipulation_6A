function encoder = theta4_convert(rad_angle)
     encoder = 2048*(rad_angle/(pi));
     encoder = 2048+encoder;
end