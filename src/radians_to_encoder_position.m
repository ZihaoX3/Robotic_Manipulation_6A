function encoder = radians_to_encoder_position(rad_angle)
    proportion = rad_angle/(2*pi);
    encoder = uint32(proportion * 4096 + 2048);% add 2048 so that angle = 0 is at half encoder range
end
