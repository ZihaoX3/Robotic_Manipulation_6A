function encoder = radians_to_encoder_position(rad_angle)
    proportion = rad_angle/(2*pi);
    encoder = int32(proportion * 4096);
end