function encoder = radians_encoder(rad_angle)
    proportion = rad_angle/(2*pi);
    encoder = int32(proportion * 4096);
end