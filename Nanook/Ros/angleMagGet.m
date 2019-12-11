
function angle_cal = angleMagGet(sensors_data,offsets)
    data = sensors_data.Data;
    data = sscanf(data,'%d %d %d %d %d %d %d %d %d %f %f %f %f');   
    Mag.x = data(7);
    Mag.y = data(8);
   angle_cal = atan2(-(Mag.y-offsets.y),(Mag.x-offsets.x));
end