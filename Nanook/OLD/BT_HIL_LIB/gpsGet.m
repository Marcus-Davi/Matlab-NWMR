
function [lat,long] = gpsGet(serial)
fprintf(serial,'G?');
vels = fscanf(serial,'%f %f');
lat = vels(1);
long = vels(2);
end