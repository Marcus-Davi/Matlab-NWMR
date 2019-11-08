
function [x,y] = getGPSxy(latlong,latlong_origin)
R = 6378137.0;
x = R*(latlong(2) - latlong_origin(2))*cos(latlong(1));
y = R*(latlong(1)-latlong_origin(1));
end