
function sensor = sensorGet(serial)
fprintf(serial,'S?');
sensor = fscanf(serial,'%d %d %d %d %d %d %d %d %d %f %f %f %f');
end