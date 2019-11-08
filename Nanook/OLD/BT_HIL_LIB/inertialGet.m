
function inertial = inertialGet(serial)
fprintf(serial,'I?');
inertial = fscanf(serial,'%d %d %d %d %d %d %d %d %d');
end