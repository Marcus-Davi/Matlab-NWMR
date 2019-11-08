
function [vd,ve] = motorGet(serial)
fprintf(serial,'M?');
vels = fscanf(serial,'%f %f');
vd = vels(1);
ve = vels(2);
end