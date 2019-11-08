function [vd,ve] = vw2rpm(v,w)
D = 0.380; %0.405
R = 0.17/2;

vd_rad = (v+w*D);
ve_rad = (v-w*D);
vd = (vd_rad*60/(2*pi*R));
ve = (ve_rad*60/(2*pi*R));
    
    
end