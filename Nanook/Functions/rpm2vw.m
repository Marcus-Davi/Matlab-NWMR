function [v,w] = rpm2vw(vd,ve)
D = 0.405; %0.405
R = 0.222/2; %"calibrado" via magnetometro

vd_lin = vd*2*pi/60*R;
ve_lin = ve*2*pi/60*R;


v = 0.5*(vd_lin+ve_lin);
w = 0.5*(vd_lin-ve_lin)/D;
end