
function motorGo(serial,vd,ve)
vd_str = num2str(vd);
ve_str = num2str(ve);
string = strcat('M!'," ",vd_str,',',ve_str);
fprintf(serial,string);

end