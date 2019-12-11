
function [v,w] = speedGet(sensors_data)
    data = sensors_data.Data;
    data = sscanf(data,'%d %d %d %d %d %d %d %d %d %f %f %f %f');   
    vd = data(10);
    ve = data(11);
[v,w] = rpm2vw(vd,ve);
end