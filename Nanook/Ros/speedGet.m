
function [v,w] = speedGet(sensors)
    sens = receive(sensors);
    data = sens.Data;
    data = sscanf(data,'%d %d %d %d %d %d %d %d %d %f %f %f %f');   
    vd = data(10);
    ve = data(11);
[v,w] = rpm2vw(vd,ve);
end