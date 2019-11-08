function calibrateGPS(b)
Ts = 0.1;
iterations = 30;
load('GPSOrigin')
LATLONG = [];
for i=1:iterations
    tic
    
    S = sensorGet(b);
    latlong = [S(12) S(13)]
    
    [x,y] = getGPSxy(latlong,GPSOrigin);
    x
    y
    
    LATLONG = [LATLONG;latlong];
    toc; 
    delta = abs(Ts-toc);
    if delta < Ts
    pause(delta);
    end %sync end
end

GPSOrigin = mean(LATLONG)
save('GPSOrigin','GPSOrigin')
    

end