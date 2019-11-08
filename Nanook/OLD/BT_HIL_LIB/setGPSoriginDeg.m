
function setGPSoriginRad(lat,long)
GPSOrigin = deg2rad([lat long]);
save('GPSOrigin','GPSOrigin')
end