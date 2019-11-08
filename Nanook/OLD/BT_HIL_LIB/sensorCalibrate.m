
function Sensor = sensorCalibrate(SensorRead,calibrationAccGyr,calibrationMag)
Sensor = SensorRead;
Sensor(1:6) = Sensor(1:6) - calibrationAccGyr;
Sensor(7:9) = Sensor(7:9) - calibrationMag;
end