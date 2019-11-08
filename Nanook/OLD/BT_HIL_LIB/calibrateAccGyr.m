%%

function calibrateAccGyr(b,samples)
Avg = [];

for i=1:samples
    S = sensorGet(b)'
    Avg = [Avg;S(1:6)];
end

CalibrationAccGyr = mean(Avg)';
CalibrationAccGyr(3) = CalibrationAccGyr(3) - 255; %grav
save('CalibrationAccGyr','CalibrationAccGyr');





