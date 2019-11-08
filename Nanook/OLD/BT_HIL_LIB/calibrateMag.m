%%

function CalibrationMag = calibrateMag(b)
Avg = [];
motorGo(b,15,-15); %1 volta a cada 15 segundos
time = 20;
Ts = 0.1;
samples = time/Ts;
for i=1:samples
    tic
    S = sensorGet(b)';
    S(7:9)
    Avg = [Avg;S(7:9)]; %mag
    
    delta = abs(Ts-toc);
    if delta < Ts
    pause(delta);
    end 
end
motorGo(b,0,0)
Max_X = max(Avg(:,1))
Min_X = min(Avg(:,1))
Max_Y = max(Avg(:,2))
Min_Y = min(Avg(:,2))
Max_Z = max(Avg(:,3))
Min_Z = min(Avg(:,3))

CalibrationMag = [(Max_X + Min_X)/2 (Max_Y + Min_Y)/2 (Max_Z + Min_Z)/2]'
save('CalibrationMag','CalibrationMag');
%% Plots
close all
plot(Avg(:,1),Avg(:,2)); 
hold on;
Avg_cal = Avg - CalibrationMag';
plot(Avg_cal(:,1),Avg_cal(:,2)); 
grid on






