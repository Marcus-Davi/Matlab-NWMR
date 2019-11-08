%% Bluetooth Interface
close all;clc
if exist('b')
    fclose(b);
end
clear
b = bluetoothInit;

%% Loop
while true
    tic;
%     M = motorGet(b);
%     I = inertialGet(b);
S = sensorGet(b);
    toc;
end