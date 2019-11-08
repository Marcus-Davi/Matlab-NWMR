%% Estabelecimento da Conexao Bluetooth
if exist('b')
    fclose(b);
end
clear all;close all;clc
disp('Conectando ao Bluetooth...');
%b = Bluetooth('HulkBluetooth',1,'Terminator','CR'); %NOME DO DISPOSITIVO BLUETOOTH
b = serial('/dev/rfcomm0','BaudRate',115200,'Terminator','CR');
fopen(b);
disp('Conectado.')
XPOINTS = 6000;
position = 1;
time = 1;
Ts = 0.1;
x = [(1:XPOINTS)'];
xlabels = (1:XPOINTS);
y = zeros(XPOINTS,8+6);
KGyro = 15.625e-3;
%%
flushinput(b);
while(1)
    
    while(b.BytesAvailable==0)
    end
    
    %read = fscanf(b,'Ax:%f,Ay:%f,Vx:%f,Vy:%f,x_:%f,y_:%f,Px:%f,Py:%f\n\r') %FORMATO DOS DADOS A RECEBER
    %read = fscanf(b,'T:%f,Ax:%f,Ay:%f,RAx:%f,RAy:%f\n\r') %FORMATO DOS DADOS A RECEBER
    %read = fscanf(b,'Gc:%fG:%f\n\r');
%     read = fscanf(b,'E:%f,%f,%f,P:%f,%f,%f,V:%f,%f\n\r'); %FORMATO DO CONTROLADOR LQ 20.12.17
read = fscanf(b,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n\r'); %FORMATO DO CONTROLADOR LQ 20.12.17
    xlabels(position) = time;
    time = time + 1;
    if (position < XPOINTS)
        position = position + 1;
    else
        position = 1;
    end
    
    
    
    y(position,1) = (read(1)); %agmx
    y(position,2) = (read(2)); %agmy
    y(position,3) = (read(3)); %theta
    
    y(position,4) = (read(4)); %gmx
    y(position,5) = (read(5)); %gmy
    
    y(position,6) = (read(6)); %ox
    y(position,7) = (read(7)); %oy
    y(position,8) = (read(8)); %ot
    
    y(position,9) = (read(9)); %lx
    y(position,10) = (read(10)); %ly
    y(position,11) = (read(11)); %lt
    
    y(position,12) = (read(12)); %ex
    y(position,13) = (read(13)); %ey
    y(position,14) = (read(14)); %et
    
    y(position,15) = (read(15)); %vx
    y(position,16) = (read(16)); %vy
    drawnow
    plot(y(1:position,1),y(1:position,2));
    
end
%% PLOTS (OPTIONAL)
close all;
% position = position - 1;
Time = 0:Ts:(time-1)*Ts;
plot(y(1:position,1),y(1:position,2));
grid;
xlabel('X[m]');
ylabel('Y[m]');
hold on
plot(y(1:position,4),y(1:position,5));
plot(y(1:position,6),y(1:position,7));
plot(y(1:position,9),y(1:position,10));

legend({'Fusion Robot$_{AGM}$','Fusion Robot$_{GM}$','Odometry Robot','LiDAR Robot'},'interpreter','latex')
figure;hold on;grid on;
plot(y(1:position,3))
plot(y(1:position,11))
legend({'Fusion $\theta$','LiDAR $\theta$'},'interpreter','latex')
figure;hold on; grid on;
plot(Time,y(1:position,12));
plot(Time,y(1:position,13));
plot(Time,y(1:position,14));
LEG = legend({'$x$ Error[m]','$y$ Error[m]','$\theta$ Error[rad]'},'Location','northeast','Interpreter','latex');
LEG.FontSize = 11;
figure;hold on; grid on;
subplot(2,1,1)
plot(Time,y(1:position,15),'b');
grid on;
LEG = legend({'$v$'},'Location','northeast','interpreter','latex');
LEG.FontSize = 11;
subplot(2,1,2)
plot(Time,y(1:position,16),'r');
grid on;
legend({'$\omega$'},'Location','northeast','interpreter','latex');
LEG = legend({'$\omega$'},'Location','northeast','interpreter','latex');
LEG.FontSize = 11;
% plot(1:position,y(1:position,11));
% plot(1:position,y(1:position,12));