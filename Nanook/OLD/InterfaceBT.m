%% Estabelecimento da Conexao Bluetooth
if exist('b')
    fclose(b);
end
clear;close all;clc
disp('Conectando ao Bluetooth...');
%b = Bluetooth('HulkBluetooth',1,'Terminator','CR'); %NOME DO DISPOSITIVO BLUETOOTH
b = serial('/dev/rfcomm0','BaudRate',115200,'Terminator','CR');
fopen(b);
disp('Conectado.')
XPOINTS = 6000;
position = 1;
time = 1;
Ts = 0.1;
x = (1:XPOINTS)';
xlabels = (1:XPOINTS);
y = zeros(XPOINTS,8);
%%
flushinput(b);
fprintf(b,'T!');
while(1)
    
    while(b.BytesAvailable==0)
    end
    
    %read = fscanf(b,'Ax:%f,Ay:%f,Vx:%f,Vy:%f,x_:%f,y_:%f,Px:%f,Py:%f\n\r') %FORMATO DOS DADOS A RECEBER
    %read = fscanf(b,'T:%f,Ax:%f,Ay:%f,RAx:%f,RAy:%f\n\r') %FORMATO DOS DADOS A RECEBER
    %read = fscanf(b,'Gc:%fG:%f\n\r');
%     read = fscanf(b,'E:%f,%f,%f,P:%f,%f,%f,V:%f,%f\n\r'); %FORMATO DO CONTROLADOR LQ 20.12.17
read = fscanf(b,'%f,%f,%f,%f,%f,%f,%f,%f\n\r'); %FORMATO DO CONTROLADOR LQ 20.12.17
    xlabels(position) = time;
    time = time + 1;
    if (position < XPOINTS)
        position = position + 1;
    else
        position = 1;
    end
    
    
    
    y(position,1) = (read(1)); %x
    y(position,2) = (read(2)); %y
    y(position,3) = (read(3)); %yaw
    
    y(position,4) = (read(4)); %ex
    y(position,5) = (read(5)); %ey
    y(position,6) = (read(6)); %e_yaw
    
    y(position,7) = (read(7)); %w
    y(position,8) = (read(8)); %w
    drawnow
    plot(y(1:position,1),y(1:position,2));
    
end
%% PLOTS (OPTIONAL)
close all