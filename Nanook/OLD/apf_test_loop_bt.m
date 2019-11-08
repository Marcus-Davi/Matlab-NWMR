%% Estabelecimento da Conexao Bluetooth
if exist('b')
    fclose(b);
end
clear ;close all;clc
disp('Conectando ao Bluetooth...');
%b = Bluetooth('HulkBluetooth',1,'Terminator','CR'); %NOME DO DISPOSITIVO BLUETOOTH
b = serial('/dev/rfcomm0','BaudRate',115200,'Terminator','CR');
fopen(b);
disp('Conectado.')

close all;clc
%rosinit

laser = rossubscriber('/scan');



while(1)
   scan = receive(laser);

   
   ranges = scan.Ranges;
angleI = scan.AngleIncrement;
angleMin = scan.AngleMin;
   F = ranges2force(ranges,angleI,angleMin)
   [v,w] = force2vw(F);
   [vd,ve] = vw2rpm(v,w);
   vd
   ve
   motorGo(b,vd,ve)
A = atan2(F(2),F(1));

   plot(scan);
   hold on;
%    plotv(F)
    quiver(0,0,F(1),F(2),'Color','r','linewidth',2);
   hold off;
   
end
    











% angle = atan2(F(2),F(1))