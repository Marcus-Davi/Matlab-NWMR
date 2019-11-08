%%
if exist('b')
fclose(b);
end
clear all
close all
clc
disp('Conectando ao Bluetooth...');
b = Bluetooth('HulkBluetooth',1); %NOME DO DISPOSITIVO BLUETOOTH
fopen(b);
disp('Conectado.')
  XPOINTS = 400;
 position = 1;       
    time = 1;
    
    x = [(1:XPOINTS)'];
    xlabels = (1:XPOINTS);
    y = zeros(XPOINTS,2);

%%

    
    while(1)
    read=0;
while(b.BytesAvailable==0);
end
       	read = fscanf(b,'%f,%f,%f,%f,%f\r\n')
        
                if (position < XPOINTS) 
            position = position + 1;
                 else
            position = 1;
                end
                u = read(2)*256 + read(1); %Vd
                isNegative = int16(bitget(u,16));
                convertedValue = int16(bitset(u,16,0)) + (-2^15)*isNegative;
                y(position,1) = convertedValue;
                u = read(5)*256 + read(4); %Ve
                isNegative = int16(bitget(u,16));
                convertedValue = int16(bitset(u,16,0)) + (-2^15)*isNegative;
                y(position,2) = convertedValue;
        
        drawnow
        plot(x,y);
   %     Fig1Ax1 = get(1, 'Children')
    %    Fig1Ax1Line1 = get(Fig1Ax1, 'Children')
     %   set(Fig1Ax1Line1, 'LineWidth', 4)
       % legend('Velocidade','Posicao(Pulsos)')
        time=time+1;
        
    end
    
    %isNegative = int16(bitget(u,16));
    %convertedValue = int16(bitset(u,16,0)) + (-2^15)*isNegative;
    






