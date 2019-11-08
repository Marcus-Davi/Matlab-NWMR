%% Estabelecimento da Conexao Bluetooth

function bluetooth = blueNanook
bluetooth = serial('/dev/rfcomm0','BaudRate',115200,'Terminator','CR','InputBufferSize',1000,'Timeout',1.5);
fopen(bluetooth);
pause(5); % tempo aprox p/ conectar
disp('Conectado.')
end


