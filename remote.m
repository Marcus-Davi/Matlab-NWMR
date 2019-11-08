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
global dir;
dir = 0;
%% Remote
h_fig =figure;
set(h_fig,'KeyPressFcn',@(fig_obj,event) myfun(fig_obj,event));

%% loop
vel = 30;
while true
   switch(dir)
       case 0
            fprintf(b,'M!0,0\r'); %space
       case 1
            fprintf(b,'M!0,-45\r'); %up
       case -1
            fprintf(b,'M!0,45\r'); %down
       case 2
           fprintf(b,'M!-45,0\r'); %left
       case -2
           fprintf(b,'M!45,0\r');%right

%        case 0
%             fprintf(b,'G!0,0\r'); %space
%        case 1
%             fprintf(b,'G!45,45\r'); %up
%        case -1
%             fprintf(b,'G!-45,-45\r'); %down
%        case 2
%            fprintf(b,'G!-30,30\r'); %left
%        case -2
%            fprintf(b,'G!30,-30\r'); %right
   end
              
              pause(0.1);
end





%%
function myfun(~,event,port)
global dir;
vel = 45;
if (strcmp(event.Key,'uparrow'))
dir = 1;
disp('forward')
elseif (strcmp(event.Key,'downarrow'))
dir = -1;
disp('back')
elseif (strcmp(event.Key,'leftarrow'))
dir = -2;         
disp('left')
elseif (strcmp(event.Key,'rightarrow'))
dir = 2 ;          
disp('right')
elseif (strcmp(event.Key,'space'))
dir = 0;
disp('stop')
end


end