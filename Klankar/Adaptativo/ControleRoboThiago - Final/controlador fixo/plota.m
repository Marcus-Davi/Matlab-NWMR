% plot(xref,yref,'r--',...
%     'LineWidth',3)    

figure
plot(xref,yref,'r--',...
    'LineWidth',3)    
hold on
% plot(x_real,y_real,'b--square',...    
%     'LineWidth',1,...    
%     'MarkerFaceColor',[.49 1 .63],...
%     'MarkerSize',5)
plot(x_real,y_real,'b',...
    'LineWidth',2)    
legend('saída de referência','saída real medida')

figure
plot(t,K(:,1),'r',t,K(:,2),'b',t,K(:,3),'k')
legend('Kx','Ky','Kteta')

figure
plot(t,Vc(:,1),'r',t,Vc(:,2),'b')
legend('Esforço de controle V','Esforço de controle W')

figure
plot(t,ep(:,1),'r',t,ep(:,2),'b',t,ep(:,3),'k')
legend('Erro x','Erro y','Erro teta')
