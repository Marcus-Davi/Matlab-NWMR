% plota trajetórias
figure %1
hold on;
title('Comparação das trajetórias')
%xlabel('x(m)'), ylabel('y(m)'), axis([-1.5 1.5 -2.5 2.5]);
xlabel('x(m)'), ylabel('y(m)');
plot(xref,yref,'r--',...
    'LineWidth',3)    
plot(x_real1,y_real1,'b--',...    
    'LineWidth',2)
plot(x_real,y_real,'g-.',...
    'LineWidth',2)    
legend('trajetória de referência',...
    'trajetória-adaptativo','trajetória-fixo',...
    'Location','Best');
    %'Location','northoutside','Orientation','horizontal');  %'Location','Best')
% plota esforços de controle
figure %2
title('Esforços de controle das velocidades')
subplot(2,1,1)
xlabel('t(s)'), ylabel('Velocidade linear V(m/s)')
plot(t,Vc1(:,1),'r',t,Vc(:,1),'b--','LineWidth',2)
legend('Esforço de controle-adaptativo V','Esforço de controle-fixo V',...
        'Location','Best');
    %'Location','northoutside','Orientation','horizontal');  %'Location','Best')
subplot(2,1,2)
xlabel('t(s)'), ylabel('Velocidade angular W(rad/s)')
plot(t,Vc1(:,2),'r',t,Vc(:,2),'b--','LineWidth',2)
legend('Esforço de controle-adaptativo W','Esforço de controle-fixo W',...
        'Location','Best');
    %'Location','northoutside','Orientation','horizontal');  %'Location','Best')

% plota os erros
figure %3
title('Erro de trajetória (variáveis de estado)')
subplot(3,1,1)
xlabel('t(s)'), ylabel('Erro x(m)')
plot(t,ep1(:,1),'r',t,ep(:,1),'b--','LineWidth',2) 
legend('Erro x - adaptativo','Erro x - fixo',...
        'Location','Best');
    %'Location','northoutside','Orientation','horizontal');  %'Location','Best')
subplot(3,1,2)
xlabel('t(s)'), ylabel('Erro y(m)')
plot(t,ep1(:,2),'r',t,ep(:,2),'b--','LineWidth',2)
legend('Erro y - adaptativo','Erro y - fixo',...    
        'Location','Best');
    %'Location','northoutside','Orientation','horizontal');  %'Location','Best')
subplot(3,1,3)
xlabel('t(s)'), ylabel('Erro theta(rad)')
plot(t,ep1(:,3),'r',t,ep(:,3),'b--','LineWidth',2)
legend('Erro theta - adaptativo','Erro theta - fixo',...
        'Location','Best');
    %'Location','northoutside','Orientation','horizontal');  %'Location','Best')

% plota os ganhos
figure %4
title('Análise dos ganhos fixo e adaptativos')
subplot(3,1,1)
xlabel('t(s)'), ylabel('Kx(u)')
plot(t,K1(:,1),'r',t,K(:,1),'b--','LineWidth',2)
legend('Kx - adaptativo','Kx - fixo',...
        'Location','Best');
    %'Location','northoutside','Orientation','horizontal');  %'Location','Best')
subplot(3,1,2)
xlabel('t(s)'), ylabel('Ku(u)')
plot(t,K1(:,2),'r',t,K(:,2),'b--','LineWidth',2)
legend('Ky - adaptativo','Ky - fixo',...
        'Location','Best');
    %'Location','northoutside','Orientation','horizontal');  %'Location','Best')
subplot(3,1,3)
xlabel('t(s)'), ylabel('Ktheta(u)')
plot(t,K1(:,3),'r',t,K(:,3),'b--','LineWidth',2)
legend('Ktheta - adaptativo','Ktheta - fixo',...
        'Location','Best');
    %'Location','northoutside','Orientation','horizontal');  %'Location','Best')

hold off
