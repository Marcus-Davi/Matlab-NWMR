% ANIMAÇÃO DA TRAJETÓRIA DO ROBÔ
figure
% Desenho do robô
a = [-1 -1]'; 
b = [2 0]'; 
c = [-1 1]'; 
P = 0.1*[a b c]; % matriz dos vértices de um triângulo (representando o robô-adaptativo)
P1 = 0.15*[a b c]; % matriz dos vértices de um triângulo (representando o robô-fixo)
tpause = 0; % variável auxiliar de tempo da animação

for k = 1:length(t)
    if tpause < k   % Tempo de exibição dos frames da animação
        tpause = tpause + 50;
        pause(0.05);                     
        
        h1 = plot(x_real1(1:k),y_real1(1:k),'MarkerEdgeColor','r');         
        hold on;
        h = plot(x_real(1:k),y_real(1:k),'MarkerEdgeColor','b'); 
        %axis equal; axis([-2 2 -2.5 2.5]);        
        href = plot(xref(1:k), yref(1:k),'k--','LineWidth',1);        
        %plot_ref = plot(xref(1:k), yref(1:k),'k--','LineWidth',1);        
        plot(x_real(1:k),y_real(1:k),'b-.','LineWidth',2);
        %plot_fixo = plot(x_real(1:k),y_real(1:k),'b-.','LineWidth',2);
        plot(x_real1(1:k),y_real1(1:k),'r-.','LineWidth',3);
        %plot_adap = plot(x_real1(1:k),y_real1(1:k),'r-.','LineWidth',3);
        % Robô-adaptativo
        R1 = [ cos(Theta1(k)) -sin(Theta1(k)); sin(Theta1(k)) cos(Theta1(k))]; % rotation matrix to rotate car. 
        Prot1 = R1*P1; % inclinação do robô
        Prot_trasl1 = Prot1 + [ones(1,3)*x_real1(k); ones(1,3)*y_real1(k)]; % add offset of car's center 
        robo1 = patch(Prot_trasl1(1,:),Prot_trasl1(2,:),'r','LineWidth',2); 
        % Robô-fixo
        R = [ cos(Theta(k)) -sin(Theta(k)); sin(Theta(k)) cos(Theta(k))]; % rotation matrix to rotate car. 
        Prot = R*P; % inclinação do robô
        Prot_trasl = Prot + [ones(1,3)*x_real(k); ones(1,3)*y_real(k)]; % add offset of car's center 
        robo = patch(Prot_trasl(1,:),Prot_trasl(2,:),'b','LineWidth',1); 
        hold off;                 
        %legend([robo1,robo],'Robô c/ ganho adaptativo','Robô c/ ganho fixo','Location','northoutside','Orientation','horizontal');        
        legend([href,robo1,robo],'Trajetória de Referência','Robô c/ ganho adaptativo','Robô c/ ganho fixo','Location','northoutside','Orientation','horizontal');        
        %l = legend([robo1,robo],'Robô c/ ganho adaptativo','Robô c/ ganho fixo','Location','northoutside','Orientation','horizontal');        
        %title(l,'Legenda'); %não funciona...
        set(h1,'EraseMode','xor');
        set(h,'EraseMode','xor');        
    end
end
        