clear all; close all; clc;

global Ts;

traj = input('Digite 1 para trajetória circular e 2 para oito: ');

% trajetória circular
v = 0.3;
w = 0.15;
u1 = [v w]'; % u = [v w];
u2 = [v -w]';
xo = [0 0 0]'; % xo = [xo yo theta0] posição inicial

m = input('Digite o número de voltas desejadas: '); %number of laps
Ts = 0.1; 
k = round(traj*(2*pi/(abs(u1(2))*Ts)+1));
t = 0:Ts:Ts*k-Ts;

for l = 1:k; 
    
    % para oito
    if traj == 2
        
        if l >= k/2
            p = modelo(u2,xo);
            ubase(:,l) = u2;
        else
            p = modelo(u1,xo);
            ubase(:,l) = u1;
        end
        
    elseif traj == 1
%     % para circulo
        p = modelo(u1,xo);
        ubase(:,l) = u1;
    end
    
    x(l)=p(1);
    y(l) = p(2);
    theta(l) = p(3);
    xo = p;
    
    % save output and input references
    ref(:,l) = p;
end;

ref = repmat(ref,1,m); % account for the number of laps
ubase = repmat(ubase,1,m);

save('ref.mat','ref');
save('ubase.mat','ubase');

figure(1)
subplot (1,2,1);
stairs(x,y,'r','LineWidth', 2);
legend('Reference Trajectory');
ylabel('Y[m]');
xlabel('X[m]');
axis tight;
grid on;

subplot (1,2,2);
plot(t,theta(1:length(t)),'LineWidth', 2);
legend('Theta');
ylabel('Theta [rad]');
axis tight;
grid on;