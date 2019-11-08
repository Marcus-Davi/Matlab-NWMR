clear all; close all; clc;

global Ts;

traj = input('Digite 1 para trajet�ria circular e 2 para oito: ');

% trajet�ria circular
v = 0.2;
w = 0.1;
u1 = [v w]'; % u = [v w];
u2 = [v -w]';
xo = [0 0 0]'; % xo = [xo yo theta0] posi��o inicial

m = input('Digite o n�mero de voltas desejadas: '); %number of laps
m = 2*m;
Ts = 0.1; 
k = traj*(2*pi/(abs(u1(2))*Ts)+1);
t = 0:Ts:Ts*k*m-Ts;

for n = 1:m;
for l = 1:k; 
    
    % para oito
    if traj == 2
        
        if l > k/2
            p = modelo(u2,xo);
            ubase(:,l+(n-1)*round(k)) = u2;
        else
            p = modelo(u1,xo);
            ubase(:,l+(n-1)*round(k)) = u1;
        end
        
    elseif traj == 1
%     % para circulo
        p = modelo(u1,xo);
        ubase(:,l+(n-1)*round(k)) = u1;
    end
    
    x(l+(n-1)*round(k))=p(1);
    y(l+(n-1)*round(k)) = p(2);
%      if p(3) >= pi
%        p(3) = p(3) - 2*pi;
%      end
    theta(l+(n-1)*round(k)) = p(3);
    xo = p;
    
    % save output and input references
    ref(:,l+(n-1)*round(k)) = p;
end;
end

save('ref.mat','ref');
save('ubase.mat','ubase');

plot(x,y);

figure(1)
subplot (1,2,1);
stairs(x,y,'r','LineWidth', 2);
legend('Reference Trajectory');
ylabel('Y[m]');
xlabel('X[m]');
axis tight;
grid on;

subplot (1,2,2);
plot(t,theta,'LineWidth', 2);
legend('Theta');
ylabel('Theta [rad]');
axis tight;
grid on;