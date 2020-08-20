clear;close all;clc
%% SIMULATION PARAMETERS
Ts = 0.1;
v = 0.15;
% R = 1;

% Modelo do robô
R_real = 0.08; R_usado = 0.08;
D_real = 0.4; D_usado = 0.4;
Uncertainty.R = R_usado/R_real; %raio da roda
Uncertainty.D = D_usado/D_real; % distancia entre rodas


% Waypoints
Xr = [0 0;
    1 0;
    1 1;
    2 1;];



%% Controller Parameters
% Pi Controller
Kp = 10;
Ki = 100; %206.157 @ Ts = 0.1

%Feuler
pi_a = Kp+ Ki*Ts;
pi_b = Ki*Ts;

%% Simulation
yk = [-0.5 -0.5 0]';
yk_real = yk;
u = [0.0 0]';
w = u(2);
e_t1 = 0; %past error

YK = [];
YK_REAL = [];
UK = [];

tol = 0.05;

for k=1:length(Xr)
   
%     k
    dist = 10;
    while dist > tol
        e = Xr(k,:) - yk(1:2)';
        dist = sqrt(e*e');
        ux = e(1);
        uy = e(2);
        
        t_r = atan2(uy,ux);
        
        v = sqrt(ux*ux + uy*uy);
        e_t = t_r - yk(3);
        e_t = atan2(sin(e_t), cos(e_t)); %ajuste
        w = w + pi_a*e_t - pi_b*e_t1;
        % w = 0.1*e_t;
        
        
        
        if v > 1
            v = 1;
        elseif v < 0
            v = 0;
        end
        
        if w > 6
            w = 6;
        elseif w < -6
            w = -6;
        end
        
        u = [v w]';
        
        yk = robot_model(yk,u,Ts); %yk = measurement
        yk_real = robot_model(yk_real,u,Ts,Uncertainty); %yk = measurement
        
        %updates
        e_t1 = e_t;
        
        
        % Plots
        YK = [YK;yk'];
        YK_REAL = [YK_REAL;yk_real'];
        UK = [UK;u'];
        
%         plot(YK(:,1),YK(:,2))
%         
%         drawnow
%         pause(0.01)
        
    end
    
    
end

%% Plots
close all
plot(YK(:,1),YK(:,2))
hold on
plot(YK_REAL(:,1),YK_REAL(:,2))
plot(Xr(:,1),Xr(:,2),'black*')
grid on
legend('Estimado','Real','Ref')
figure
plot(UK(:,1))
hold on
plot(UK(:,2))
grid on
legend('v','w')





