%% Klankar Trajectory Tracking + IMU feedback
% Modificado por Marcus
% 08.04.2016
clear;
close all;
clc
% global Ts_sensor;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            Inicializa��o de Vari�veis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 0.1; % sampling time (control)
Ts_sensor = 1/50; %sampling time (sensor)
Tsim = 50;
%Tsim = (2*pi*1)/Ts; %pro oito

D_rodas = 0.4;            %distancia entre rodas
R_rodas = 0.07; %raio das rodas
limit = 1;



%x0 = [-0.5 -0.5 0];%posicao inicial robo

Amostras = round(Tsim/Ts);
%% Reference Trajectory
xr_k = [0.5; 0.5; 0];  % x,y,theta referenciais para in�cio de trajet�ria.
X = xr_k;
for k = 1:Amostras  %
    
%     %S
%     v(k) = 0.1;
%     w(k) = 0.1;
%     if(k>Amostras/2)
%         w(k) = -0.1;
%     end
    
    % "LL"
%         w(k)=0;
%         v(k)=0.1;
%         if(k<Amostras*2/5)
%             xr_k(3)= 0;
%         elseif(k>Amostras*2/5 && k<=Amostras*3/5)
%             xr_k(3)= -pi/2;
%         else
%             xr_k(3)=0;
%          end

%     "LS"

        v(k)=0.1;
        if(k<150)
            xr_k(3)= 0;
        elseif(k>=150 && k<=300)
            xr_k(3)= -pi/2;
        else
            w(k) = -0.15;
            xr_k(3) = xr_k(3) + Ts*w(k);
         end

%        "SQUARE"
%         w(k) = 0;
%         v(k) = 0.1;
%         if(k<Amostras/4)
%             xr_k(3) = 0;
%         elseif(k<2*Amostras/4)
%             xr_k(3)=-pi/2;
%         elseif(k<3*Amostras/4)
%             xr_k(3) = -pi;
%         else
%             xr_k(3) = -3*pi/2;
%         end


%    X = [X, X(:,k)+ Ts*([v(k)*cos(xr_k(3));v(k)*sin(xr_k(3));w(k)])];  %Euler
%    xr_k = X(:,k+1); %Computed Angle
%    
    X = [X, X(:,k)+ Ts*([v(k)*cos(xr_k(3));v(k)*sin(xr_k(3));0])]; %euler xy
    X(3,k)=xr_k(3); %Forced Angle
end
X(3,end) = xr_k(3);Xr = X; %Referencia computada
vr = v;
wr = w;
XrSIM = timeseries(Xr',0:Ts:Tsim);
UrSIM = timeseries([v' w'],0:Ts:Tsim-Ts);

%@OVERRIDE PARA USO DE DADOS PRE COMPUTADOS
% Xr = load('POSTURES_REF');
% Xr = Xr.y2;
% Tsim = length(Xr)*Ts;
% v = 0.1*ones(1,length(Xr));
% w = 0.0*ones(1,length(Xr));
% XrSIM = timeseries(Xr',0:Ts:Tsim-Ts);
% UrSIM = timeseries([v' w'],0:Ts:Tsim-Ts);
%@OVERRIDE FIM

% plot(Xr(1,:),Xr(2,:),'--','Linewidth',1.5,'color','black'); %glimpse

save('GLOBAL_REF.mat','XrSIM');
save('GLOBAL_FEEDFORWARD_REF.mat','UrSIM');
%% K CONTROLLER
ksi = 0.6;
% ohmega_n = 1; %calculado iterativamente no simulink
g =40;
%% LQ CONTROLLER DESIGN
ur1 = 0.1;
ur2 = 0.0;
A = [0 ur2 0;-ur2 0 ur1;0 0 0];
B = [1 0;0 0;0 1];
C = eye(3);
D = 0;
SYS = ss(A,B,C,D);
Q = diag([1 50 0]);
R = 1*eye(2);
[K_LQ,S,E] = lqr(SYS,Q,R);
%% MOTOR DYNAMICS;
% Parameters of the DC motors and Speed Controller
J = 0.01; %kg.m^2  moment of inertia of the rotor
b = 0.1; %N.m.s   motor viscous friction constant
Ke = 0.08; % V/rad/sec    electromotive force constant
Kt = 0.08; % N.m/Amp   motor torque constant
R = 1; % Ohm electric resistance
L = 0.5; %H  electric inductance
HMotor = tf(Ke,[J*L  (J*R+b*L) (b*R+Ke^2)]);
%PID
Kp=5.0;
Ki=10.0;
Kd=0;
%% GYRO + MAG
Gyro_bias = 0.004;
A_g = [1 -Ts_sensor;0 1];
B_g = [Ts_sensor 0]';
C_g = [1 0];
SYS_g = ss(A_g,[B_g eye(2)],C_g,0,Ts_sensor);
Rw_g = 0.01*[Ts_sensor^2/2 0;0 Ts_sensor];
Rv_g =0.025;
[KEST_g,K_KALM_g,P_g] = kalman(SYS_g,Rw_g,Rv_g);

%% Encoder-Accelerometer Fusion (Body X)
A_enc = [1 -Ts_sensor;0 1];
B_enc = [Ts_sensor 0]';
C_enc = [1 0];
Rw_enc = 0.1*[Ts_sensor^2/2 0;0 Ts_sensor^2/2];
Rv_enc = 0.002;

 SYS_enc = ss(A_enc,[B_enc eye(2)],C_enc,0,Ts_sensor);
 [KEST_enc,K_KALM_enc,P_enc] = kalman(SYS_enc,Rw_enc,Rv_enc);




% K_KALM_enc
%% Accelerometer Double Integration (BODY Y)
A_y = [1 Ts_sensor Ts_sensor^2/2;
    0 1 Ts_sensor;
    0 0 1];
B_y = zeros(3,1);
C_y = [0 0 1];
Rv_y = 0.01; %Senosr variance
Rw_y = 0.05*[Ts_sensor^5/20 Ts_sensor^4/8 Ts_sensor^3/6; %Process
    Ts_sensor^4/8 Ts_sensor^3/3 Ts_sensor^2/2;
    Ts_sensor^3/6 Ts_sensor^2/2 Ts_sensor];

%SYS_a = ss(A_x,[B_x eye(3)],C_x,0,Ts_sensor);
discriminator = 0.8;
buffer_size = 15;
% [KEST_a,K_KALM_a,P_a] = kalman(SYS_a,Qna,Rna);
%% DISTURBANCE DYNAMIC
D_QSI = 1;
D_WN = 9;
D_num = D_WN^2;
D_den = [1 2*D_WN*D_QSI D_WN^2];
% D_num = 1;
% D_den = [0.00001 1];
%% ROBOT SIMULATION 
sim('Robot_Sim');
%%
close all;
%realPI
x = pose.data(:,1);
y = pose.data(:,2);
th = pose.data(:,3);
%odometry_mag_gyro_acc estimation
o_x = odometry.data(:,1);
o_y = odometry.data(:,2);
o_th = odometry.data(:,3);
%odometry_mag_gyro estimation
ogm_x = odometry_mag.data(:,1);
ogm_y = odometry_mag.data(:,2);
ogm_th = odometry_mag.data(:,3);
%odometry_mag_gyro_acc estimation
oagm_x = odometry_mag_acc.data(:,1);
oagm_y = odometry_mag_acc.data(:,2);
oagm_th = odometry_mag_acc.data(:,3);
%odometry_mag_gyro_acc estimation
% oagm_x2 = odometry_mag_acc2.data(:,1);
% oagm_y2 = odometry_mag_acc2.data(:,2);
% oagm_th2 = odometry_mag_acc2.data(:,3);
%errors
e_x = errors.data(:,1);
e_y = errors.data(:,2);
e_t = errors.data(:,3);
etime = errors.time;

%control
uv = control.data(:,1);
uw = control.data(:,2);
time =control.time;
Rv = var(noise.data);
plot(Xr(1,:),Xr(2,:),'--','Linewidth',1.5,'color','black');
hold on;
plot(x,y,'Linewidth',1.5,'color','blue');
plot(o_x,o_y,'Linewidth',1.2);
plot(ogm_x,ogm_y,'Linewidth',1.2);
plot(oagm_x,oagm_y,'Linewidth',1.2,'color','red');
% plot(oagm_x2,oagm_y2,'Linewidth',1.2,'color','magenta');
LEG = legend({'Reference','Actual Robot','Odometry','$\mathrm{Fusion}_{GM}$','$\mathrm{Fusion}_{AGM}$'},'Location','northeast','interpreter','latex');
xlabel('X[m]');
ylabel('Y[m]');
LEG.FontSize = 11;
grid on;
figure
subplot(2,1,1)
plot(time,uv,'Linewidth',1.2,'color','blue');
xlabel('Time[s]')

ylabel('m/s');
grid on;
LEG = legend({'$v$'},'Location','northeast','interpreter','latex');
LEG.FontSize = 11;
subplot(2,1,2);
plot(time,uw,'Linewidth',1.2,'color','red');
xlabel('Time[s]')
ylabel('rad/s');
grid on;
LEG = legend({'$\omega$'},'Location','northeast','interpreter','latex');
LEG.FontSize = 11;
figure
hold on;
grid on;
plot(etime,e_x,'Linewidth',1.2,'color','blue');
plot(etime,e_y,'Linewidth',1.2);
plot(etime,e_t,'Linewidth',1.2);
LEG = legend({'$x$ Error[m]','$y$ Error[m]','$\theta$ Error[rad]'},'Location','northeast','Interpreter','latex');
LEG.FontSize = 11;
xlabel('Time[s]');

%% fusion yaw
figure; hold on;
grid on;
title('Yaw estimation');
xlabel('Time [s]');
ylabel('Yaw [rad]');
plot(yaw_mag.time,yaw_mag.data,'linewidth',1.5)
plot(yaw_odo.time,yaw_odo.data,':','linewidth',1.5)
plot(yaw_gyro.time,yaw_gyro.data,'--','linewidth',2)
plot(yaw_kalman.time,yaw_kalman.data,'linewidth',2)
plot(yaw_real.time,yaw_real.data,'-black','linewidth',1.5)

legend({'Magnetometer $\phi$','Odometry $\phi$','Gyroscope $\phi$','KF $\phi$','Real $\phi$'},'Location','Southeast','Interpreter','latex');

%% fusion acc
figure;hold on; grid on;
title('Position estimation on navigational X axis');
xlabel('Time [s]');
plot(acc_data_x.time,acc_data_x.data)
plot(acc_kalman_x.time,acc_kalman_x.data(:,1),'--black','linewidth',2)
plot(acc_kalman_x.time,acc_kalman_x.data(:,2),':red','linewidth',2)
plot(acc_kalman_x.time,acc_kalman_x.data(:,3),'-')
legend({'Raw acceleration $[m/s^2]$','Modified KF position $\hat{\delta}_x$ $[m]$','Modified KF velocity $v$ $[m/s]$','Modified KF acceleration $a$ $[m/s^2]$'},'Interpreter','latex')

figure;hold on; grid on;
title('Position estimation on navigational Y axis');
xlabel('Time [s]');
plot(acc_data_y.time,acc_data_y.data)
plot(acc_kalman_y.time,acc_kalman_y.data(:,1),'--black','linewidth',2)
plot(acc_kalman_y.time,acc_kalman_y.data(:,2),':red','linewidth',2)
plot(acc_kalman_y.time,acc_kalman_y.data(:,3),'-')
legend({'Raw acceleration $[m/s^2]$','Modified KF position $\hat{\delta}_y$ $[m]$','Modified KF velocity $v$ $[m/s]$','Modified KF acceleration $a$ $[m/s^2]$'},'Interpreter','latex','location','northwest')
return
%% OTHER
figure;
plot(rec_vs_ss.time,rec_vs_ss.data)
grid on;
xlabel('Time [s]','interpreter','latex')
ylabel('Estimated Yaw $\phi$','Interpreter','Latex')
legend({'Recursive KF','Steady-State KF'},'Interpreter','Latex')