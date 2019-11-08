clear all; close all; clc;

Ts=0.1;

%Initial Condition
xo = 0;
yo = 0;
thetao = 0;

% Parameters of the robot physic structure
r = 0.07; %raio da roda
d = 0.4; % comprimento do eixo entre rodas;

% Parameters of the DC motors and Speed Controller
J = 0.01; %kg.m^2  moment of inertia of the rotor
b = 0.1; %N.m.s   motor viscous friction constant
Ke = 0.08; % V/rad/sec    electromotive force constant
Kt = 0.08; % N.m/Amp   motor torque constant
R = 1; % Ohm electric resistance
L = 0.5; %H  electric inductance

%PID
Kp=5.0;
Ki=10.0;
Kd=0;

%%%
% load reference
load ('ref.mat');
xr = ref(1,:); yr = ref(2,:); tr = ref(3,:);

load ('ubase.mat');
vr = ubase(1,:);
wr = ubase(2,:);
w = length(ref);
tmax = Ts*w-Ts;
t = 0:Ts:tmax;
k = 1:w;
k = [t' k'];

Tsim = tmax;
sim('motorsPID');

% rename simout data
t = simout.time;
v = simout.signals.values(:,1);
w = simout.signals.values(:,2);

x = pos.signals.values(:,1);
y = pos.signals.values(:,2);
theta = pos.signals.values(:,3);

% Put theta in range [-pi,pi]
for l = 1:length(theta)
    
    tmed = theta(l);
    N = tmed/(2*pi);
    
    if N > 1;
        tmed = tmed - (round(N))*2*pi;
    end
    
    if N < -1;
        tmed = tmed - (round(N))*2*pi;
    end
    
    if tmed > pi
        tmed = tmed - 2*pi;
    end
    
    if tmed < -pi
        tmed = tmed + 2*pi;
    end
    
    theta(l) = tmed;
    
end
%plots
figure(1)
plot(x,y,'k','LineWidth',2); hold on;
plot(xr,yr,'r--','LineWidth',2);
legend('Real Robot','Reference Robot');
ylabel('Y[m]');
xlabel('X[m]');
axis tight;
grid on;

figure(2)
plot(t,theta,'LineWidth',2); hold on;
plot(t,tr,'LineWidth',2);
legend('Real Robot','Reference Robot');
ylabel('Theta[rad]');
xlabel('T[s]');
axis tight;
grid on;
%
figure(3)
plot(t,x-xr','LineWidth',2); hold on;
plot(t,y-yr','LineWidth',2); hold on;
err_theta = atan2(sin(theta-tr'),cos(theta-tr'));
plot(t,err_theta,'LineWidth',2);
legend('Xerro','Yerro','Thetaerro');
ylabel('Error');
xlabel('T[s]');
axis tight;
grid on;
%
figure(4)
plot(t,vr,'LineWidth',2); hold on;
plot(t,v,'LineWidth',2); hold on;
plot(t,wr,'LineWidth',2); hold on;
plot(t,w,'LineWidth',2);
legend('vref','v','wref','w');
xlabel('T[s]');
axis tight;
grid on;