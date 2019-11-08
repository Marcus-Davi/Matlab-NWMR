clear all; close all; clc;
global Ts;

% alfa = 0.1;
% D = [1 -1];
% C = conv([1 -alfa],[1 -alfa]);
% D = [D 0];
N = 100; % Horizonte de predição
% [E,F] = GetFE(C,D,N);
Nu = 1; % Horizonte de controle
format long g;
% lamb = 0.1; % Ajuste de controle
% Qlamb = lamb*eye(Nu);
% Qlamb(1,1) = 0;
% Qlamb = blkdiag(Qlamb,Qlamb,Qlamb);
% np = zeros(length(C)-1,1);

% error weight
% xa = -1+1/N:1/N:0; ya = exp(xa);
% Qdelta = diag(ya);
% Qdelta = eye(N);
% Qdelta = blkdiag(Qdelta,Qdelta,Qdelta);

% %cria M, depois T e uz
% l = ones(1,Nu);
% M = diag(l);
% 
% for na = 1:(Nu-1);
%     M(na+1,na)=-1;
% end;
% M = blkdiag(M,M,M);

load ('ref.mat');
xr = ref(1,:); yr = ref(2,:); tr = ref(3,:);
load ('ubase.mat');

du = 0.00001;
ub = ubase(:,1:N);% tamanho do horizonte de predição
vb = ub; vb(2,:) = 0; vbd = vb; vbd(1,:) = vbd(1,:) + du;
wb = ub; wb(1,:) = 0; wbd = wb; wbd(2,:) = wbd(2,:) + du;
% U0 = zeros(Nu,2); 
% U0 = blkdiag(U0,U0,U0);
xo = [0 0.5 0]';

% time profile
Ts = 0.1; 
w = length(ref)/2;
t = 0:Ts:Ts*w-Ts;

P = [0 0]';
u0 = ub(:,1);
% [xx,zi] = filter(1,C,0);

for k = 1:w;
    
    if k > 100;
        y(:,k)=modelo(u0+P,xo); % *(LIDA) simulando a saída da planta,
        %ub = u0*ones(1,N);
    else
        y(:,k)=modelo(u0,xo);
    end

%     if k >= 2;
%         ub = [ub(:,2:end) ub(:,end)];
%     else
%         ub = u0*ones(1,N);
%     end

   % atualiza a base
    ub = ubase(:,k:N+k-1);
    vb = ub; vb(2,:) = 0; vbd = vb; vbd(1,:) = vbd(1,:) + du;
    wb = ub; wb(1,:) = 0; wbd = wb; wbd(2,:) = wbd(2,:) + du;
    
    %nada haver com o controlador. Próximo passo é determinar resposta base 
    x(:,k)=modelo(u0,xo); % (CALCULADA)
    n = y(:,k) - x(:,k);
    
%     [nf(:,k),zf] = filter(1,C,n(:,k),zi);
%     zi = zf;
%     np = [nf(:,k);np(1:end-1)];
%     Np = F*np;
    
    % atualiza postura atual
    xo = y(:,k);
    
    % monta G
    G = cell(1,3);
    for j = 1:3; %j=1 -> x, j=2 -> y, j=3 -> theta. 
  
    xbv = ybase(vb,xo,n); xbv = xbv(j,:);
    xbvd = ybase(vbd,xo,n); xbvd = xbvd(j,:);
    xbw = ybase(wb,xo,n); xbw = xbw(j,:);
    xbwd = ybase(wbd,xo,n); xbwd = xbwd(j,:);
    
    if Nu > 1;
        % for v
        gxv = (xbvd'-xbv')/du; gxv2 = [0; gxv]; hxv = diff(gxv2);
        Gxv = zeros(N,Nu);
        % for w
        gxw = (xbwd'-xbw')/du; gxw2 = [0; gxw]; hxw = diff(gxw2);
        Gxw = zeros(N,Nu);
        
    for l = 1:Nu-1;

        Gxv(:,l) = [zeros(l-1,1);hxv(1:length(hxv)-l+1)]; % for v
        Gxw(:,l) = [zeros(l-1,1);hxw(1:length(hxw)-l+1)]; % for w
        
    end
    
    for l = 1:N; 

        Gxv(l,Nu) = sum(Gxv(1:l,1)); % for v
        Gxw(l,Nu) = sum(Gxw(1:l,1)); % for w
        
    end;
    
    Gxv(:,Nu) = [zeros(1,Nu-1) Gxv(1:N-Nu+1,Nu)']'; % for v
    Gxw(:,Nu) = [zeros(1,Nu-1) Gxw(1:N-Nu+1,Nu)']'; % for w
    
    else
    
        Gxv = (xbvd'-xbv')/du; % for v
        Gxw = (xbwd'-xbw')/du; % for w
        
    end;
    
    G{1,j} = [Gxv Gxw];
    
    end;

    Gx = G{1,1};
    Gy = G{1,2};
    Gt = G{1,3};
    
    Yb = ybase(ub,xo,n); 
    xb = Yb(1,:);
    yb = Yb(2,:);
    tb = Yb(3,:);
    errx = (xr(:,k:N+k-1)-(xb))';
    erry = (yr(:,k:N+k-1)-(yb))';
    errt = (tr(:,k:N+k-1)-(tb))';
    errt = atan2(sin(errt),cos(errt)); % smooth the error
    
    % control action
    
    uo = (Gx'*Gx+Gy'*Gy+Gt'*Gt)\(Gx'*errx+Gy'*erry+Gt'*errt);

    u(:,k) = uo(1:2,1)+ub(:,1);
    u0 = u(:,k);
    %U0(:,1) = u(:,k);
    %ub = u0*ones(1,N);

end

figure(1)
subplot(2,1,1)
plot(y(1,:),y(2,:),'LineWidth',2); hold on;
plot(ref(1,1:length(y(1,:))),ref(2,1:length(y(1,:))),'r--','LineWidth',2);
legend('Real Robot','Reference Robot');
ylabel('Y[m]');
xlabel('X[m]');
axis tight;
grid on;

subplot(2,1,2)
plot(t(1:k),y(3,:),'LineWidth',2); hold on;
plot(t(1:k),tr(1:k),'LineWidth',2); 
legend('Real Robot','Reference Robot');
ylabel('Theta[rad]');
xlabel('T[s]');
axis tight;
grid on;

figure(2)
subplot(2,1,1)
plot(t(1:k),y(1,:)-xr(1:length(y)),'LineWidth',2); hold on;
plot(t(1:k),y(2,:)-yr(1:length(y)),'LineWidth',2); hold on;
err_theta = atan2(sin(y(3,:)-tr(1:length(y))),cos(y(3,:)-tr(1:length(y))));
plot(t(1:k),err_theta,'LineWidth',2); 
legend('Xerro','Yerro','Thetaerro');
ylabel('Error');
xlabel('T[s]');
axis tight;
grid on;

subplot(2,1,2)
plot(t(1:k),ubase(1,1:k),'LineWidth',2); hold on;
plot(t(1:k),u(1,1:k),'LineWidth',2); hold on;
plot(t(1:k),ubase(2,1:k),'LineWidth',2); hold on;
plot(t(1:k),u(2,1:k),'LineWidth',2);
legend('vref','v','wref','w');
xlabel('T[s]');
axis tight;
grid on;