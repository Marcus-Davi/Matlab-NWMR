%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% EPSAC FOR MOBILE ROBOT 05/09/2014
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc
format long g
tic

Ts = 0.1;             % sampling time
Tsim = 85;
h = 0.1;           % integrating time


x0 = [0; 0; 0];  % x,y,theta
X = x0;


xa = [0; 0; 0];  % x,y,theta
X1 = xa;

% Runge kutta

% Y(n+1) = Y(n) + (1/6)(k1 +2 k2 +2 k3 +k4);
% k1 = h f(X(n),Y(n))
% k2 = h f(X(n)+h/2, Y(n)+k1/2)
% k3 = h f(X(n)+h/2, Y(n) +k2/2)
% k4 = h f(X(n)+h, Y(n)+k3)

% The next loop difines the reference

for k = 1:round(Tsim/h)
    v(k) = 0.3;
    if k<= 419;
        w(k) = 0.15;
    else
        w(k) = -0.15;
    end
    
    k1 = h*[v(k)*cos(x0(3)); v(k)*sin(x0(3));w(k)];
    k2 = h*[v(k)*cos(x0(3)+k1(3)/2); v(k)*sin(x0(3)+k1(3)/2);w(k)];
    k3 = h*[v(k)*cos(x0(3)+k2(3)/2); v(k)*sin(x0(3)+k2(3)/2);w(k)];
    k4 = h*[v(k)*cos(x0(3)+k3(3)); v(k)*sin(x0(3)+k3(3));w(k)];
    
    X = [X, X(:,k) + (1/6)*(k1+2*k2+2*k3+k4)];
    x0 = X(:,k+1);
    
    X1 = [X1, X1(:,k)+ h*([v(k)*cos(xa(3));v(k)*sin(xa(3));w(k)])];
    xa = X1(:,k+1);
    
end

Xr = X;
Ur = [v;w];
plot(X(1,:),X(2,:),'r--','linewidth',4)
hold on
plot(0,0,'b--','linewidth',2)
plot(0,0,'k','linewidth',2)
% hold on
% plot(X1(1,:),X1(2,:),':')
%legend('runge_kutta','euler')
grid

vr = v;
wr = w;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%% Calculo do algoritmo
rel = 1;
pon = 1;
for caso = 1:8
    
   switch caso
        case 1
            qt = 0.5;
            x0 = [0; -1; pi/2];  % x,y,theta
            vmin = -0.4*rel;
            vmax = 0.4*rel;
            wmin = -0.4*rel;
            wmax = 0.4*rel;
            
        case 2
            qt = 0.5;
            x0 = [0; -3; pi/2];  % x,y,theta
            vmin = -0.4*rel;
            vmax = 0.4*rel;
            wmin = -0.4*rel;
            wmax = 0.4*rel;
            
        case 3
            qt = 0.5;
            x0 = [0; -1; 0*pi/2];  % x,y,theta
            vmin = -0.4*rel;
            vmax = 0.4*rel;
            wmin = -0.4*rel;
            wmax = 0.4*rel;
        case 4
            qt = 0.5;
            x0 = [0; -3; 0*pi/2];  % x,y,theta
            vmin = -0.4*rel;
            vmax = 0.4*rel;
            wmin = -0.4*rel;
            wmax = 0.4*rel;
            
        case 5
            qt = 0.1;
            x0 = [0; -1; pi/2];  % x,y,theta
            vmin = -0.4*rel;
            vmax = 0.4*rel;
            wmin = -0.4*rel;
            wmax = 0.4*rel;
            
        case 6
            qt = 0.1;
            x0 = [0; -3; 1*pi/2];  % x,y,theta
            vmin = -0.4*rel;
            vmax = 0.4*rel;
            wmin = -0.4*rel;
            wmax = 0.4*rel;
        case 7
            qt = 0.1;
            x0 = [0; -1; 0*pi/2];  % x,y,theta
            vmin = -0.4*rel;
            vmax = 0.4*rel;
            wmin = -0.4*rel;
            wmax = 0.4*rel;
        case 8
            qt = 0.1;
            x0 = [0; -3; 0*pi/2];  % x,y,theta
            vmin = -0.4*rel;
            vmax = 0.4*rel;
            wmin = -0.4*rel;
            wmax = 0.4*rel;
    end
    


N = 5;
Q = [1 0 0;
    0 1 0;
    0 0 qt];   %no segundo exemplo foi usado 0.1
R = 0.1*eye(2);
% dado = 1;
% vmin = -0.4*dado;
% vmax = 0.4*dado;
% wmin = -0.4*dado;
% wmax = 0.4*dado;

% vmin = -10;
%  vmax = 10;
%  wmin = -10;
%  wmax = 10;
lb = [];
ub = [];
for k = 1:N
    Qb(3*k-2:3*k,3*k-2:3*k) =Q;
    Rb(2*k-1:2*k,2*k-1:2*k)=R;
    lb = [lb;[vmin;wmin]];
    ub = [ub;[vmax;wmax]];
end

% matrizes de ponderação do Nepsac
Qn = Qb;
Rn = [R(1,1)*eye(N),zeros(N);zeros(N),R(2,2)*eye(N)];



%x0 = [0; -1.5; 1*pi/2];  % x,y,theta
X = x0;
v = [];
w = [];
  Erro(:,1) =  - x0;


Xm = x0;

for k = 1:round(Tsim/h)-N;
    
    % Algoritmo do nepsac
    if k ==1
    %    Ubase = Ur(:,k:N+k-1);    % [v;w]
    Ubase = [0.3*ones(1,N);zeros(1,N)];  % Epsac melhorado
    else
        Ubase = [Ubase(:,2:end),Ubase(:,end)];    % [v;w]
    end
    Uopt = 1;
    contador = 0;
    while sum(abs(Uopt)) > 10e-3; % PreciSão
        contador = contador +1;
        % cálculo da saída base
        pert = (x0-Xm);
        %pert = [0;0;0];
        
        for j = 1:N;
            if j ==1
                Xbase(1,j) = x0(1) + Ts*Ubase(1,j)*cos(x0(3));%+Ts*pert(1);
                Xbase(2,j) = x0(2) + Ts*Ubase(1,j)*sin(x0(3));%+Ts*pert(2);
                Xbase(3,j) = x0(3) + Ts*Ubase(2,j);%+Ts*pert(3);
            else
                Xbase(1,j) = Xbase(1,j-1) + Ts*Ubase(1,j)*cos(Xbase(3,j-1));%+pert(1);
                Xbase(2,j) = Xbase(2,j-1) + Ts*Ubase(1,j)*sin(Xbase(3,j-1));%+pert(2);
                Xbase(3,j) = Xbase(3,j-1) + Ts*Ubase(2,j);%+pert(3);
            end
        end
        
        dUv = 0.00001;
        dUw = 0.00001;
        
        for j = 1:N;
            if j ==1
                %  V
                Xv(1,j) = x0(1) + Ts*(Ubase(1,j)+dUv)*cos(x0(3));
                Xv(2,j) = x0(2) + Ts*(Ubase(1,j)+dUv)*sin(x0(3));
                Xv(3,j) = x0(3) + Ts*Ubase(2,j);
                % W
                Xw(1,j) = x0(1) + Ts*(Ubase(1,j))*cos(x0(3));
                Xw(2,j) = x0(2) + Ts*(Ubase(1,j))*sin(x0(3));
                Xw(3,j) = x0(3) + Ts*(Ubase(2,j)+dUw);
                
            else
                %  V
                Xv(1,j) = Xv(1,j-1) + Ts*Ubase(1,j)*cos(Xv(3,j-1));
                Xv(2,j) = Xv(2,j-1) + Ts*Ubase(1,j)*sin(Xv(3,j-1));
                Xv(3,j) = Xv(3,j-1) + Ts*Ubase(2,j);
                % W
                Xw(1,j) = Xw(1,j-1) + Ts*Ubase(1,j)*cos(Xw(3,j-1));
                Xw(2,j) = Xw(2,j-1) + Ts*Ubase(1,j)*sin(Xw(3,j-1));
                Xw(3,j) = Xw(3,j-1) + Ts*Ubase(2,j);
                
            end
        end
        av = (Xv-Xbase)/dUv;
        aw = (Xw-Xbase)/dUw;
        
        Gv = zeros(3*N,N);
        Gw = zeros(3*N,N);
        
        pcv = [];
        pcw = [];
        Gnep = zeros(3*N,2*N);
        nep = [];
        xbnep = [];
        for j = 1:N
            nep = [nep;[av(:,j),aw(:,j)]];
            Gnep((3*(N-j)+1):end,2*(N-j)+1:2*(N-j+1)) = nep;
            pcv = [pcv;av(:,j)];
            pcw = [pcw;aw(:,j)];
            Gv(3*(N-j)+1:end,N+1-j) = pcv;
            Gw(3*(N-j)+1:end,N+1-j) = pcw;
            xbnep = [xbnep;Ubase(:,j)];
        end
        G = [Gv,Gw];
        
        
        
        % Montar a Matriz Dinâmica de resposta ao impulso
        
        teste = 1;
        
        % Fim do nepsac
        
        
        
        %         A = [];
        %         xb = [];
        %         aux = eye(3);
        %         for j = 0:N-1
        %             Aj{j+1} = [1 0 -Ts*vr(k+j)*sin(Xr(3,k+j));
        %                 0 1  Ts*vr(k+j)*cos(Xr(3,k+j));
        %                 0 0  1];
        %             Bj{j+1} = [cos(Xr(3,k+j))*Ts  0;
        %                 sin(Xr(3,k+j))*Ts  0;
        %                 0                Ts];
        %             aux = Aj{j+1}*aux;
        %             A = [A;aux];
        %             xb = [xb;Ur(:,k+j)];
        %         end
        %
        %         B = [];
        %
        %         for j = 1:N
        %             aux2  = eye(3);
        %             Baux = [];
        %             for i = 1:j
        %                 if i<j
        %                     Baux = [aux2*Bj{j-i+1},Baux];
        %                     aux2 = aux2*Aj{j-i+1};
        %                 else
        %                     Baux = [aux2*Bj{j-i+1},Baux,zeros(3,(N-j)*2)];
        %                 end
        %             end
        %             B = [B;Baux];
        %
        %         end
        %
        %         H = 2*(B'*Qb*B+Rb);
        %         f = 2*B'*Qb*A*(X(:,k)-Xr(:,k));
        %Uopt = quadprog(H,f,[],[],[],[],[lb-xb],[ub-xb]);
        
        Hnep = 2*(Gnep'*Qb*Gnep+Rb);
        XbXr = [];
        UbUr = [];
        for j = 1:N
            XbXr = [XbXr;(Xbase(:,j)-Xr(:,k+j))];
            UbUr = [UbUr;Ubase(:,j)-Ur(:,k+j-1)]; % EPSAC MELHORADO
        end
        fnep = 2*(Gnep'*Qb*XbXr +Rb*UbUr);   % EPSAC MELHORADO
        options = optimset('Display','off','Largescale','off');
        %Uopt = quadprog(Hnep,fnep,[],[],[],[],[lb-xbnep],[ub-xbnep],[],options);
       
        Uopt = -inv(Hnep)*fnep;
        u01 = [];
        for j = 1:N
            u01 = [u01, Uopt(2*(j-1)+1:2*j)];
        end
        Ubase= Ubase+u01;         %   Epsac Melhorado
        %Ubase = Ur(:,k:N+k-1);
        Ulinear(k) = Uopt'*Uopt;
        Uopt_aux = Uopt;
         Uopt =0;                % com zero é o EPSAC
    end
    
     
    custo(k) = contador;
    U(:,k) = Ubase(:,1)+Uopt_aux(1:2,1);
     U(:,k) = Ubase(:,1);                    % Epsac Melhorado
    %U(:,k) = Ur(:,k)+Uopt(1:2,1);
    v(k) = U(1,k);
    w(k) = U(2,k);
    
    vr(k) = Ur(1,k);
    wr(k) = Ur(2,k);
    
    vt = v(k) - vr(k);
    wt = w(k) - wr(k);
    
    
        dado = rel;
 % saturaçao......  
    if v(k) >=0.4*dado
        v(k)=0.4*dado;
    end
    
    if v(k)< -0.4*dado
        v(k) = -0.4*dado;
    end
    
    if w(k)>= 0.4*dado;
        w(k) = 0.4*dado;
    end
    
    if w(k)<-0.4*dado;
        w(k)=-0.4*dado;
    end
    
     k1 = h*[v(k)*cos(x0(3)); v(k)*sin(x0(3));w(k)];
     k2 = h*[v(k)*cos(x0(3)+k1(3)/2); v(k)*sin(x0(3)+k1(3)/2);w(k)];
     k3 = h*[v(k)*cos(x0(3)+k2(3)/2); v(k)*sin(x0(3)+k2(3)/2);w(k)];
     k4 = h*[v(k)*cos(x0(3)+k3(3)); v(k)*sin(x0(3)+k3(3));w(k)];
    
     X = [X, X(:,k) + (1/6)*(k1+2*k2+2*k3+k4)];
     x0 = X(:,k+1);
    %cálculo da saída do modelo
    
%     Xm(1,1) = x0(1) + Ts*v(k)*cos(x0(3));
%     Xm(2,1) = x0(2) + Ts*v(k)*sin(x0(3));
%     Xm(3,1) = x0(3) + Ts*w(k);
%     X = [X,Xm];
%     x0 =Xm;
   
    
    Erro(:,k+1) = Xr(:,k+1)- X(:,k+1);
    
     er1 = Erro(1,k);
     er2 = Erro(2,k);
     er3 = Erro(3,k);
    
    J{caso}(k) = er1*er1*Q(1,1)+er2*er2*Q(2,2)+er3*er3*Q(3,3)+vt*vt*R(1,1) +wt*wt*R(2,2); 
    J2{caso}(k) = er1*er1*Q(1,1)+er2*er2*Q(2,2)+er3*er3*Q(3,3); 
    J3{caso}(k) = er1*er1*Q(1,1)+er2*er2*Q(2,2); 

end
custo_total = toc

if caso <5
        figure(1)
        hold on
        plot(X(1,:),X(2,:),'b--','linewidth',2)
        %legend('referência','trajetória do robô')
%         title('EPSAC-Sub')
%         xlabel('x[m]')
%         ylabel('y[m]')
        
        figure(2)
        hold on
        ne = length(Erro);
        tempo = Ts*(0:ne-1);
        
        hold on
        plot(tempo(1:end-1),v)
        hold on
        plot (tempo(1:end-1),w)
        
        figure(3)
        hold on
        E1 = Erro(1,:);
        E2 = Erro(2,:);
        E3 = Erro(3,:);
        
        plot(tempo,E1,tempo,E2,tempo,E3);
    else
        figure(1)
        hold on
        plot(X(1,:),X(2,:),'k','linewidth',2)
        %legend('referência','trajetória do robô')
%         title('EPSAC-Sub')
%         xlabel('x[m]')
%         ylabel('y[m]')
        
        figure(2)
        hold on
        ne = length(Erro);
        tempo = Ts*(0:ne-1);
        
        hold on
        plot(tempo(1:end-1),v,'--')
        hold on
        plot (tempo(1:end-1),w,'--')
        
        figure(3)
        hold on
        E1 = Erro(1,:);
        E2 = Erro(2,:);
        E3 = Erro(3,:);
        plot(tempo,E1,'--',tempo,E2,'--',tempo,E3,'--');
    end

end

legend('x','y','theta')

 E1 = Erro(1,:);
 E2 = Erro(2,:);
 E3 = Erro(3,:);
ERRO_TOTAL = E1*E1'*Q(1,1)+E2*E2'*Q(2,2)+E3*E3'*Q(3,3);
grid
figure

Custo = [sum(J{1}),sum(J{2}),sum(J{3}),sum(J{4}),sum(J{5}),sum(J{6}),sum(J{7}),sum(J{8})] 
Custo2 = [sum(J2{1}),sum(J2{2}),sum(J2{3}),sum(J2{4}),sum(J2{5}),sum(J2{6}),sum(J2{7}),sum(J2{8})] 
Custo3 = [sum(J3{1}),sum(J3{2}),sum(J3{3}),sum(J3{4}),sum(J3{5}),sum(J3{6}),sum(J3{7}),sum(J3{8})] 

figure(1)

        title('EPSAC-Sub','fontsize',14,'fontname','times new roman')
        xlabel('x[m]','fontsize',14,'fontname','times new roman')
        ylabel('y[m]','fontsize',14,'fontname','times new roman')
        legend('reference','q = 0.5','q = 0.1','Location','SouthWest')
        set(gca,'fontsize',12)

%plot(tempo,J{1})
