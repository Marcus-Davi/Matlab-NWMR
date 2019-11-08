function [sys, x0, str, ts] = Nepsac_DeltaU(t,x,u,flag,ts)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

%inicialização
if flag == 0
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 2;
    sizes.NumInputs      = 9;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;
    
%     sys = [0.3; 0];
    sys = simsizes(sizes);

    x0 = [ ];
    str = [ ];
    ts = [0.1]; %tempo de amostragem variável
    
    %Calcula próximo instante de amostragem
elseif flag == 4
    sys=[];
    
elseif flag == 3
    
        % Put theta in range [-pi,pi]
    tmed = u(3);
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
    
    y = [u(1);u(2);tmed]; %% medição, entrada de parâmetro
    k = round(u(4));
    xo = [u(7);u(8);u(9)];
    
    N = 5; % Horizonte de predição
    Nu = 1; % Horizonte de controle
    
    % saturation
    vmax = 0.4;
    vmin = 0;
    wmax = 0.4;
    wmin = -wmax;
    
    umin = [];
    umax = [];
    
    for ba = 1:Nu;
        umin = [umin;[vmin;wmin]];
        umax = [umax;[vmax;wmax]];
    end
    
    % weighting matrices
    R = 0.01*eye(2*Nu);
    %R(1,1) = 0; R(2,2) = 0;
    Q = diag([1 1 0.01]);
    Qcell = repmat({Q},1,N);
    Q = blkdiag(Qcell{:});
    
    % M matrix
    l = ones(1,2*Nu);
    M = diag(l);
    
    for na = 1:(2*Nu-2);
        M(na+2,na)=-1;
    end;
    
    % load reference
    load ('ref.mat');
    load ('ubase.mat');
    
    % account for extended prediction
    for l = 1:N
        ref = [ref ref(:,end)];
        ubase = [ubase [0 0]'];
    end
    xr = ref(1,:); 
    yr = ref(2,:); 
    tr = ref(3,:);
    
    % other definitions
    du = 0.00001;
    
    % limite de atualização
    ro = 0.001;
    
    %nada haver com o controlador. Próximo passo é determinar resposta base
    ent = [u(5);u(6)];

    x=modelo(ent,xo); % (CALCULADA)
    n = y - x;
    
    % Base update
    if k > 1
        urefp = ubase(:,k-1);
    end
    
    uref = ubase(:,k:N+k-1);
    
    ub = zeros(2,N);
    ub(1,:) = 0.3; % lembrar de verificar
    ub(2,:) = 0; % lembrar de verificar
    
    vbd = ub;
    vbd(1,1) = vbd(1,1) + du;
    wbd = ub;
    wbd(2,1) = wbd(2,1) + du;
    
    Yb = ybase(ub,xo,n);
    xb = Yb(1,:);
    yb = Yb(2,:);
    tb = Yb(3,:);
    
    % counter
    aux = ro+0.1*ro;
    
    while (aux > ro)
        
        % G matrix
        % impulse response
        xbv = ybase(ub,xo,n); xbv = reshape(xbv,[3*N,1]);
        xbvd = ybase(vbd,xo,n); xbvd = reshape(xbvd,[3*N,1]);
        xbw = ybase(ub,xo,n); xbw = reshape(xbw,[3*N,1]);
        xbwd = ybase(wbd,xo,n); xbwd = reshape(xbwd,[3*N,1]);
        
        % for v
        gv = (xbvd-xbv)/du;
        
        % for w
        gw = (xbwd-xbw)/du;
        
        %
        Gnep = zeros(3*N,2*N);
        nep = [];
        av = reshape(gv,[3,N]);
        aw = reshape(gw,[3,N]);
        
        for j = 1:N
            nep = [nep;[av(:,j),aw(:,j)]];
            Gnep((3*(N-j)+1):end,2*(N-j)+1:2*(N-j+1)) = nep;
        end
        
        % for Nu different than N
        if (N ~= Nu);
            % step response
            avg = av(:,1); awg = aw(:,1);
            for l = 2:N
                avg = [avg avg(:,l-1)+av(:,l)];
                awg = [awg awg(:,l-1)+aw(:,l)];
            end
            avg = reshape(avg,[3*N,1]);
            awg = reshape(awg,[3*N,1]);
            Gnep = Gnep(:,1:2*Nu);
            Gnep(:,end-1) = [zeros(3*(Nu-1),1);avg(1:end-3*(Nu-1))];
            Gnep(:,end) = [zeros(3*(Nu-1),1);awg(1:end-3*(Nu-1))];
        end
        
        % reference errors
        errx = (xb-xr(:,k:N+k-1))';
        erry = (yb-yr(:,k:N+k-1))';
        errt = (tb-tr(:,k:N+k-1))';
        errt = atan2(sin(errt), cos(errt)); % smooth the error
        
        E = [];
        for s = 1:length(errx)
            E = [E errx(s) erry(s) errt(s)];
        end
        E = E';
        
        % cost function
        
        Hnep = 2*(Gnep'*Q*Gnep+M'*R*M);
        xbnep = reshape(ub(:,1:Nu),[2*Nu,1]);
        UbUr = xbnep-reshape(uref(:,1:Nu),[2*Nu,1]);
        
        if k>1
            L = [urefp-ent; zeros(2*Nu,1)];
        else
            L = [-ent; zeros(2*Nu,1)];
        end
        
        L = L(1:2*Nu,:); % account for difference btw Nu and N
        Ku = M*UbUr + L;    % mgn
        Fnep = 2*(Gnep'*Q*E + M'*R*Ku);   % mgn
        
        % analytical solution
        uo = -Hnep\Fnep;
        
        % quadprog solution
        %options = optimset('Display','off','Largescale','off');
        %uo = quadprog(Hnep,Fnep,[],[],[],[],[umin-xbnep],[umax-xbnep],[],options);
        
        % account for Nu < N
        uo = reshape(uo,[2,Nu]);
        [muo,nuo] = size(uo); [mub,nub] = size(ub); na = nub - nuo;
        uo = [uo [uo(1,end).*ones(1,na);uo(2,end).*ones(1,na)]]; %for when N ~= Nu
        
        % update Ubase and counter
        ub = uo+ub;
        aux = sum(uo*uo');
        aux = 0; % epsac
    end
    
    u = ub(:,1);
    
    % saturation
    u(1,1) = min(u(1,1),vmax);
    u(1,1) = max(u(1,1),vmin);
    u(2,1) = min(u(2,1),wmax);
    u(2,1) = max(u(2,1),wmin);

    % control action
    u0 = u(:,1);
    sys = u0;
    
else
    sys = [ ]; %não faz nada
end

end

