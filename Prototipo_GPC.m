%% ***********************************************************************
%	Controlador Preditivo Generalizado GPC
%   Planta Benchmark 10 - Planta instável
% ***********************************************************************

clear all; clc; close all

%% ----- Condições inciais
nit = 300; Ny = 6; Nu = 1; lambda = 0.1; ts = 0.1; 
y(1:nit) = 0; yp(1:nit) = 0; u(1:nit) = 0; du(1:nit) = 0; 
e(1:nit) = 0; fi(1:nit) = 0;
I = eye(Nu); G = zeros(Ny,Nu); g = zeros(1,Ny); 
G_aux = zeros(Ny,Nu); Gt = zeros(Ny,Ny);
    
%% ----- Discretização da planta
s = tf('s');
Gp = 1/(s^2+0.0023*s-1.034);
Gpd = c2d(Gp,ts);
A = Gpd.den{1}; B = Gpd.num{1};
na = length(A) - 1; nb = length(B) - 1;

%% ----- Discretização do Modelo
Gm = 1/(s^2-1);

Gmd = c2d(Gm,ts);
Am = Gmd.den{1}; Bm =  Gmd.num{1};

nbm = length(Bm)-1; nam = length(Am)-1;
delta = [1 -1];
Atil = conv(Am,delta); natil = length(Atil);


%% ----- Referência ----- Degrau
ref(1:100) = 1; ref(101:200) = 2.5; ref(201:nit+Ny) = 4.5; 

%% ----- Perturbação
d0(1:150) = 0; d0(151:nit) = 0.1;
%d0(1:nit) = rand(1,nit)*0.01;

%% Cálculo dos matrizes do preditor (F, H, G)
% primeira equação diophantine
rr = [1 zeros(1,natil-1)];
E = zeros(1,Ny); % polinômio E (gradu ne = j-1)

for k = 1:Ny
    [q(k),r] = deconv(rr,Atil);
    F(k,:) = r(2:end);
    rr = [r(2:end) 0];
end

for j = 1:Ny
    if (j==1)
        aux = [q(j) zeros(1,Ny-j)]; % elementos passados, u(t-1)...u(t-1-nb) :)
    else
        aux = [q(1:j) zeros(1,Ny-j)];
    end
    E(j,:) = aux;
end

% segunda equação diophantine
B_aux = conv(Bm,E(end,:)); nb_aux = length(B_aux);
T_BE = [1 zeros(1,nb_aux-1)]; % polinômio C

rr_BE = B_aux;

for k = 1:Ny
    [q_BE,r_BE] = deconv([rr_BE(2:end) 0],T_BE);
    G_aux(k) = q_BE;
    rr_BE = r_BE;
end

 for i = 1:Nu
     G(i:end,i) = G_aux(1:end-i+1,1);
 end
 
 for i = 1:Ny
    Haux(i,:) = conv(E(i,:),[Bm(2:end) zeros(1,Ny)]);
end

H = zeros(Ny,1);

for j = 1:Ny
    H(j,:)= Haux(j,j+1:j+1+nbm-1-1); % elementos passados, u(t-1)...u(t-1-nb) :)
end 

gt = (inv(G'*G + lambda*I))*G';
Kgpc = gt(1,:);

%% Cálculo dos elementos do sinal de controle R, S e T
% Polinômio R

R = [1 Kgpc*H];  % polinômio que afeta a ação de cotnrole passada du(t-1)

% Polinômio S
S = Kgpc*F;    % polinômio que afeta as saídas preditas

% Polinômio T
T = sum(Kgpc);  % polinômio que afeta o sinal de entrada

ns = length(S);

% Cálculo dos Coeficientes do PID Continuo
t0 = sum(S);

% Kc
Kc = S(1) - t0 - S(2);

% Ti
Ti = (S(1) - t0 - S(2))/t0;

% Td
Td = S(2)*ts/(S(1)-S(2)-t0);

q0 = Kc*ts/Ti;
q1 = -Kc*(Td/ts+ts/Ti+1);
q2 = Kc*(2*Td/ts+1);
q3 = -Kc*Td/ts;
 
%% Calculando os sinais de controle a partir do preditor e saída
tbegin = 1 + max([1+nb,1+na]);

for k = tbegin:nit
    yp(k) = -A(2:end)*yp(k-1:-1:k-2)' + B(2:end)*u(k-1:-1:k-2)'; 
    y(k) = yp(k) + d0(k);
    
    ent_fut = 0;

    if (ent_fut==0)
        % ---- Referência futura desconhecida
        aux_ref = ref(k)*ones(1,Ny)';
    elseif(ent_fut==1)
        % ---- Referência futura conhecida
        aux_ref = ref(k:k+Ny-1)';
    end
    
      fi = H*du(k-1) + F(1:end,:)*y(k:-1:k-nam)';
    
    du(k) = Kgpc*(aux_ref - fi);
    
    u(k) = u(k-1) + du(k);
    e(k) = ref(k) - y(k);
end


%%
figure(1)
t = 0:ts:(nit-1)*ts;
subplot(211), plot(t,ref(1:nit),'--k',t,y,'r','linewidth',2), grid on; hold on
ylabel('Reference and Output'), xlabel('Time (s)');
subplot(212), plot(t,u,'b','linewidth',2), grid on; hold on, %grid on
ylabel('Control'), xlabel('time (s)');

