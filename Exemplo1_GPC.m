%% ***********************************************************************
%	Controlador Preditivo Generalizado 
% ***********************************************************************

clear all; clc; close all

%% ----- Condi��es inciais
nit = 300; Ny = 15; Nu = 1; lambda = 1; ts = 0.1; 
y(1:nit) = 0; yp(1:nit) = 0; u(1:nit) = 0; du(1:nit) = 0; 
e(1:nit) = 0; fi(1:nit) = 0;
I = eye(Nu); G = zeros(Ny,Nu); g = zeros(1,Ny); 
G_aux = zeros(Ny,Nu); Gt = zeros(Ny,Ny);
    
%% ----- Discretiza��o da planta 
A = [1.0000   -0.9835]; B = [0    0.1279];
na = length(A) - 1; nb = length(B) - 1;
a1 = A(2); 
%a2 = A(3); 
b0 = B(2); 
% b1 = B(3);

%% ----- Discretiza��o do Modelo
Bm = B; Am = A; nbm = length(Bm)-1; nam = length(Am)-1;
delta = [1 -1];
Atil = conv(Am,delta); natil = length(Atil);

%% ----- Refer�ncia ----- Degrau
ref(1:100) = 50; ref(101:200) = 80; ref(201:nit+Ny) = 20; 

%% ----- Perturba��o
d0(1:150) = 0; d0(151:nit) = 0;

%% C�lculo dos matrizes do preditor (F, H, G)
% primeira equa��o diophantine
rr = [1 zeros(1,natil-1)];
E = zeros(1,Ny); % polin�mio E (gradu ne = j-1)

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

% segunda equa��o diophantine
B_aux = conv(Bm,E(end,:)); nb_aux = length(B_aux);
T_BE = [1 zeros(1,nb_aux-1)]; % polin�mio C

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
    H(j,:)= Haux(j,j+1:j+1+nbm-1); % elementos passados, u(t-1)...u(t-1-nb) :)
end 

gt = (inv(G'*G + lambda*I))*G';
Kgpc = gt(1,:);

tbegin = 1 + max([1+nb,1+na]);
% -------- Satura��es de pot�ncia
max_pot = 15;
min_pot = 1;
    
%% Calculando os sinais de controle a partir do preditor e sa�da
for k = tbegin:nit
    yp(k) = -a1*yp(k-1) + b0*u(k-1); 
    %yp(k) = -a1*yp(k-1) - a2*yp(k-2) + b0*u(k-1) + b1*u(k-2); 
    y(k) = yp(k) + d0(k);
    
    ent_fut = 1;
    if (ent_fut==0)
        % ---- Refer�ncia futura desconhecida
        aux_ref = ref(k)*ones(1,Ny)';
    elseif(ent_fut==1)
        % ---- Refer�ncia futura conhecida
        aux_ref = ref(k:k+Ny-1)';
    end
    
      fi = H*du(k-1) + F(1:end,:)*y(k:-1:k-nam)';
    
    du(k) = Kgpc*(aux_ref - fi);
    
    u(k) = u(k-1) + du(k);
    %e(k) = ref(k) - y(k);
    u(k) = max(min_pot, min(u(k),max_pot));

end


%%
figure(1)
t = 0:ts:(nit-1)*ts;
subplot(211), plot(t,ref(1:nit),'--k',t,y,'r','linewidth',2), grid on; hold on
ylabel('reference and output'), xlabel('time (s)');
subplot(212), plot(t,u,'b','linewidth',2), grid on; hold on, %grid on
ylabel('control'), xlabel('time (s)');

