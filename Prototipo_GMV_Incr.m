%% Projeto GMV Incremental
% Engenharia de Controle de Processos

%clc; clear all; close all

%% Condições Iniciais
nit = 1200; ts = 0.01;
angulo_sensor = zeros(1,nit); 
angulo_model1 = zeros(1,nit); 
angulo_model2 = zeros(1,nit); 
erro1 = zeros(1,nit);
erro2 = zeros(1,nit);
d1 = zeros(1,nit);

d1(1:150) = 0; d1(350:nit) = 0.5; d0(1:nit) = rand(1,nit)*0.1;


%% Referência e Pertubação
angulo_ref = 50*ones(1,nit);
mudref1 = 150;
mudref2 = 300;
% angulo_ref(1:mudref1) = 50; 
% angulo_ref(mudref1+1:mudref2) = 50; 
% angulo_ref(mudref2+1:nit) = 50;


%% ----- Variável Controlada
pot_motor_1 = zeros(1,nit);
pot_motor_2 = zeros(1,nit);
delta_pot_motor_1 = zeros(1,nit);
delta_pot_motor_2 = zeros(1,nit);
u = strings(1,nit); u(1:nit) = "0,0";


% Iniciar o Protótipo
start = input("Start Daqduino? ","s");
if start == "y"
    daqduino_start('COM6'); % Starts DaqDuino board connected to COM7
end

%% Planta e Modelo

%Coefiientes do Modelo Smith motor 1
Kpsmith1 = 7.737;
thetasmith1 =0.65;
tausmith1 =0.6;

% Coefiientes do Modelo Smith motor 2
Kpsmith2 =  12.86;
thetasmith2 = 0.5;
tausmith2 =   0.66;

% Função de transferência motor 1
Gm1 = tf(Kpsmith1,[tausmith1 1],'InputDelay',thetasmith1)

% Discretização do modelo
Gmz1 = c2d(Gm1,ts);
Bm1 = Gmz1.num{1}; Am1= Gmz1.den{1};

% Coeficientes do Modelo
b0m1 = Bm1(2); a1m1 = Am1(2);



%% Determinar E(z^-1) e S(z^-1)
% na = 2;
% nb = 2;
% ns = na;
% ne = d-1;
% E(z^-1) = e0
% S(z^-1) = s0 + s1*z^-1 + s2*z^-2

% Identidade Polinomial
% P(z^-1) = A(z^-1)Delta E(z^-1) + z^-d S(z^-1)
% P(z^-1) = 1

%% Resolução por matemática simbólica

% Declarar variáveis simbólicas
syms a1 a2 s0 s1 s2 z
% Definir os polinômios
pAz = 1+ a1*z + a2*z^2;
delta = 1 - z;
d = 1;
pEz = 1  ;
pSz = s0 + s1*z + s2*z^2;

% Igualdade Polinomial P(z^-1) = A(z^-1)Delta E(z^-1) + z^-d S(z^-1)
Pr = collect( pAz * delta * pEz + z^d * pSz - 1 ,z);

% Extrair os coeficientes do polinômio simbolico
coef_Pr = coeffs(Pr,z);

% Resolver o sistema simbólico
sol_Pr = solve(coef_Pr==0,[s0,s1,s2]);


% Determinar polinômio R(z^-1) = B(z^-1)E(z^-1) + Q(z^-1)
% E(z^-1) = e0 + e1*z^-1 + e2*z^-2 + e3*z^-3
% B(z^-1) = b0 + b1*z^-1 + b2*z^-2

syms q0s b0 b1 b2 e1s e2s e3s
pEz = 1;
pBz = b0 + b1*z;
pQz = q0s;

Rz =  coeffs( collect( pBz * pEz + pQz ,z),z);

% Substituir os valores numéricos do modelo na estrutura. (ainda fica
% simbolico)
sol_Pr = subs(sol_Pr,[a1 a2],[a1m a2m]);

% Transformar a função double em cada propriedade da estrutura com a função
% structfun. Retorna array.
sol_Pr_double = structfun(@double,sol_Pr);

% Transformar o array em cell para extrair cada elemento em variáveis
% diferentes. Não obrigatório, apenas para deixar mais clean.
sol_Pr_double_cell = num2cell(sol_Pr_double);

% Extrair cada valor em uma variável
%[s0,s1,s2] = sol_Pr_double_cell{:};
pS = sol_Pr_double';

% Determinar polinômio R(z^-1)

% T(z^-1) = t0 = P(1)
t0 = 1;

% Q(z^-1) = q0
q0 = 5;

Rz_subs = subs(Rz,[q0s b0 b1],[q0 b0m b1m]);
Rcoef = double(Rz_subs)


%% Processamento 

limpar = input("Limpar memória? ","s");
if limpar == "y"

    % Limpar Serial
    daqduino_read
    u0 = [num2str(0),',',num2str(0),'\n'];
    daqduino_write(u0,ts);

    daqduino_read
    daqduino_write(u0,ts);
end

for k = 6:nit
    % Sinal de saída

    yp(k) = -A(2:end)*yp(k-1:-1:k-2)' + B(2:end)*uInterno(k-1:-1:k-2)';
    y(k) = yp(k)+ d0(k) + d1(k);

    % Sinal de controle
    deltaPot(k) = (-Rcoef(2:end)*deltaPot(k-1) + t0*angulo_ref(k) - pS*y(k:-1:k-2)')/Rcoef(1);

    uExterno(k) = uExterno(k-1) + deltaPot(k);

    % Erro na malha interna
    erroInterno(k) = uExterno(k) - y(k);

    % Sinal de controle interno
    uInterno(k) = -a1comp*uInterno(k-1) + (b0comp)*erroInterno(k) + b1comp*erroInterno(k-1) ;


end

%% Resultados
t = (0:nit-1)*Ts;

subplot(311)
h = plot(t,angulo_ref,'--k',t,y,'r','LineWidth',1); grid on
h(1).LineWidth = 1;
title('Resposta do Pêndulo Invertido')
h1 = subplot(312),plot(t,uInterno,'m'),grid on
title('Sinal de controle Interno (Compensador)')


h2 = subplot(313), plot(t,uExterno,'b'); grid on
h2.YLim = h1.YLim;
title('Sinal de controle Externo (GMV - Incremental)')




