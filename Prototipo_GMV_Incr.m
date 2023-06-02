%% Projeto GMV Incremental
% Engenharia de Controle de Processos

%clc; clear all; close all

%% Condições Iniciais
nit = 500; ts = 0.01;
angulo_sensor = zeros(1,nit); 

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

% Função de transferência motor 2
Gm2 = tf(Kpsmith2,[tausmith2 1],'InputDelay',thetasmith2)

% Discretização do modelo
Gmz2 = c2d(Gm2,ts);
Bm2 = Gmz2.num{1}; Am2= Gmz2.den{1};

% Coeficientes do Modelo
b0m2 = Bm2(2); a1m2 = Am2(2);

%% Determinar E(z^-1) e S(z^-1)
% na = 1;
% nb = 0;
% ns = na;
% ne = d-1;
% E(z^-1) = e0
% S(z^-1) = s0 + s1*z^-1

% Identidade Polinomial
% P(z^-1) = A(z^-1)Delta E(z^-1) + z^-d S(z^-1)
% P(z^-1) = 1

%% Resolução por matemática simbólica motor 1

% Declarar variáveis simbólicas
syms a1 a2 s0 s1 s2 z

% Definir os polinômios
pAz = 1+ a1m1*z;
delta = 1 - z;
d = 1;
pEz = 1  ;
pSz = s0 + s1*z;

% Igualdade Polinomial P(z^-1) = A(z^-1)Delta E(z^-1) + z^-d S(z^-1)
Pr = collect( pAz * delta * pEz + z^d * pSz - 1 ,z);

% Extrair os coeficientes do polinômio simbolico
coef_Pr = coeffs(Pr,z);

% Resolver o sistema simbólico
sol_Pr = solve(coef_Pr==0,[s0,s1]);


% Determinar polinômio R(z^-1) = B(z^-1)E(z^-1) + Q(z^-1)
% E(z^-1) = e0 + e1*z^-1 + e2*z^-2 + e3*z^-3
% B(z^-1) = b0 

syms q0s b0 b1 b2 e1s e2s e3s
pEz = 1;
pBz = b0m1;
pQz = q0s;

Rz =  coeffs( collect( pBz * pEz + pQz ,z),z);

% Substituir os valores numéricos do modelo na estrutura. (ainda fica
% simbolico)
sol_Pr = subs(sol_Pr,[a1],[a1m1]);

% Transformar a função double em cada propriedade da estrutura com a função
% structfun. Retorna array.
sol_Pr_double = structfun(@double,sol_Pr);

% Transformar o array em cell para extrair cada elemento em variáveis
% diferentes. Não obrigatório, apenas para deixar mais clean.
sol_Pr_double_cell1 = num2cell(sol_Pr_double);

% Extrair cada valor em uma variável
%[s0,s1,s2] = sol_Pr_double_cell1{:};
pS1 = sol_Pr_double';

% Determinar polinômio R(z^-1)

% T(z^-1) = t0 = P(1)
t01 = 1;

% Q(z^-1) = q0
q01 = 5;

Rz_subs = subs(Rz,[q0s b0m1],[q01 b0m1]);
Rcoef1 = double(Rz_subs)


%% Resolução por matemática simbólica motor 2

% Declarar variáveis simbólicas
syms a1 a2 s0 s1 s2 z

% Definir os polinômios
pAz = 1+ a1m2*z;
delta = 1 - z;
d = 1;
pEz = 1  ;
pSz = s0 + s1*z;

% Igualdade Polinomial P(z^-1) = A(z^-1)Delta E(z^-1) + z^-d S(z^-1)
Pr = collect( pAz * delta * pEz + z^d * pSz - 1 ,z);

% Extrair os coeficientes do polinômio simbolico
coef_Pr = coeffs(Pr,z);

% Resolver o sistema simbólico
sol_Pr = solve(coef_Pr==0,[s0,s1]);


% Determinar polinômio R(z^-1) = B(z^-1)E(z^-1) + Q(z^-1)
% E(z^-1) = e0 + e1*z^-1 + e2*z^-2 + e3*z^-3
% B(z^-1) = b0 

syms q0s b0 b1 b2 e1s e2s e3s
pEz = 1;
pBz = b0m2;
pQz = q0s;

Rz =  coeffs( collect( pBz * pEz + pQz ,z),z);

% Substituir os valores numéricos do modelo na estrutura. (ainda fica
% simbolico)
sol_Pr = subs(sol_Pr,[a1],[a1m2]);

% Transformar a função double em cada propriedade da estrutura com a função
% structfun. Retorna array.
sol_Pr_double = structfun(@double,sol_Pr);

% Transformar o array em cell para extrair cada elemento em variáveis
% diferentes. Não obrigatório, apenas para deixar mais clean.
sol_Pr_double_cell2 = num2cell(sol_Pr_double);

% Extrair cada valor em uma variável
%[s0,s1,s2] = sol_Pr_double_cell{:};
pS2 = sol_Pr_double';

% Determinar polinômio R(z^-1)

% T(z^-1) = t0 = P(1)
t02 = 1;

% Q(z^-1) = q0
q02 = 5;

Rz_subs = subs(Rz,[q0s b0m2],[q02 b0m2]);
Rcoef2 = double(Rz_subs)


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

for k = 3:nit
     
    % ----- Saída da planta
    angulo_sensor(k) = daqduino_read;

    % Sinal de controle GMV
    delta_pot_motor_1(k) =  (t01*angulo_ref(k) - (pS1(1)*angulo_sensor(k-1) + pS1(2)*angulo_sensor(k-2)))/Rcoef1(1);
    delta_pot_motor_2(k) = (t02*angulo_ref(k) - (pS2(1)*angulo_sensor(k-1) + pS2(2)*angulo_sensor(k-2)))/Rcoef2(1);

    pot_motor_1(k) = pot_motor_1(k-1) + delta_pot_motor_1(k);
    pot_motor_2(k) = pot_motor_2(k-1) - delta_pot_motor_2(k);

      %% -------- Saturações de potência

    if pot_motor_1(k)> 15
        pot_motor_1(k) = 15;
    elseif pot_motor_1(k)< 7
        pot_motor_1(k) = 7;

    end

    % Saturações de potência
    if pot_motor_2(k)> 15
        pot_motor_2(k) = 15;
    elseif pot_motor_2(k)< 7
        pot_motor_2(k) = 7;

    end

    u(k) = [num2str(pot_motor_1(k)),',',num2str(pot_motor_2(k)),'\n'];
    daqduino_write(u(k),ts);

    if(angulo_sensor(k)<=0 || angulo_sensor(k)>90)
        angulo_sensor(k) = angulo_sensor(k-1);  % Tratar os dados errados
    end

end



% Limpar a Serial
daqduino_read
u0 = [num2str(0),',',num2str(0),'\n'];
daqduino_write(u0,ts);


%% Resultados
metodo = 'GMV';
ajuste = '';

t = 0:ts:(nit-1)*ts;
figure(1)
p = plot(t,angulo_sensor,'r',t,angulo_ref,'--k');grid
title(["Controle " metodo ajuste])

ylim([0,90])

indices = input("Calcular indices? ","s");
if indices == "y"
    % Settling criteria
    desired_range = [0.95*angulo_sensor(end), 1.05*angulo_sensor(end)];  % Desired steady-state range
    tolerance = 0.01;             % Tolerance for settling range

    % Find the indices where the response is within the desired range
    settling_indices = find(angulo_sensor > desired_range(1) & angulo_sensor < desired_range(2));

    % Calculate the settling time
    settling_time = t(settling_indices(1)) - t(1);

    % Rise time calculation

    rise_9_index = find(angulo_sensor>=0.9*angulo_sensor(end));
    rise_1_index = find(angulo_sensor >=0.1*angulo_sensor(end));
    rise_time = t(rise_9_index(1)) - t(rise_1_index(1));

    % Plot the settling time point
    hold on;


    plot(t(settling_indices(1)), angulo_sensor(settling_indices(1)), 'b.','MarkerSize',20);
    text(t(settling_indices(1)), angulo_sensor(settling_indices(1))+5,rise_time, ['Settling Time = ', num2str(settling_time)],...
        'VerticalAlignment', 'bottom','HorizontalAlignment','center','Color','b');
    % Plot the rise time point

    plot(t(rise_9_index(1)), angulo_sensor(rise_9_index(1)), 'k.','MarkerSize',20);
    text(t(rise_9_index(1))+0.2, angulo_sensor(rise_9_index(1))+0.5, ['Rise Time = ', num2str(rise_time)], 'VerticalAlignment', 'top','HorizontalAlignment','left');

    legend('Real','Referência')

end

% Potência dos Motores
figure(2)
subplot(211)
plot(t,pot_motor_1)
ylim([6,16])
grid
title(['Potência do Motor 1 - ' metodo ajuste])

subplot(212)
plot(t,pot_motor_2)
ylim([6,16])
grid
title(['Potência do Motor 2 - ' metodo ajuste])

