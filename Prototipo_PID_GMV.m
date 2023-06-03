%% Projeto GMV Incremental
% Engenharia de Controle de Processos

%clc; clear all; close all
clear

%% Condições Iniciais
nit = 300; ts = 0.01;
angulo_sensor = zeros(1,nit); 

%% Referência e Pertubação
angulo_ref = 50*ones(1,nit);
% mudref1 = 150;
% mudref2 = 300;
% angulo_ref(1:mudref1) = 50; 
% angulo_ref(mudref1+1:mudref2) = 50; 
% angulo_ref(mudref2+1:nit) = 50;


%% ----- Variável Controlada
pot_motor_1 = zeros(1,nit);
pot_motor_2 = zeros(1,nit);
delta_pot_motor_1 = zeros(1,nit);
delta_pot_motor_2 = zeros(1,nit);
erro = zeros(1,nit);
u = strings(1,nit); u(1:nit) = "0,0";

% Maxima e minima potência
max_pot = 15;
min_pot = 7;

% Iniciar o Protótipo
start = input("Start Daqduino? ","s");
if start == "y"
    daqduino_start('COM9'); % Starts DaqDuino board connected to COM7
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

%% Projeto do GMV motor 1
d = 1;
na1 = 1;
nb1 = 0;
ns1 = na1;
ne1 = d-1;

a21 = 0;

% Determinar E e S
e01 = 1;
s01 = -(a1m1-1)*e01;
s11 = -(a21-a1m1)*e01;
s21 = a21*e01;

% Lei de Controle GMV
% [B(z^-1)B(z^-1) + B(z^-1)]* DeltaU = B(z^-1)yr(k) - B(z^-1)y(k)

% T(z^-1) = t0 = P(1)
t01 = 1;

% Q(z^-1) = q0/b0
q01 = 0.1;

% q0 do PID
que01 = 1/(b0m1+q01);

% Determinar polinômio R(z^-1)
r0 = b0m1+q01;
r1 = 0;

% Sintonia do PID

Kc1 = -que01*(s11+2*s21);
Ti1 = -(s11+2*s21)*ts/(s11 + s01 + s21);
Td1 = -s21*ts/(s11 + 2*s21);

% q10 =  Kc1*(1 + ts/Ti1 + Td1/ts);
% q11 = -Kc1*(1 + 2*Td1/ts);
% q12 =  Kc1*Td1/ts;


%% Projeto do GMV motor 2


na2 = 1;
nb2 = 0;
ns2 = na2;
ne2 = d-1;
a22 = 0;

% Determinar E e S
e02 = 1;
s02 = -(a1m2-1)*e02;
s12 = -(a22-a1m2)*e02;
s22 = a22*e02;

% Lei de Controle GMV
% [B(z^-1)B(z^-1) + B(z^-1)]* DeltaU = B(z^-1)yr(k) - B(z^-1)y(k)

% T(z^-1) = t0 = P(1)
t02 = 1;

% Q(z^-1) = q0/b0
q02 = 0.1;

% q0 do PID
que02 = 1/(b0m2+q02);

% Determinar polinômio R(z^-1)
r02 = b0m1+q02;
r12 = 0;

% Sintonia do PID

Kc2 = -que02*(s12+2*s22);
Ti2 = -(s12+2*s22)*ts/(s12 + s02 + s22);
Td2 = -s22*ts/(s12 + 2*s22);

ajuste_ki1 = 0.05;
ajuste_kd1 = 0.1;

ajuste_ki2 = 0.05;
ajuste_kd2 = 0.1;

Ki1 = ajuste_ki1*ts*Kc1/Ti1;
Kd1 = ajuste_kd1*(Kc1*Td1)/ts;

Ki2 = ajuste_ki2*ts*Kc2/Ti2;
Kd2 = ajuste_kd2*(Kc2*Td2)/ts;

% Coeficientes do PID estrutura paralela motor 1
q10 = Kc1 + Kd1;
q11 = -Kc1 - 2*Kd1 +Ki1;
q12 = Kd1;

% Coeficientes do PID estrutura paralela motor 2
q20 = Kc2 + Kd2;
q21 = -Kc2 - 2*Kd2 + Ki2;
q22 = Kd2;

% q20 =  Kc2*(1 + ts/Ti2 + Td2/ts);
% q21 = -Kc2*(1 + 2*Td2/ts);
% q22 =  Kc2*Td2/ts;


%% Processamento 

limpar = input("Limpar memória? ","s");
if limpar == "y"

    % Limpar Serial
    daqduino_read;
    u0 = [num2str(0),',',num2str(0),'\n'];
    daqduino_write(u0,ts);

    daqduino_read;
    daqduino_write(u0,ts);
end

for k = 3:nit
     
    % ----- Saída da planta
    angulo_sensor(k) = daqduino_read;

    erro(k) = angulo_ref(k) - angulo_sensor(k);

    pot_motor_1(k) = pot_motor_1(k-1) + q10*erro(k) + q11*erro(k-1) + q12*erro(k-2);
    pot_motor_2(k) = pot_motor_2(k-1) -(q20*erro(k) + q21*erro(k-1) + q22*erro(k-2));

    % -------- Saturações de potência

    % Saturação
    if pot_motor_1(k)> 15
        pot_motor_1(k) = 15;
    elseif pot_motor_1(k)< 7
        pot_motor_1(k) = 7;

    end

    if pot_motor_2(k)> 15
        pot_motor_2(k) = 15;
    elseif pot_motor_2(k)< 7
        pot_motor_2(k) = 7;

    end
    

    % Mandar sinal de controle para os Motores
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
set(gcf,'position',[50 100 800 800])
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
    ylabel("Ângulo (º)")
    xlabel("Tempo (s)")

    legend('Real','Referência')

end

% Potência dos Motores
figure(2)
set(gcf,'position',[900 100 800 800])
subplot(211)
plot(t,pot_motor_1)
yticks(6:16)
ylabel("PWM Bandwidth (%)")
xlabel("Tempo (s)")
ylim([6,16])
grid
title(['Potência do Motor 1 - ' metodo ajuste])

subplot(212)
plot(t,pot_motor_2)
ylim([6,16])
yticks(6:16)
ylabel("PWM Bandwidth (%)")
xlabel("Tempo (s)")
grid
title(['Potência do Motor 2 - ' metodo ajuste])

