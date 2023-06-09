%% Projeto GMV Incremental
% Engenharia de Controle de Processos

%clc; clear all; close all
clear

%% Condi��es Iniciais
nit = 300; ts = 0.01;
angulo_sensor = zeros(1,nit); 

%% Refer�ncia e Pertuba��o
angulo_ref = 50*ones(1,nit);
% mudref1 = 150;
% mudref2 = 300;
% angulo_ref(1:mudref1) = 50; 
% angulo_ref(mudref1+1:mudref2) = 50; 
% angulo_ref(mudref2+1:nit) = 50;


%% ----- Vari�vel Controlada
pot_motor_1 = zeros(1,nit);
pot_motor_2 = zeros(1,nit);
delta_pot_motor_1 = zeros(1,nit);
delta_pot_motor_2 = zeros(1,nit);
u = strings(1,nit); u(1:nit) = "0,0";


% Iniciar o Prot�tipo
start = input("Start Daqduino? ","s");
if start == "y"
    daqduino_start('COM9'); % Starts DaqDuino board connected to COM7
end

%% Planta e Modelo
% Coeficientes da Identifica��o MQR com atualiza��o da matriz P (Random Walk)
a1  = -0.42673;
a2  = -0.41589;
b10 =  0.43853;
b11 =  0.43853;
b20 =  0.25997;
b21 =  0.25997;



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

%% Projeto do GMVC
d = 1;
na1 = 1;
nb1 = 0;
ns1 = na1;
ne1 = d-1;

f0 = 1-a1;
f1 = a1-a2;
f2 = a2;

q1 = 0.01;
q2 = 0.01;

%% Processamento 

limpar = input("Limpar mem�ria? ","s");
if limpar == "y"

    % Limpar Serial
    daqduino_read;
    u0 = [num2str(0),',',num2str(0),'\n'];
    daqduino_write(u0,ts);

    daqduino_read;
    daqduino_write(u0,ts);
end

for k = 3:nit
     
    % ----- Sa�da da planta
    angulo_sensor(k) = daqduino_read;

    % Sinal de controle GMV
    delta_pot_motor_1(k) =  (-b11*delta_pot_motor_2(k-1) + angulo_ref(k) - [f0 f1 f2]*angulo_sensor(k:-1:k-2)')/(b10 + q1);
    delta_pot_motor_2(k) =  (-b21*delta_pot_motor_2(k-1) + angulo_ref(k) - [f0 f1 f2]*angulo_sensor(k:-1:k-2)')/(b20 + q2);

    pot_motor_1(k) = pot_motor_1(k-1) + delta_pot_motor_1(k);
    pot_motor_2(k) = pot_motor_2(k-1) - delta_pot_motor_2(k);

    % -------- Satura��es de pot�ncia
    max_pot = 15;
    min_pot = 7;

    pot_motor_1(k) = max(min_pot, min(pot_motor_1(k),max_pot));
    pot_motor_2(k) = max(min_pot, min(pot_motor_2(k),max_pot));
    
    

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
    ylabel("�ngulo (�)")
    xlabel("Tempo (s)")

    legend('Real','Refer�ncia')

end

% Pot�ncia dos Motores
figure(2)
set(gcf,'position',[900 100 800 800])
subplot(211)
plot(t,pot_motor_1)
yticks(6:16)
ylabel("PWM Bandwidth (%)")
xlabel("Tempo (s)")
ylim([6,16])
grid
title(['Pot�ncia do Motor 1 - ' metodo ajuste])

subplot(212)
plot(t,pot_motor_2)
ylim([6,16])
yticks(6:16)
ylabel("PWM Bandwidth (%)")
xlabel("Tempo (s)")
grid
title(['Pot�ncia do Motor 2 - ' metodo ajuste])

