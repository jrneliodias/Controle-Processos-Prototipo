%% Controle Proporcional Integral Sintonizado por Cohen-Coon
clear all; close all; clc

%% ----- Condi��es iniciais
nit = 1500; ts = 0.01;
angulo_sensor = zeros(1,nit); %u(1:nit) = '0,0\n';
erro = zeros(1,nit);

% %% ----- Refer�ncia
angulo_ref = 1*ones(1,nit);
angulo_ref(1:nit/3)  = 20;
angulo_ref(nit/3 +1 : 2*nit/3)  = 60;
angulo_ref(2*nit/3 +1 : 3*nit/3)  = 20;

% %% ----- Vari�vel Controlada
pot_motor_1 = zeros(1,nit);
pot_motor_2 = zeros(1,nit);
u = strings(1,nit); u(1:nit) = "0,0";

% Iniciar o Prot�tipo
start = input("Start Daqduino? ","s");
if start == "y"
    daqduino_start('COM6'); % Starts DaqDuino board connected to COM6
end

%% Sintonia Cohen Coon

% Coefientes do Modelo Smith motor 1
Kpsmith1    = 7.737;
thetasmith1 = 0.65;
tausmith1   = 0.6;


% Coefiientes do Modelo Smith motor 2
Kpsmith2    = 12.86;
thetasmith2 = 0.5;
tausmith2   = 0.66;


% Sintonia do PID Tabela Astrom para o motor 1
Kp1 = 0.94/(Kpsmith1*tausmith1);
Ti1 = 2*tausmith1;
Td1 = 0.5*tausmith1;

% C�lculo Ki1 e Kd1
ajuste = 0.01;
ajuste_ki = 3;

Ki1 = ajuste_ki*ts*Kp1/Ti1;
Kd1 = ajuste*(Kp1*Td1)/ts;

% Sintonia do PID da Tabela Astrom para o motor 2
Kp2 = 0.94/(Kpsmith2*tausmith2);
Ti2 = 2*tausmith2;
Td2 = 0.5*tausmith2;

% C�lculo Ki2 e Kd2
Ki2 = ajuste_ki*ts*Kp2/Ti2;
Kd2 = ajuste*Kp2*Td2/ts;


%% Coeficientes estrutura PID Paralelo

% Param�tro do PID motor 1
q10 = Kp1 + Kd1;
q11 = -Kp1 - 2*Kd1 +Ki1;
q12 = Kd1;

% Param�tro do PID motor 2
q20 = Kp2 + Kd2;
q21 = -Kp2 - 2*Kd2 + Ki2;
q22 = Kd2;



%% ----- Processamento - PID
for k = 3:nit
    % ----- Sa�da da planta
    angulo_sensor(k) = daqduino_read;
    erro(k) = angulo_ref(k) - angulo_sensor(k);
    pot_motor_1(k) = pot_motor_1(k-1) + q10*erro(k) + q11*erro(k-1) + q12*erro(k-2);
    pot_motor_2(k) = pot_motor_2(k-1) -(q20*erro(k) + q21*erro(k-1) + q22*erro(k-2));

    % Satura��o motor 1
    if pot_motor_1(k)> 15
        pot_motor_1(k) = 15;
    elseif pot_motor_1(k)< 7
        pot_motor_1(k) = 7;
    end

    % Satura��o motor 2

    if pot_motor_2(k)> 15
        pot_motor_2(k) = 15;
    elseif pot_motor_2(k)< 7
        pot_motor_2(k) = 7;
    end

    % Desligar motores
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


%% ----- Plotar sinais
metodo = 'Astrom e Hagglund';
ajuste = ' com ajuste';

t = 0:ts:(nit-1)*ts;
figure(1)
p = plot(t,angulo_sensor,'r',t,angulo_ref,'--k');grid
title(["Controle PID sintonizado por " metodo ajuste])

ylim([0,90])
% 
% % Settling criteria
% desired_range = [0.95*angulo_sensor(end), 1.05*angulo_sensor(end)];  % Desired steady-state range
% tolerance = 0.01;             % Tolerance for settling range
% 
% % Find the indices where the response is within the desired range
% settling_indices = find(angulo_sensor > desired_range(1) & angulo_sensor < desired_range(2));
% 
% % Calculate the settling time
% settling_time = t(settling_indices(1)) - t(1);
% 
% % Rise time calculation
% 
% rise_9_index = findidx(angulo_sensor,0.9*angulo_sensor(end));
% rise_1_index = findidx(angulo_sensor,0.1*angulo_sensor(end));
% rise_time = t(rise_9_index(1)) - t(rise_1_index(1));
% 
% % Plot the settling time point
% hold on;
% 
% 
% plot(t(settling_indices(1)), angulo_sensor(settling_indices(1)), 'b.','MarkerSize',20);
% text(t(settling_indices(1)), angulo_sensor(settling_indices(1))+5,rise_time, ['Settling Time = ', num2str(settling_time)],...
%     'VerticalAlignment', 'bottom','HorizontalAlignment','center','Color','b');
% % Plot the rise time point
% 
% plot(t(rise_9_index), angulo_sensor(rise_9_index), 'k.','MarkerSize',20);
% text(t(rise_9_index)+0.2, angulo_sensor(rise_9_index)+0.5, ['Rise Time = ', num2str(rise_time)], 'VerticalAlignment', 'top','HorizontalAlignment','left');
% 
% legend('Real','Refer�ncia')


% Pot�ncia dos Motores
figure(2)
subplot(211)
plot(t,pot_motor_1)
ylim([6,16])
grid
title(['Pot�ncia do Motor 1 - ' metodo ajuste])

subplot(212)
plot(t,pot_motor_2)
ylim([6,16])
grid
title(['Pot�ncia do Motor 2 - ' metodo ajuste])
