%% Controle Proporcional Integral Sintonizado por Ziegler Nichols
%clear all; close all; clc

%% ----- Condições iniciais
nit = 1200; ts = 0.01;
angulo_sensor = zeros(1,nit); 
erro = zeros(1,nit);

% %% ----- Referência
angulo_ref = 50*ones(1,nit);
angulo_ref(1:nit/2)  = 80;
angulo_ref(nit/2 +1 : nit)  = 20;
% angulo_ref(nit/3 +1 : 2*nit/3)  = 50;
% angulo_ref(2*nit/3 +1 : 3*nit/3)  = 50;

% %% ----- Variável Controlada
pot_motor_1 = zeros(1,nit);
pot_motor_2 = zeros(1,nit);
u = strings(1,nit); u(1:nit) = "0,0";


% Iniciar o Protótipo
start = input("Start Daqduino? ","s");
if start == "y"
    daqduino_start('COM6'); % Starts DaqDuino board connected to COM7
end

%% Sintonia ZN

% Coefiientes do Modelo Smith motor 1
Kpsmith1 = 7.737;
thetasmith1 =0.65;
tausmith1 =0.6;

tau_zn1   = tausmith1;
theta_zn1 = thetasmith1;
Kp_zn1 = Kpsmith1;

% Coefiientes do Modelo Smith motor 2
Kpsmith2 =  12.86;
thetasmith2 = 0.5;
tausmith2 =   0.66;

tau_zn2   = tausmith2;
theta_zn2 = thetasmith2;
Kp_zn2 = Kpsmith2;


% Sintonia do PID Tabela ZN para o motor 1
Kp1 = 1.2*tau_zn1/(Kp_zn1*theta_zn1);
Ti1 = 2*theta_zn1;
Td1 = theta_zn1/2;

ajuste_kd = 0.01;

% Cálculo Ki1 e Kd1
Ki1 = ts*Kp1/Ti1;
Kd1 = ajuste_kd*(Kp1*Td1)/ts;

% Sintonia do PID da Tabela ZN para o motor 2
Kp2 = 1.2*tau_zn2/(Kp_zn2*theta_zn2);
Ti2 = 2*theta_zn2;
Td2 = theta_zn2/2;


% Cálculo Ki2 e Kd2
Ki2 = ts*Kp2/Ti2;
Kd2 = ajuste_kd*Kp2*Td2/ts;

% Sintonia PI


% Kp1 = 0.9*tau_zn1/(Kp_zn1*theta_zn1);
% Ti1 = 3*theta_zn1;
% Td1 = theta_zn1/2;
% 
% Ki1 = ts*Kp1/Ti1;
% Kd1 = Kp_zn1*Td1/ts;
% 
% Kp2 = 0.9*tau_zn2/(Kp_zn2*theta_zn2);
% Ti2 = 3*theta_zn2;
% Td2 = theta_zn2/2;
% 
% Ki2 = ts*Kp2/Ti2;
% Kd2 = Kp_zn2*Td2/ts;

%% Coeficientes estrutura PID Paralelo

% Paramêtro do PID motor 1
q10 = Kp1 + Kd1;
q11 = -Kp1 - 2*Kd1 +Ki1;
q12 = Kd1;

% Paramêtro do PID motor 2
q20 = Kp2 + Kd2;
q21 = -Kp2 - 2*Kd2 + Ki2;
q22 = Kd2;



%% ----- Processamento - PID
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
    erro(k) = angulo_ref(k) - angulo_sensor(k);
    pot_motor_1(k) = pot_motor_1(k-1) + q10*erro(k) + q11*erro(k-1) + q12*erro(k-2);
    pot_motor_2(k) = pot_motor_2(k-1) -(q20*erro(k) + q21*erro(k-1) + q22*erro(k-2));


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
metodo = 'PID Ziegler Nichols';
ajuste = ' Ajuste no Kd';

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