%% Controle Proporcional na Planta - Tentativa e Erro

%clear all; close all; clc

%% ----- Condições iniciais

nit = 1200; ts = 0.01;
angulo_sensor = zeros(1,nit); %u(1:nit) = '0,0\n';
erro = zeros(1,nit);

% %% ----- Referência
angulo_ref = 1*ones(1,nit);
angulo_ref(1:nit/2)  = 80;
angulo_ref(nit/2 +1 : nit)  = 20;
% angulo_ref(2*nit/3 +1 : 3*nit/3)  = 20;

% %% ----- Variável Controlada
pot_motor_1 = zeros(1,nit);
pot_motor_2 = zeros(1,nit);
u = strings(1,nit); u(1:nit) = "0,0";

% Iniciar o Protótipo
start = input("Start Daqduino? ","s");
if start == "y"
    daqduino_start('COM6'); % Starts DaqDuino board connected to COM6
end

%% Ganhos do Controlador

Kp1 = 1.1;
Kp2 = 1.8;

%% ----- Processamento - Estimação
limpar = input("Limpar memória? ","s");
if limpar == "y"

    % Limpar Serial
    daqduino_read
    u0 = [num2str(0),',',num2str(0),'\n'];
    daqduino_write(u0,ts);

    daqduino_read
    daqduino_write(u0,ts);
end


for k = 2:nit
    % ----- Saída da planta
  
        angulo_sensor(k) = daqduino_read;
        erro(k) = angulo_ref(k) - angulo_sensor(k);
        pot_motor_1(k) = Kp1*erro(k);
        pot_motor_2(k) = -Kp2*erro(k);

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

% Desligar motor
u0 = [num2str(0),',',num2str(0),'\n'];
    daqduino_write(u0,ts);


%% ----- Plotar sinais
metodo = 'Proporcional';
ajuste = ' com mudança de referência';

t = 0:ts:(nit-1)*ts;
figure(1)
p = plot(t,angulo_sensor,'r',t,angulo_ref,'--k');grid
title(["Controle " metodo ajuste])

ylim([0,90])

indices = input("Calcular indices? ","s");
if indices == "y"
    daqduino_start('COM6'); % Starts DaqDuino board connected to COM6
    % Settling criteria
    desired_range = [0.95*angulo_sensor(end), 1.05*angulo_sensor(end)];  % Desired steady-state range
    tolerance = 0.01;             % Tolerance for settling range

    % Find the indices where the response is within the desired range
    settling_indices = find(angulo_sensor > desired_range(1) & angulo_sensor < desired_range(2));

    % Calculate the settling time
    settling_time = t(settling_indices(1)) - t(1);

    % Rise time calculation

    rise_9_index = findidx(angulo_sensor,0.9*angulo_sensor(end));
    rise_1_index = findidx(angulo_sensor,0.1*angulo_sensor(end));
    rise_time = t(rise_9_index(1)) - t(rise_1_index(1));

    % Plot the settling time point
    hold on;


    plot(t(settling_indices(1)), angulo_sensor(settling_indices(1)), 'b.','MarkerSize',20);
    text(t(settling_indices(1)), angulo_sensor(settling_indices(1))+5,rise_time, ['Settling Time = ', num2str(settling_time)],...
        'VerticalAlignment', 'bottom','HorizontalAlignment','center','Color','b');
    % Plot the rise time point

    plot(t(rise_9_index), angulo_sensor(rise_9_index), 'k.','MarkerSize',20);
    text(t(rise_9_index)+0.2, angulo_sensor(rise_9_index)+0.5, ['Rise Time = ', num2str(rise_time)], 'VerticalAlignment', 'top','HorizontalAlignment','left');

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
