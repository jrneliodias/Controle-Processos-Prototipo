%% Controle IMC Incremental
%clear all; close all; clc

%% ----- Condições iniciais
nit = 500; ts = 0.01;
angulo_sensor = zeros(1,nit); 
angulo_model1 = zeros(1,nit); 
angulo_model2 = zeros(1,nit); 
erro1 = zeros(1,nit);
erro2 = zeros(1,nit);
d0barra1 = zeros(1,nit);
d0barra2 = zeros(1,nit);

d=1;

% ----- Ajuste do taumf1 e taumf2
ajuste1 = 0.6;
ajuste2 = 0.6;

% %% ----- Referência
angulo_ref = 50*ones(1,nit);
% angulo_ref(1:nit/2)  = 80;
% angulo_ref(nit/2 +1 : nit)  = 20;
% angulo_ref(nit/3 +1 : 2*nit/3)  = 50;
% angulo_ref(2*nit/3 +1 : 3*nit/3)  = 50;

% %% ----- Variável Controlada
pot_motor_1 = zeros(1,nit);
pot_motor_2 = zeros(1,nit);
delta_pot_motor_1 = zeros(1,nit);
delta_pot_motor_2 = zeros(1,nit);
u = strings(1,nit); u(1:nit) = "0,0";


% Iniciar o Protótipo
start = input("Start Daqduino? ","s");
if start == "y"
    daqduino_start('COM9'); 
end

%% Condições da Identificação MQR
% Coeficientes do motor 1
a1m1 = 0;
b0m1 = 0;
b1m1 = 0;

% Coeficientes do motor 2
b0m2 = 0;
b1m2 = 0;
a1m2 = 0;

% Inicializar os erros de estimação
erro_mqr1 = 0;
erro2_mqr2 = 0;

% Matriz P
P1= 10^3*eye(3);
P2= 10^3*eye(3);

% Vetores Theta
teta1 = 0.05*ones(3,1);
teta2 = 0.05*ones(3,1);




%% ----- Processamento
limpar = input("Limpar memória? ","s");
    if limpar == "y"

        % Limpar Serial
        daqduino_read
        u0 = [num2str(0),',',num2str(0),'\n'];
        daqduino_write(u0,ts);

        daqduino_read
        daqduino_write(u0,ts);
    end

for k = 3+d:nit
    
    %% ----- Saída da planta
    angulo_sensor(k) = daqduino_read;


    %% ----------- Identificação MQR

    % Motor 1
    fi1 = [-angulo_sensor(k-1); pot_motor_1(k-1); pot_motor_1(k-2)];
    erro_mqr1 = angulo_sensor(k)-teta1'*fi1;
    K1= P1*fi1/(1 + fi1'*P1*fi1);
    teta1 = teta1 + K1*erro_mqr1;
    P1 = P1 - K1*fi1'*P1;

    % ---- Coeficientes do modelo do motor 1
    a1m1 = teta1(1);
    b0m1 = teta1(2); b1m1 = teta1(3);

    % ---- Saida do modelo do motor 1
    angulo_model1(k) = -a1m1*angulo_model1(k-1) + b0m1*pot_motor_1(k-d-1) + b1m1*pot_motor_1(k-d-2);

    % Motor 2
    fi2 = [-angulo_sensor(k-1); pot_motor_2(k-1); pot_motor_2(k-2)];
    erro_mqr2 = angulo_sensor(k)-teta2'*fi2;
    K2= P2*fi2/(1 + fi2'*P2*fi2);
    teta2 = teta2 + K2*erro_mqr2;
    P2 = P2 - K2*fi2'*P2;

    % ---- Coeficientes do modelo do motor 2
    a1m2  = teta2(1);
    b0m2 = teta2(2); b1m2 = teta2(3);

    % ---- Saida do modelo do motor 2
    angulo_model2(k) = -a1m2*angulo_model2(k-1) + b0m1*pot_motor_2(k-d-1) + b1m2*pot_motor_2(k-d-2);


    %% ---------- Projeto do controlador RST
    % Calculo do tau de malha fechada
    alpha1 = 0.98%-a1m1;

    % Calculo do tau de malha fechada
    alpha2 = 0.97%-a1m2;

    
    % Determinar a incerteza
    d0barra1(k) = angulo_sensor(k) - angulo_model1(k);
    d0barra2(k) = -(angulo_sensor(k) - angulo_model2(k));
    
    % Determinar o Erro
    erro1(k) = angulo_ref(k) - d0barra1(k);
    erro2(k) = -(angulo_ref(k) + d0barra2(k));
    
    % Sinal de controle
   % pot_motor_1(k) = alpha1*pot_motor_1(k-1) + ((1-alpha1)/b0m1)*erro1(k-1)+ a1m1*((1-alpha1)/b0m1)*erro1(k-2);
   % pot_motor_2(k) = alpha2*pot_motor_2(k-1) + ((1-alpha2)/b0m2)*erro2(k-1)+ a1m2*((1-alpha2)/b0m2)*erro2(k-2);

     pot_motor_1(k) = ((alpha1*b0m1-b1m1)/b0m1)*pot_motor_1(k-1) + ((alpha1*b1m1)/b0m1)*pot_motor_1(k-2)...
                    + ((1-alpha1)/b0m1)*erro1(k-1) + a1m1*((1-alpha1)/b0m1)*erro1(k-2);

     pot_motor_2(k) = ((alpha2*b0m2-b1m2)/b0m2)*pot_motor_2(k-1) + ((alpha2*b1m2)/b0m2)*pot_motor_2(k-2)...
                    + ((1-alpha2)/b0m2)*erro2(k-1) + a1m2*((1-alpha2)/b0m2)*erro2(k-2);



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




%% ----- Plotar sinais
metodo = 'Alocações de Polos por IMC Adaptativo';
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