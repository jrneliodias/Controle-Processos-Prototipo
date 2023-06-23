%% Controle IMC Incremental
%clear all; close all; clc

%% ----- Condições iniciais
nit = 1200; ts = 0.01;
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

% -------- Saturações de potência
max_pot = 15;
min_pot = 7;


%% ----- Referência
angulo_ref = 50*ones(1,nit);
angulo_ref(1:nit/2)  = 30;
angulo_ref(nit/2 +1 : nit)  = 70;
%angulo_ref(nit/3 +1 : 2*nit/3)  = 50;
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
    daqduino_start('COM6'); % Starts DaqDuino board connected to COM7
end


%% Projeto do controlador RST

% Coefiientes do Modelo Smith motor 1
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
Bm1 = Gmz1.num{1}, Am1= Gmz1.den{1};

% Coeficientes do Modelo
b0m1 = Bm1(2); a1m1 = Am1(2);


% Calculo do tau de malha fechada
tau_mf1 = ajuste1*tausmith1;

alpha1 = exp(-ts/tau_mf1);


% Função de transferência motor 2
Gm2 = tf(Kpsmith2,[tausmith2 1],'InputDelay',thetasmith2)

% Discretização do modelo
Gmz2 = c2d(Gm2,ts);
Bm2 = Gmz2.num{1}; Am2= Gmz2.den{1};

% Coeficientes do Modelo
b0m2 = Bm2(2); a1m2 = Am2(2);

% Calculo do tau de malha fechada
tau_mf2 = ajuste2*tausmith2;
alpha2 = exp(-ts/tau_mf2);


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

for k = 2+d:nit
    
    % ----- Saída da planta
    angulo_sensor(k) = daqduino_read;

    % ---- Saida do modelo
    angulo_model1(k) = -a1m1*angulo_model1(k-1) + b0m1*pot_motor_1(k-d-1);
    angulo_model2(k) = -a1m2*angulo_model2(k-1) - b0m2*pot_motor_2(k-d-1);
    
    % Determinar a incerteza
    d0barra1(k) = angulo_sensor(k) - angulo_model1(k);
    d0barra2(k) = -(angulo_sensor(k) - angulo_model2(k));
    
    % Determinar o Erro
    erro1(k) = angulo_ref(k) - d0barra1(k);
    erro2(k) = -(angulo_ref(k) + d0barra2(k));
    
    % Sinal de controle
    pot_motor_1(k) = alpha1*pot_motor_1(k-1) + ((1-alpha1)/b0m1)*erro1(k-1)+ a1m1*((1-alpha1)/b0m1)*erro1(k-2);
    pot_motor_2(k) = alpha2*pot_motor_2(k-1) + ((1-alpha2)/b0m2)*erro2(k-1)+ a1m2*((1-alpha2)/b0m2)*erro2(k-2);

    % Saturações do sinal de Controle
    pot_motor_1(k) = max(min_pot, min(pot_motor_1(k),max_pot));
    pot_motor_2(k) = max(min_pot, min(pot_motor_2(k),max_pot));
    

    % Enviar sinal de controle para os motores
    u(k) = [num2str(pot_motor_1(k)),',',num2str(pot_motor_2(k)),'\n'];
    daqduino_write(u(k),ts);

    % Retirar dados do sensor errado
    if(angulo_sensor(k)<=0 || angulo_sensor(k)>90)
        angulo_sensor(k) = angulo_sensor(k-1);  % Tratar os dados errados
    end



end


% Limpar a Serial
daqduino_read
u0 = [num2str(0),',',num2str(0),'\n'];
daqduino_write(u0,ts);




%% ----- Plotar sinais
metodo = 'Controle IMC';
ajuste = ' com mudança de referência e pertubação';

t = 0:ts:(nit-1)*ts;
figure(1)
p = plot(t,angulo_sensor,'r',t,angulo_ref,'--k');grid
title(["Controle " metodo ajuste])
ylabel("Ângulo (º)")
xlabel("Tempo (s)")

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
ylabel("PWM Bandwidth (%)")
xlabel("Tempo (s)")
grid
title(['Potência do Motor 1 - ' metodo ajuste])

subplot(212)
plot(t,pot_motor_2)
ylim([6,16])
ylabel("PWM Bandwidth (%)")
xlabel("Tempo (s)")
grid
title(['Potência do Motor 2 - ' metodo ajuste])

