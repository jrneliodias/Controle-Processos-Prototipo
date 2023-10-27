%% Projeto GMV Incremental
% Engenharia de Controle de Processos

%clc; clear all; close all


%% Condições Iniciais
nit = 50; ts = 0.01;
angulo_sensor = zeros(1,nit); 

%% Referência e Pertubação
angulo_ref = 70*ones(1,nit);
mudref1 = 500;
mudref2 = 1000;
angulo_ref(1:mudref1) = 20;
angulo_ref(mudref1+1:mudref2) = 80;
angulo_ref(mudref2+1:nit) = 30;


%% ----- Variável Controlada
pot_motor_1 = zeros(1,nit);
pot_motor_2 = zeros(1,nit);
delta_pot_motor_1 = zeros(1,nit);
delta_pot_motor_2 = zeros(1,nit);
u = strings(1,nit); u(1:nit) = "0,0";


% Iniciar o Protótipo
start = input("Start Daqduino? ","s");
if start == "y"
    arduino = serialport("COM7",250000,"Timeout",5);
    pause(2)
    writeline(arduino,'0');
    
end

%% Planta e Modelo

%Coefiientes do Modelo Smith motor 1
Kpsmith1 = 0.999;
thetasmith1 = 0.65;
tausmith1 = 1.827;

% Função de transferência motor 1
Gm1 = tf(Kpsmith1,[tausmith1 1],'InputDelay',thetasmith1)

% Discretização do modelo
Gmz1 = c2d(Gm1,ts);
Bm1 = Gmz1.num{1}; Am1= Gmz1.den{1};

% Coeficientes do Modelo
b0m1 = Bm1(2); a1m1 = Am1(2);


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

% Definir o polinômio P1(z)
P1 = zeros(ne1 + ns1 + 2,1);
P1(1) = 1;
%P1(2) = -0.98;

% Encontrar o polinômio E1(z) e S1(z)
Am1_barra1 = conv(Am1,[1 -1]);

mat_Scoef1 = [zeros(ne1+1,ns1+1);eye(ns1+1)];

mat_EAcoef1 = zeros(ne1 + ns1 + 2, ne1+1);
am1_barra_len = length(Am1_barra1);

for k = 1:ne1+1
    mat_EAcoef1(k:k+am1_barra_len-1,k) = Am1_barra1';
end

mat_SEcoef1 = [mat_EAcoef1 mat_Scoef1];

EScoef1_array = mat_SEcoef1\P1;

epoly1 = EScoef1_array(1:ne1+1)';
spoly1 = EScoef1_array(ne1+2:end)';

% Encontrar o polinômio R por meio do Q1(z) e B1(z)

q01 = 5;
q0_barra1 = q01;%*[1 -1];

% Calculo do polinômio R1(z)
%rpoly1 = [Bm1(2)+epoly1, 0] +q0_barra1;
%rpoly1 = Bm1(2)+ epoly1+q01;

BE_poly = conv(Bm1,epoly1);
BE_poly_len = length(BE_poly);

%rpoly1 = BE_poly + [zeros(1,BE_poly_len-2) q0_barra1];
rpoly1 = BE_poly(2) +q01;
nr1 = length(rpoly1)-1;

% Calculo do polinômio T1(z)
t01 = sum(P1);

% -------- Saturações de potência
max_pot = 100;
min_pot = 0;

%% Processamento 

limpar = input("Limpar memória? ","s");
if limpar == "y"
    % Limpar Serial
    flush(arduino);
end

for k = 3+max(nr1,ns1):nit
     
    % ----- Saída da planta
    angulo_sensor(k) = str2double(readline(arduino));

    % Sinal de controle GMV
    %delta_pot_motor_1(k) =  (-rpoly1(2:end)*delta_pot_motor_1(k-1:-1:k-nr1)'+ t01*angulo_ref(k) - spoly1*angulo_sensor(k:-1:k-ns1)')/rpoly1(1);
    %delta_pot_motor_2(k) =  (-rpoly2(2:end)*delta_pot_motor_2(k-1:-1:k-nr2)'+ t02*angulo_ref(k) - spoly2*angulo_sensor(k:-1:k-ns2)')/rpoly2(1);
    delta_pot_motor_1(k) =  (t01*angulo_ref(k) - spoly1*angulo_sensor(k:-1:k-ns1)')/rpoly1(1);

    pot_motor_1(k) = pot_motor_1(k-1) + delta_pot_motor_1(k);

    % -------- Saturações de potência
    pot_motor_1(k) = max(min_pot, min(pot_motor_1(k),max_pot));
 
    

    % Mandar sinal de controle para os Motores
    %u(k) = [num2str(pot_motor_1(k)),',',num2str(pot_motor_2(k)),'\n'];
    u(k) = [num2str(pot_motor_1(k)) '\r'];
    writeline(arduino,u(k));
    

    if(angulo_sensor(k)<=0 || angulo_sensor(k)>90)
        angulo_sensor(k) = angulo_sensor(k-1);  % Tratar os dados errados
    end

end



% Limpar a Serial
str2double(readline(arduino))
writeline(arduino,'0');

%u0 = [num2str(0),',',num2str(0),'\n'];
%daqduino_write(u0,ts);


%% Resultados
metodo = 'GMV';
ajuste = '';

t = 0:ts:(nit-1)*ts;
figure(1)
set(gcf,'position',[100 100 650 500])
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
set(gcf,'position',[800 100 650 500])
subplot(211)
plot(t,pot_motor_1)
yticks(6:20)
ylabel("PWM Bandwidth (%)")
xlabel("Tempo (s)")
ylim([6,20])
grid
title(['Potência do Motor 1 - ' metodo ajuste])

subplot(212)
plot(t,pot_motor_2)
ylim([6,20])
yticks(6:20)
ylabel("PWM Bandwidth (%)")
xlabel("Tempo (s)")
grid
title(['Potência do Motor 2 - ' metodo ajuste])

