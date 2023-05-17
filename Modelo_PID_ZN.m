%% Controle Proporcional Integral Sintonizado por Cohen-Coon
clear all;  clc

%% ----- Condições iniciais
nit = 800; ts = 0.01;
[angulo_m1,angulo_m2,erro,angulo_sensor] = deal(zeros(1,nit));

% %% ----- Referência
angulo_ref = 50*ones(1,nit);
% angulo_ref(1:nit/2)  = 80;
% angulo_ref(nit/2 +1 : nit)  = 20;

% %% ----- Variável Controlada
pot_motor_1 = zeros(1,nit);
pot_motor_2 = zeros(1,nit);
u = strings(1,nit); u(1:nit) = "0,0";

% Iniciar o Protótipo
% start = input("Start Daqduino? ","s");
% if start == "y"
%     daqduino_start('COM6'); % Starts DaqDuino board connected to COM6
% end

%% Sintonia ZN

% Coefientes do Modelo Smith motor 1
Kpsmith1    = 7.737;
thetasmith1 = 0.65;
tausmith1   = 0.6;

% Função de Transferência do motor 1
s= tf('s');
Gsmith1 = Kpsmith1*exp(-thetasmith1*s)/(tausmith1*s+1);

% Discretizacao do modelo
Gmz1 = c2d(Gsmith1,ts);
Bm1  = num2cell(Gmz1.num{1});  Am1  = num2cell(Gmz1.den{1});

% Atraso de Transporte
d1 = ceil(thetasmith1/ts);
%d1 = 30;
% Planta Real
b0m1 = Bm1{2:end};
a1m1 = Am1{2:end};

% Coefiientes do Modelo Smith motor 2
Kpsmith2    = 12.86;
thetasmith2 = 0.5;
tausmith2   = 0.66;

% Função de Transferência do motor 2
s= tf('s');
Gsmith2 = Kpsmith2*exp(-thetasmith2*s)/(tausmith2*s+1);

% Discretizacao do modelo
Gmz2 = c2d(Gsmith2,ts);
Bm2  = num2cell(Gmz2.num{1});  Am2  = num2cell(Gmz2.den{1});

% Atraso de Transporte
d2 = ceil(thetasmith2/ts);

%d2 =10;
% Planta Real
b0m2 = Bm2{2:end};
a1m2 = Am2{2:end};


%% Sintonia do PID Tabela Ziegler Nichols para o motor 1

% Sintonia do PID Tabela ZN para o motor 1
Kp1 = 1.2*tausmith1/(Kpsmith1*thetasmith1);
Ti1 = 2*thetasmith1;
Td1 = thetasmith1/2;

% Cálculo Ki1 e Kd1
Ki1 = ts*Kp1/Ti1;
Kd1 = (Kp1*Td1)/ts;

% Sintonia do PID da Tabela ZN para o motor 2
Kp2 = 1.2*tausmith2/(Kpsmith2*thetasmith2);
Ti2 = 2*thetasmith2;
Td2 = thetasmith2/2;


% Cálculo Ki2 e Kd2
Ki2 = ts*Kp2/Ti2;
Kd2 = Kp2*Td2/ts;



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
for k = 3+max(d1,d2):nit

    % ----- Saída da planta
    angulo_m1(k) = -a1m1*angulo_m1(k-1) + b0m1*pot_motor_1(k-d1);
    angulo_m2(k) = -a1m2*angulo_m2(k-1) + b0m2*pot_motor_2(k-d2);

    angulo_sensor(k) = angulo_m1(k)-angulo_m2(k);
    erro(k) = angulo_ref(k) - angulo_sensor(k);
    pot_motor_1(k) = pot_motor_1(k-1) + q10*erro(k) + q11*erro(k-1) + q12*erro(k-2);
    pot_motor_2(k) = pot_motor_2(k-1) -(q20*erro(k) + q21*erro(k-1) + q22*erro(k-2));

    % Saturação motor 1
    if pot_motor_1(k)> 15
        pot_motor_1(k) = 15;
    elseif pot_motor_1(k)< 7
        pot_motor_1(k) = 7;
    end

    % Saturação motor 2

    if pot_motor_2(k)> 15
        pot_motor_2(k) = 15;
    elseif pot_motor_2(k)< 7
        pot_motor_2(k) = 7;
    end

%     % Desligar motores
%     u(k) = [num2str(pot_motor_1(k)),',',num2str(pot_motor_2(k)),'\n'];
%     daqduino_write(u(k),ts);

    if(angulo_sensor(k)<=0 || angulo_sensor(k)>90)
        angulo_sensor(k) = angulo_sensor(k-1);  % Tratar os dados errados
    end



end



% 
% % Limpar a Serial
% daqduino_read
% u0 = [num2str(0),',',num2str(0),'\n'];
% daqduino_write(u0,ts);




%% ----- Plotar sinais
t = 0:ts:(nit-1)*ts;
figure(1)
plot(t(2:nit),angulo_sensor(2:nit),'r',t(2:nit),angulo_ref(2:nit),'--k'),grid
title("Controle PID sintonizado por Ziegler Nichols")
legend('Real')
ylim([0,90])

figure(2)
subplot(211)
plot(t,pot_motor_1)
ylim([6,16])
grid
title("Potência do Motor 1")

subplot(212)
plot(t,pot_motor_2)
ylim([6,16])
grid
title("Potência do Motor 2")


%
%
%
% finish = input("Close Daqduino? ","s");
% if finish =="y"
%     daqduino_end; % End the connection to the DaqDuino device.
% end













%         angulo_sensor(k) = daqduino_read;
%         erro(k) = angulo_ref(k) - angulo_sensor(k);
%         if(erro(k)>0)
%             pot_motor_1(k) = 10;
%             pot_motor_2(k) = 7;
%         else
%             pot_motor_1(k) = 7;
%             pot_motor_2(k) = 10;
%         end
%
%         if (pot_motor_1(k)> 12) || (pot_motor_2(k) >12)
%             u = [num2str(7),',',num2str(7),'\n'];
%             daqduino_write(u,ts);
%         else
%             u = [num2str(pot_motor_1(k)),',',num2str(pot_motor_2(k)),'\n'];
%             daqduino_write(u,ts); % D/A to analog output, and delaying Ts seconds.
%
%
%         end
