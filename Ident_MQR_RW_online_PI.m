%% Engenharia de Controle de Processos
%% Identificação MQR Clássico
clear all;close all; clc

%% iniciar as variáveis
nit = 200; ts = 0.01;

% Sinal de controle (cuidado!!)
% ----- Referência
angulo_ref(1:nit)  = 50;
angulo_sensor(1:nit)  = 0;
u1(1:nit)  = 0;
u2(1:nit) = 0;
a1 = 0;
a2 = 0;
b10 = 0;
b11 = 0;
erro(1:nit) = 0;
yest(1:nit) = 0;

P = 10^2*eye(3);

teta = 0.01*ones(3,1);

pot_motor_1(1:nit)  = 0;
pot_motor_2(1:nit) = 0;
u = strings(1,nit); u(1:nit) = "0,0";
Kp1 = 0.3;
Ki1 = 0.1;
fi = zeros(1,4);
erro_soma = 0;
%%%%% Processamento do teta - MQR

start = input("Start Daqduino? ","s");
if start == "y"
    daqduino_start('COM6'); % Starts DaqDuino board connected to COM7
end

%% Algoritmo MQR

for k=3:nit
    angulo_sensor(k) = daqduino_read;
    if(angulo_sensor(k)<=0 || angulo_sensor(k)>90)
        angulo_sensor(k) = angulo_sensor(k-1);  % Tratar os dados errados
    end

    % Processo do PI
    erro(k) = angulo_ref(k) - angulo_sensor(k);
    erro_soma = erro_soma + erro(k)*ts;
    pot_motor_1(k) = Kp1*erro(k) + Ki1*erro_soma;

    % Saturação do sinal de controle
    if pot_motor_1(k)> 15
        pot_motor_1(k) = 15;
    elseif pot_motor_1(k)< 7
        pot_motor_1(k) = 7;

    end

    fi = [-angulo_sensor(k-1); pot_motor_1(k-1); pot_motor_1(k-2)];
    erro = angulo_sensor(k)-teta'*fi;
    K= P*fi/(1 + fi'*P*fi);
    teta = teta + K*erro;
    P = P - K*fi'*P;
    % Atualização da matriz P - Método Random Walk
    q = trace(P)/3;
    Q = q*eye(3);
    P = P + Q;

    a1  = teta(1); 
    b10 = teta(2); b11 = teta(3);

    
    yest(k) = -a1*yest(k-1)  + b10*pot_motor_1(k-1) + b11*pot_motor_1(k-2);

    % Mandar o sinal de controle para o arduino
    u(k) = [num2str(pot_motor_1(k)),',',num2str(pot_motor_2(k)),'\n'];
    daqduino_write(u(k),ts);



end

% Desligar os motores
u2 = [num2str(0),',',num2str(0),'\n'];
daqduino_write(u2,ts);


%%G = tf([b1(end) b2(end)],[1 a1(end) a2(end)],1)


% figure(3)
% subplot(211),plot(t,a1,'r',t,a2,'b'), grid on , legend('a1','a2')
% subplot(212),plot(t,b1,'r'),grid on , legend('b1')

% Limpar Memória
daqduino_read
u2 = [num2str(0),',',num2str(0),'\n'];
daqduino_write(u2,ts);



%% Resultados Obtidos

yest_last(1:nit)=0;

for k =3:nit

    yest_last(k) = -a1*yest_last(k-1) + b10*pot_motor_1(k-1) + b11*pot_motor_1(k-2);

end

%%% somatorio do erro quadratico
e= sum((angulo_sensor-yest).^2);

%   figure(2)
t = 0:ts:(nit-1)*ts;
figure(1)
plot(t,angulo_sensor,'r',t,yest_last,'b'),grid on;
title("Identificação MQR do motor 2 - Última amostra")

% 
% figure(2)
% plot(t,angulo_sensor,'r',t,yest,'b'),grid on;
% legend('Real','Estimado Online')
% title("Identificação MQR do motor Separado")
% Resultados
%
% u1(1:nit)  = 7;
% u2(1:nit) = 10;


%
% u1(1:nit)  = 10;
% u2(1:nit) = 7;

