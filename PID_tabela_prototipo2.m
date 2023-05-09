%% Controle Proporcional Integral
clear all; close all; clc

%% ----- Condições iniciais
nit = 1200; ts = 0.01;
angulo_sensor(1:nit) = 0; %u(1:nit) = '0,0\n';
erro(1:nit) = 0;
erro_soma=0;
% %% ----- Referência
angulo_ref(1:nit)  = 70;
% angulo_ref(1:nit/2)  = 80;
% angulo_ref(nit/2 +1 : nit)  = 20;

% %% ----- Variável Controlada
pot_motor_1(1:nit)  = 0;
pot_motor_2(1:nit) = 0;
u = strings(1,nit); u(1:nit) = "0,0";
% 
% tau_zn1   = 0.71;
% theta_zn1 = 0.82;
% Kp_zn1 = 6.7;
% 
% tau_zn2   = 0.54;
% theta_zn2 = 0.66;
% Kp_zn2 = 8.3375;
% 
% erro_soma =0;
% 
% Kp1 = 1.2*tau_zn1/(Kp_zn1*theta_zn1);
% Ti1 = 2*theta_zn1;
% Td1 = theta_zn1/2;
% 
% Ki1 = ts*Kp1/Ti1;
% Kd1 = Kp_zn1*Td1/ts;
% 
% Kp2 = 1.2*tau_zn2/(Kp_zn2*theta_zn2);
% Ti2 = 2*theta_zn2;
% Td2 = theta_zn2/2;
% 
% Ki2 = ts*Kp2/Ti2;
% Kd2 = Kp_zn2*Td2/ts;


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

%% PID ZN
% 
% % Paramêtro do PID motor 1
% q10 = Kp1 + Kd1;
% q11 = -Kp1 - 2*Kd1 +Ki1;
% q12 = Kd1;
% 
% % Paramêtro do PID motor 2
% q20 = Kp2 + Kd2;
% q21 = -Kp2 - 2*Kd2 + Ki2;
% q22 = Kd2;

%% PI ZN
% Paramêtro do PID motor 1
% 
Kp1 = 0.6;
Kp2 = 0.7;
Ki1 = 0.5;
Ki2 = 0.3;
Kd1 = 0.7;
Kd2 = 0.9;
% 
% q10 = Kp1;
% q11 = -Kp1 +Ki1;
% 
% 
% % Paramêtro do PID motor 2
% q20 = Kp2 ;
% q21 = -Kp2 + Ki2;
% 
% 
start = input("Start Daqduino? ","s");
if start == "y"
     daqduino_start('COM6'); % Starts DaqDuino board connected to COM7
end

%% ----- Processamento - Estimação

angulo_sensor(1) = daqduino_read;
u0 = [num2str(0),',',num2str(0),'\n'];
daqduino_write(u0,ts);

for k = 3:nit
    % ----- Saída da planta
    angulo_sensor(k) = daqduino_read;
    erro(k) = angulo_ref(k) - angulo_sensor(k);
        erro_soma = erro_soma + erro(k)*ts;
        pot_motor_1(k) = Kp1*erro(k) + Ki1*erro_soma+ Kd1*(erro(k)-erro(k-1));
        pot_motor_2(k) = -(Kp2*erro(k) + Ki2*erro_soma + Kd2*(erro(k)-erro(k-1)));

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

% Limpar Serial
daqduino_read
u0 = [num2str(0),',',num2str(0),'\n'];
daqduino_write(u0,ts);




%% ----- Plotar sinais
t = 0:ts:(nit-1)*ts;
figure(1)
plot(t(2:nit),angulo_sensor(2:nit),'r',t(2:nit),angulo_ref(2:nit),'--k'),grid
title("Controle PID Paralelo")
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
