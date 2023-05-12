%% Controle Proporcional Integral
clear all; close all; clc

%% ----- Condições iniciais
nit = 900; ts = 0.01;
angulo_sensor(1:nit) = 0; %u(1:nit) = '0,0\n';
erro(1:nit) = 0;

% %% ----- Referência
angulo_ref(1:nit/2)  = 20;
angulo_ref(nit/2 +1 : nit)  = 20;

% %% ----- Variável Controlada
pot_motor_1(1:nit)  = 0;
pot_motor_2(1:nit) = 0;
u = strings(1,nit); u(1:nit) = "0,0";
Kp1 = 0.6;
Kp2 = 0.7;
Ki1 = 0.1;
Ki2 = 0.1;

erro_soma = 0;


start = input("Start Daqduino? ","s");
if start == "y"
    daqduino_start('COM9'); % Starts DaqDuino board connected to COM7
end

%% ----- Processamento - Estimação
for k = 2:nit
    % ----- Saída da planta
        angulo_sensor(k) = daqduino_read;
        erro(k) = angulo_ref(k) - angulo_sensor(k);
        erro_soma = erro_soma + erro(k)*ts;
        pot_motor_1(k) = Kp1*erro(k) + Ki1*erro_soma;
        pot_motor_2(k) = -(Kp2*erro(k) + Ki2*erro_soma);

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



u0 = [num2str(0),',',num2str(0),'\n'];
daqduino_write(u0,ts);




%% ----- Plotar sinais
t = 0:ts:(nit-1)*ts;
figure(1)
plot(t(2:nit),angulo_sensor(2:nit),'r',t(2:nit),angulo_ref(2:nit),'--k'),grid
title("Controle Proporcional-Integral")
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
