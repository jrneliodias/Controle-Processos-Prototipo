%% Sistema em malha aberta

clear all; close all; clc

%% ----- Condições iniciais
nit = 200; ts = 0.01; 
angulo(1:nit) = 0; %u(1:nit) = '0,0\n'; 
%
% angulo(1)=90;
% %% ----- Variável Controlada
% u1(1:nit)  = 0;
% u2(1:nit) = 0;
% 
u1(1:nit)  = 10;
u2(1:nit) = 8;
% % 
% u1(1:nit/2) = 7;  u1(nit/2+1:nit) = 15;
% u2(1:nit/2) = 15; u2(nit/2+1:nit) = 7;

start = input("Start Daqduino? ","s");
if start == "y"
    daqduino_start('COM6'); % Starts DaqDuino board connected to COM7
end

%% ----- Processamento - Estimação
for k = 2:nit
    % ----- Saída da planta

    angulo(k) = daqduino_read;
    if(angulo(k)<=0 || angulo(k)>90)
        angulo(k) = angulo(k-1);  % Tratar os dados errados
    end
    u = [num2str(u1(k)),',',num2str(u2(k)),'\n'];
    daqduino_write(u,ts); % D/A to analog output, and delaying Ts seconds.



end



u = [num2str(0),',',num2str(0),'\n'];
daqduino_write(u,ts);

finish = input("Close Daqduino? ","s");
if finish =="y"
    daqduino_end; % End the connection to the DaqDuino device.
end



%% ----- Plotar sinais
t = 0:ts:(nit-1)*ts;
figure(1)
plot(t,angulo,'r','linewidth',1),grid
title("Simulação Malha Aberta")
legend('Real')


%% Identificação do motor direito
angulo(1:3) =90
values = -angulo +90
plot(values)