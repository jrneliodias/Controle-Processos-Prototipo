
clear all; close all; clc

%% ----- Condições iniciais
nit = 500; ts = 0.01; 
angulo(1:nit) = 0; %u(1:nit) = '0,0\n'; 

% %% ----- Variável Controlada
% u1(1:nit)  = 0;
% u2(1:nit) = 0;

% u1(1:nit)  = 0;
% u2(1:nit) = 7;
% 
u1(1:nit/2) = 7;  u1(nit/2+1:nit) = 10;
u2(1:nit/2) = 10; u2(nit/2+1:nit) = 7;

start = input("Start Daqduino? ","s");
if start == "y"
    daqduino_start('COM6'); % Starts DaqDuino board connected to COM7
end
