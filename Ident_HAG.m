%% Indentificação de sistemas dinâmicos
% Método Hägglund (FOPDT)
% clear all; close all; clc

y = yest;
A = 70;
%% Tomar a mÃ©dia da variaÃ§Ã£o no tempo
h = mean(diff(t));
% Achar as variaÃ§Ãµes da resposta para o intervalo de tempo
dy = gradient(y, h);                                            
idx63y = findidx(y, 0.632*y(end))

r = dy(idx63y)*(t-t(idx63y))+y(idx63y);

% Encontrar os indices
idx1 = findidx(r,0)
idx2 = findidx(r,y(end))

% Plotar a reta tangente 
plot(t(idx1:idx2),r(idx1:idx2))
hold on
plot(t(idx63y), y(idx63y), '.r')
plot(t,y)
grid
hold off

%% Coeficientes de Hägglund

theta_hag = t(idx1)
tau_hag = t(idx63y) - t(idx1)

Kp_hag = (y(end)-y(1))/1;

% Função de Transferência Estimada
Ghg = Kp_hag*exp(-theta_hag*s)/(tau_hag*s+1)

figure(2)
step(Ghg)

[y_hag,t_hag] = step(Ghg,t);

% % Cálculo de J e MVI
% [Jhag,MVIhag] = JMVI(y,y_hag)
% 
% % MCI (Model Characterization Index)
% MCIhag = theta_hag/(tau_hag + theta_hag)
figure(3)
plot(t,y_hag,t,y)


% Ghg = exp(-0.32*s) * 70.4 / (0.89 s + 1)
