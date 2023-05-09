%% Indentifica��o de sistemas din�micos
% M�todo H�gglund (FOPDT)
% clear all; close all; clc

y = yest;
A = 70;
%% Tomar a média da variação no tempo
h = mean(diff(t));
% Achar as variações da resposta para o intervalo de tempo
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

%% Coeficientes de H�gglund

theta_hag = t(idx1)
tau_hag = t(idx63y) - t(idx1)

Kp_hag = (y(end)-y(1))/1;

% Fun��o de Transfer�ncia Estimada
Ghg = Kp_hag*exp(-theta_hag*s)/(tau_hag*s+1)

figure(2)
step(Ghg)

[y_hag,t_hag] = step(Ghg,t);

% % C�lculo de J e MVI
% [Jhag,MVIhag] = JMVI(y,y_hag)
% 
% % MCI (Model Characterization Index)
% MCIhag = theta_hag/(tau_hag + theta_hag)
figure(3)
plot(t,y_hag,t,y)


% Ghg = exp(-0.32*s) * 70.4 / (0.89 s + 1)
