%% Método de Smith
y = yest;
% y(t2) - tempo t2
yt2 = 0.632*y(end);

% Loop para achar o valor mais próximo
idxt2 = findidx(y,yt2);

% Valor de t2
t2 = t(idxt2);

% y(t1) - tempo t1
yt1 = 0.283*y(end);

idxt1 = findidx(y,yt1);
t1 = t(idxt1);

% Cálculo do ganho Kp
Kpsmith = 5*(y(end)-y(1))/u1(1);

% Cálculo da constante Tau
tausmith = 0.67*(t2-t1);

% Cálculo de theta
thetasmith = 1.5*(t2 - t1);

% Determinar a função de transferência de Sundaresan

s= tf('s')
Gsmith = Kpsmith*exp(-thetasmith*s)/(tausmith*s+1)

% Obter a resposta ao impulso
[ysmith,tsmith] = step(Gsmith,t);

plot(t,y,t,ysmith)

% Cálculo de J

Jgeral = sum((y-ysmith).^2)

% Cálculo do MVI (Model Validation Index)
MVIgeral = 1-abs(ysmith-y)/abs(y)

% MCI (Model Characterization Index)
MCIgeral = thetasmith/(tausmith + thetasmith)

