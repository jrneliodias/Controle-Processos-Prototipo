%% Extrair os dados da base de dados
angulo = values
%% Tomar a média da variação no tempo
h = mean(diff(t));
% Achar as variações da resposta para o intervalo de tempo
dy = gradient(angulo, h);                                            
[~,idx] = max(dy);
r = dy(idx)*(t-t(idx))+angulo(idx);

% Encontrar os índices
idx1 = findidx(r,0)
idx2 = findidx(r,angulo(end))

% Plotar a reta tangente 
plot(t(idx1:idx2),r(idx1:idx2))
hold on
plot(t(idx), angulo(idx), '.r')
plot(t,angulo)
grid


%% Coeficientes de Ziegler_Nichols

theta_zn = t(idx1);
tau_zn = t(idx2) - t(idx1);

Kp_zn = (angulo(end)-angulo(1))/u2(1)

% Função de Transferência
s = tf('s')
Gzn = Kp_zn*exp(-theta_zn*s)/(tau_zn*s+1)

opt = stepDataOptions('StepAmplitude',8);

[y_zn,t_zn] = step(Gzn,t,opt);

% Cálculo de J e MVI
[Jzn,MVIzn] = JMVI(angulo,y_zn')

plot(t,y_zn,t,angulo)

