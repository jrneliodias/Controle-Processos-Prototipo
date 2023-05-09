teta = 0.35;
G = tf(7.737,[1 0],'inputdelay',teta);
opt = stepDataOptions('StepAmplitude',8);
[y_smith,t] = step(G,t,opt);

plot(t,y_smith,t,angulo)



Jgeral = sum((angulo-y_smith').^2)

% Cálculo do MVI (Model Validation Index)
MVIgeral = 1-abs(y_smith'-angulo)/abs(angulo)