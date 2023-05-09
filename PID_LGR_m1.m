%% Questão 4 - Projeto de um PID

Kpsmith = 7.737
thetasmith =0.65
tausmith =0.6

s= tf('s');
Gsmith = Kpsmith*exp(-thetasmith*s)/(tausmith*s+1);
Gsmith2 = pade(Gsmith,1)
step(Gsmith2)
% Obter ganho para overshoot requerido
Mpdes = 0.015;
csi = -log(Mpdes)/sqrt(pi^2+log(Mpdes)^2);

% Tempo de acomodação desejado
ts = 2/3

wnd = 4/(ts*csi)

%Polo desejado
sdes = wnd*exp(j*(pi-acos(csi)))

% Transformar a função de trasnferência em simbolica
syms s
[num,den] = tfdata(Gsmith2);

g4(s) = vpa((poly2sym(cell2mat(num),s)/poly2sym(cell2mat(den),s)));

% Ganho não compensado
rlocus(Gsmith2)
kncomp = rlocfind(Gsmith2,-4+2.99*j)

% Obtendo a fase do polo do compensador LEAD
comp_plant = eval(g4(sdes))
phase_plant = angle(comp_plant)*180/pi

if phase_plant >0 
    phase_comp = 180-phase_plant
else
    phase_comp = abs(-180-phase_plant)
end


% Calculo do zero do PD
zpd = abs(real(sdes)-imag(sdes)/tand(phase_comp))

% Cálculo ganho do compensador PD
kpd = eval(abs(1/((sdes+zpd)*g4(sdes))))

% TF do compensador PD
Gpd = tf([1 zpd],1)

% Teste para verificar as especificações
G4pd = kpd * Gpd*Gsmith2

G4pdmf = feedback(G4pd,1)

step(G4pdmf)

% Definindo o PI como zero em 2.1

Gpi = tf([1 2.1],[1 0])

% Calculo do ganho final
syms s
[num,den] = tfdata(Gpi*Gpd*Gsmith2);

g4pid(s) = vpa((poly2sym(cell2mat(num),s)/poly2sym(cell2mat(den),s)),3);

kpid = eval(1/abs(g4pid(sdes)))

% Verificação da resposta final

G4pid = kpid *Gpi* Gpd *Gsmith2

G4pidmf = feedback(G4pid,1)
G4mf = feedback(Gsmith2*kncomp,1)


opt = stepDataOptions('StepAmplitude',8);
step(G4pidmf, Gsmith,opt)
legend('Com PID via LGR', 'Malha Aberta Simulado')


