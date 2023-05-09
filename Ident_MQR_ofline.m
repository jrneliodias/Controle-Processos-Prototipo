%% Engenharia de Controle de Processos
%% Identificação MQR Clássico
clear all;close all;clc

%% iniciar as variáveis
%nit = 500; ts = 0.01;
% %% ----- Variável Controlada
 u1(1:nit)  = 7;
 u2(1:nit) = 10;

a1 = 0;
a2 = 0;
b10 = 0;
b11 = 0;
b20 = 0;
b21 = 0;

a1 = -0.513458658433547;
a2 = -0.488975595708869;
b10 = 8.808982117032048e-04;
b11 = 8.808742969853931e-04;
b20 = 0.002907559163446;
b21 = 0.002907597446128;

erro = 0;
yest(1:nit) = 0;
P = 10^6*eye(6);

teta = 0.01*ones(6,1);

%%%%% Processamento do teta - MQR

%daqduino_start('COM6');

%% Algoritmo MQR

for k=3:nit
   

        fi = [-angulo(k-1); -angulo(k-2); u1(k-1); u1(k-2); u2(k-1); u2(k-2)];
        erro = angulo(k)-teta'*fi;
        K= P*fi/(1 + fi'*P*fi);
        teta = teta + K*erro;
        P = P - K*fi'*P;

        a1  = teta(1); a2  = teta(2);
        b10 = teta(3); b11 = teta(4);
        b20 = teta(5); b21 = teta(6);

        yest(k) = -a1*angulo(k-1) -a2*angulo(k-2) + b10*u1(k-1) + b11*u1(k-2)...
            + b20*u2(k-1) + b21*u2(k-2);
   
end


%%G = tf([b1(end) b2(end)],[1 a1(end) a2(end)],1)

% for k =3:nit
%   yest(k) = -a1(end)*yest(k-1) - a2(end)*yest(k-2) + b1(end)*u(k-1);
%
% end
%%%% somatorio do erro quadratico
% e= sum((y-yest).^2)

%   figure(2)
t=1:nit;
plot(t,angulo,'r',t,yest,'b'),grid on;


% figure(3)
% subplot(211),plot(t,a1,'r',t,a2,'b'), grid on , legend('a1','a2')
% subplot(212),plot(t,b1,'r'),grid on , legend('b1')

disp(["a1 = " a1])