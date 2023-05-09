%% Engenharia de Controle de Processos
%% Identificação MQR Clássico
clear all;close all;clc

%% iniciar as variáveis
nit = 400; ts = 0.01;
% %% ----- Variável Controlada
 u1(1:nit)  = 0;
 u2(1:nit) = 7;

%u1(1:nit/2) = 9;  u1(nit/2+1:nit) = 7;
%u2(1:nit/2) = 7; u2(nit/2+1:nit) = 9;

a1 = 0;
a2 = 0;
b0 = 0;
b1 = 0;

fi = zeros(4,1)
erro = 0;
yest(1:nit) = 0;
yest(1:2) = 90;

angulo(1:nit) = 0;
P = 10^6*eye(4);

teta = 0.01*ones(4,1);

%%%%% Processamento do teta - MQR

daqduino_start('COM6');

%% Algoritmo MQR

for k=3:nit
    try
        angulo(k) = daqduino_read;

        if(angulo(k)<=0 || angulo(k)>90)
            angulo(k) = angulo(k-1);  % Tratar os dados errados
        end

        fi = [-angulo(k-1); -angulo(k-2); u2(k-1); u2(k-2)];
        erro = angulo(k)-teta'*fi;
        K= P*fi/(1 + fi'*P*fi);
        teta = teta + K*erro;
        P = P - K*fi'*P;

        a1  = teta(1); a2  = teta(2);
        b0 = teta(3); b1 = teta(4);

        yest(k) = -a1*angulo(k-1) -a2*angulo(k-2) + b0*u2(k-1) + b1*u2(k-2);
            

        % Mandar o sinal de controle para o arduino
        u = [num2str(u1(k)),',',num2str(u2(k)),'\n'];
        daqduino_write(u,ts); % D/A to analog output, and delaying Ts seconds.


    catch ME
    end
end

% Desligar os motores
u = [num2str(0),',',num2str(0),'\n'];
daqduino_write(u,ts);


%%G = tf([b1(end) b2(end)],[1 a1(end) a2(end)],1)


% figure(3)
% subplot(211),plot(t,a1,'r',t,a2,'b'), grid on , legend('a1','a2')
% subplot(212),plot(t,b1,'r'),grid on , legend('b1')



%% Resultados Obtidos


yest_last(1:nit)=0;
yest_last(1:2)=90;
for k =3:nit
     yest_last(k) = -a1*yest_last(k-1) -a2*yest_last(k-2) + b0*u2(k-1) + b1*u2(k-2);

end

%%% somatorio do erro quadratico
e= sum((angulo-yest_last).^2);

%   figure(2)
t = 0:ts:(nit-1)*ts;
figure(1)
plot(t,angulo,'r',t,yest_last,'b'),grid on;
legend('Real','Estimado último valor')
title("Identificação MQR do motor 1 ")
figure(2)
plot(t,angulo,'r',t,yest,'b'),grid on;
legend('Real','Estimado Online')
title("Identificação MQR do motor Separado")


% Resultados
% 


