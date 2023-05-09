%% Engenharia de Controle de Processos
%% Identificação MQR Clássico
clear all;close all;clc

%% iniciar as variáveis
nit = 200; ts = 0.01;
% %% ----- Variável Controlada
% u1(1:nit)  = 7;
% u2(1:nit) = 15;
% 
% u1(1:nit/2) = 9;  u1(nit/2+1:nit) = 7;
% u2(1:nit/2) = 7; u2(nit/2+1:nit) = 9;


% Sinal de controle (cuidado!!)
 u1(1:nit)  = 10;
 u2(1:nit) = 0;
a1(1:nit) = 0;
a2(1:nit) = 0;
b10(1:nit) = 0;
b11(1:nit) = 0;


erro = 0;
yest(1:nit) = 0;
%yest(1:3) = 90;

angulo(1:nit) = 0;
P = 10^2*eye(4);

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

        fi = [-angulo(k-1); -angulo(k-2); u1(k-1); u1(k-2)];
        erro = angulo(k)-teta'*fi;
        K= P*fi/(1 + fi'*P*fi);
        teta = teta + K*erro;
        P = P - K*fi'*P;
        % Atualização da matriz P - Método Random Walk
        q = trace(P)/4;
        Q = q*eye(4);
        P = P + Q;

        a1(k)  = teta(1); a2(k)  = teta(2);
        b10(k) = teta(3); b11(k) = teta(4);

        yest(k) = -a1(k)*yest(k-1) -a2(k)*yest(k-2) + b10(k)*u1(k-1) + b11(k)*u1(k-2);          

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

% Limpar Memória
daqduino_read
u = [num2str(0),',',num2str(0),'\n'];
daqduino_write(u,ts);



%% Resultados Obtidos

yest_last(1:nit)=0;
%yest_last(1:3)=90;
amostra = 100;
for k =30:nit

    %yest_last(k) = -a1(end-amostra)*yest_last(k-1) -a2(end-amostra)*yest_last(k-2) + b10(end-amostra)*u1(k-1) + b11(end-amostra)*u1(k-2);
    yest(k) = -a1(k)*yest(k-1) -a2(k)*yest(k-2) + b10(k)*u1(k-1) + b11(k)*u1(k-2);

end

a1e = a1(end-amostra)
a2e = a2(end-amostra)
b0e = b10(end-amostra)
b1e = b11(end-amostra)

G = tf([b0e b1e],[1 a1e a2e], ts)
pole(G)

%%% somatorio do erro quadratico
e= sum((angulo-yest).^2);

%   figure(2)
% t = 0:ts:(nit-1)*ts;
% figure(1)
% plot(t,angulo,'r',t,yest_last,'b'),grid on;
% title("Identificação MQR do motor 2 - Última amostra")

% 
figure(2)
plot(t,angulo,'r',t,yest,'b-.'),grid on;
legend('Real','Estimado Online')
title("Identificação MQR do motor Separado")
% Resultados
% 
% u1(1:nit)  = 7;
% u2(1:nit) = 10;


% 
% u1(1:nit)  = 10;
% u2(1:nit) = 7;

G2 = tf(1,[1 0 0])
G2z = c2d(G2,ts)
