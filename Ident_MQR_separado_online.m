%% Engenharia de Controle de Processos
%% Identificação MQR Clássico
%clear all;close all;clc

%% iniciar as variáveis
nit = 500; ts = 0.01;
% %% ----- Variável Controlada
% u1(1:nit)  = 7;
% u2(1:nit) = 10;
% 
% u1(1:nit/2) = 9;  u1(nit/2+1:nit) = 7;
% u2(1:nit/2) = 7; u2(nit/2+1:nit) = 9;

u1(1:nit)  = 10;
u2(1:nit) = 7;

a11 = 0;
b01 = 0;
b11 = 0;


b02 = 0;
b12 = 0;
a12 = 0;

erro1 = 0;
erro2 = 0;

yest1 = zeros(1,nit);
yest2 = zeros(1,nit);

angulo(1:nit) = 0;
P1= 10^3*eye(3);
P2= 10^3*eye(3);

teta1 = 0.05*ones(3,1);
teta2 = 0.05*ones(3,1);

%%%%% Processamento do teta - MQR

%%daqduino_start('COM6');

%% Algoritmo MQR

for k=3:nit
  
        angulo(k) = daqduino_read;

        if(angulo(k)<=0 || angulo(k)>90)
            angulo(k) = angulo(k-1);  % Tratar os dados errados
        end

        % Motor 1
        fi1 = [-angulo(k-1); u1(k-1); u1(k-2)];
        erro1 = angulo(k)-teta1'*fi1;
        K1= P1*fi1/(1 + fi1'*P1*fi1);
        teta1 = teta1 + K1*erro1;
        P1 = P1 - K1*fi1'*P1;

        a11 = teta1(1);
        b01 = teta1(2); b11 = teta1(3);
        
        yest1(k) = -a11*yest1(k-1) + b01*u1(k-1) + b11*u1(k-2);
            
        % Motor 2
        fi2 = [-angulo(k-1); u2(k-1); u2(k-2)];
        erro2 = angulo(k)-teta2'*fi2;
        K2= P2*fi2/(1 + fi2'*P2*fi2);
        teta2 = teta2 + K2*erro2;
        P2 = P2 - K2*fi2'*P2;

        a12  = teta2(1);
        b02 = teta2(2); b12 = teta2(3);
        

        yest2(k) = -a12*yest2(k-1) + b01*u2(k-1) + b12*u2(k-2);
         

        % Mandar o sinal de controle para o arduino
        u = [num2str(u1(k)),',',num2str(u2(k)),'\n'];
        daqduino_write(u,ts); % D/A to analog output, and delaying Ts seconds.


  
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
yest_last(1:3)=90;

for k =3:nit

    yest_last(k) = -a11*yest_last(k-1) + b01*u1(k-1) + b11*u1(k-2);

end

for k =3:nit

    yest_last(k) = -a1*yest_last(k-1) -a2*yest_last(k-2) + b10*u1(k-1) + b11*u1(k-2)...
        + b20*u2(k-1) + b21*u2(k-2);

end

%%% somatorio do erro quadratico
e= sum((angulo-yest_last).^2);

%   figure(2)
t = 0:ts:(nit-1)*ts;
figure(1)
plot(t,angulo,'r',t,yest_last,'b'),grid on;
title("Identificação MQR do motor 2")
figure(2)
plot(t,angulo,'r',t,yest,'b'),grid on;

plot(t,angulo,t,yest1-yest2)
