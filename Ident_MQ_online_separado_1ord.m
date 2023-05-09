%% Sistema em malha aberta

clear all; close all; clc

%% ----- Condições iniciais
nit = 200; ts = 0.01; 
angulo(1:nit) = 0; %u(1:nit) = '0,0\n'; 

% %% ----- Variável Controlada

u1(1:nit)  = 9;
u2(1:nit) = 0;

start = input("Start Daqduino? ","s");
if start == "y"
    daqduino_start('COM9'); % Starts DaqDuino board connected to COM7
end

%% ----- Processamento - Estimação
% start the timer
tic;
for k = 2:nit
    % ----- Saída da planta
    angulo(k) = daqduino_read;

    if angulo(k)>89 % Modificar conforme a simulação

        u = [num2str(0),',',num2str(0),'\n'];
        daqduino_write(u,ts);
    else

        if(angulo(k)<0 || angulo(k)>90)
            angulo(k) = angulo(k-1);  % Tratar os dados errados
        end
        u = [num2str(u1(k)),',',num2str(u2(k)),'\n'];
        daqduino_write(u,ts); % D/A to analog output, and delaying Ts seconds.
    end

end

% stop the timer and display the elapsed time
elapsedTime = toc;
disp(['Elapsed time: ' num2str(elapsedTime) ' seconds']);


% Limpar a memória
daqduino_read
u = [num2str(0),',',num2str(0),'\n'];
daqduino_write(u,ts);

finish = input("Close Daqduino? ","s");
if finish =="y"
    daqduino_end; % End the connection to the DaqDuino device.
end

%% Algoritmo Mínimos Quadrados
t = 0:ts:(nit-1)*ts;

phi = [-[0 ;angulo(1:end-1)'] -[0 ;0;angulo(1:end-2)'] [0 ; u1(1:end-1)'] [0 ; 0; u1(1:end-2)']];

    teta = inv(phi'*phi)*phi'*angulo';

    a1 = teta(1); a2 = teta(2);
    b1 = teta(3); b2 = teta(4);

    yest(1:2)=0;

    for m = 3:nit    
        yest(m) = -a1*yest(m-1)-a2*yest(m-2)+b1*u1(m-1)+b2*u1(m-2);
    end

    plot(t,angulo,'r',t,yest,'-.b','linewidth',1)
    legend('Real','Estimado MQ');
    title('Identificação MQ não recursivo');

    % Somatório do erro quadrático
    e = (angulo - yest);
    SEQmq = sum(e.^2)
    % 
    % % Coeficiente de Correlação multipla
    my = mean(angulo);
    den1 = angulo - my;
    den2 = sum(den1.^2);
    R2mq = 1 - (SEQmq/den2)

%% ----- Plotar sinais
figure(1)
plot(t(1:nit),angulo(1:nit),'r','linewidth',1),grid
title("Simulação Malha Aberta")
legend('Real')
% 
% figure(2)
% plot(t(1:nit),u1(1:nit),'b','linewidth',1),grid
% title("Potência Motor")

myvalues = angulo(angulo>0 & angulo <90);
meanDifff = mean(diff(myvalues))


%%


