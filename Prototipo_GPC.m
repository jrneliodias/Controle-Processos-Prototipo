%% ***********************************************************************
%	Controlador Preditivo Generalizado
% ***********************************************************************

clear; clc; 

%% ----- Condições inciais
nit = 200; ts = 0.01;

Ny1 = 20; Nu1 = 2; lambda1 = 1;
Ny2 = 20; Nu2 = 2; lambda2 = 1;
angulo_sensor = zeros(1,nit);

% -------- Saturações de potência
max_pot = 15;
min_pot = 7;
    
%% ----- Discretização do Modelo
%Coefiientes do Modelo Smith motor 1
Kpsmith1 = 7.737;
thetasmith1 =0.65;
tausmith1 =0.6;

% Coefiientes do Modelo Smith motor 2
Kpsmith2 =  12.86;
thetasmith2 = 0.5;
tausmith2 =   0.66;

% Função de transferência motor 1
Gm1 = tf(Kpsmith1,[tausmith1 1],'InputDelay',thetasmith1);

% Discretização do modelo
Gmz1 = c2d(Gm1,ts);
Bm1 = Gmz1.num{1}; Am1= Gmz1.den{1};
nbm1 = length(Bm1)-1; nam1 = length(Am1)-1;

% Coeficientes do Modelo
b0m1 = Bm1(2); a1m1 = Am1(2);

% Função de transferência motor 2
Gm2 = tf(Kpsmith2,[tausmith2 1],'InputDelay',thetasmith2);

% Discretização do modelo
Gmz2 = c2d(Gm2,ts);
Bm2 = Gmz2.num{1}; Am2= Gmz2.den{1};
nbm2 = length(Bm2)-1; nam2 = length(Am2)-1;

% Coeficientes do Modelo
b0m2 = Bm2(2); a1m2 = Am2(2);


%% ----- Referência ----- Degrau
angulo_ref = ones(1,nit+Ny1);
angulo_ref(1:100) = 50; angulo_ref(101:200) = 50; angulo_ref(201:nit+Ny1) = 50;

%% Inicialização do GPC para cada motor
gpc_m1 = GeneralizedPredictiveController(nit,Ny1,Nu1,lambda1,ts,Am1,Bm1);
gpc_m2 = GeneralizedPredictiveController(nit,Ny2,Nu2,lambda2,ts,Am2,Bm2);

gpc_m1.calculateController();
gpc_m2.calculateController();

%% Iniciar o Protótipo
start = input("Start Daqduino? ","s");
if start == "y"
    daqduino_start('COM9'); % Starts DaqDuino board connected to COM7
end

limpar = input("Limpar memória? ","s");
if limpar == "y"

    % Limpar Serial
    daqduino_read;
    u0 = [num2str(0),',',num2str(0),'\n'];
    daqduino_write(u0,ts);

    daqduino_read;
    daqduino_write(u0,ts);
end


tbegin = 3; %+ max([1+nb,1+na]);
ent_fut = 1;

%% Calculando os sinais de controle a partir do preditor e saída
for k = tbegin:nit

    % ----- Saída da planta
    angulo_sensor(k) = daqduino_read;

    if (ent_fut==0)
        % ---- Referência futura desconhecida
        aux_ref = angulo_ref(k)*ones(1,Ny1)';
    elseif(ent_fut==1)
        % ---- Referência futura conhecida
        aux_ref = angulo_ref(k:k+Ny1-1)';
    end

    fim1 = gpc_m1.H*gpc_m1.du(k-1) + gpc_m1.F(1:end,:)*angulo_sensor(k:-1:k-nam1)';
    fim2 = gpc_m2.H*gpc_m2.du(k-1) + gpc_m2.F(1:end,:)*angulo_sensor(k:-1:k-nam2)';

    gpc_m1.du(k) = gpc_m1.Kgpc*(aux_ref - fim1);
    gpc_m2.du(k) = gpc_m2.Kgpc*(aux_ref - fim2);

    gpc_m1.u(k) = gpc_m1.u(k-1) + gpc_m1.du(k);
    gpc_m2.u(k) = gpc_m2.u(k-1) - gpc_m2.du(k);

    % -------- Saturações de potência
   
    gpc_m1.u(k) = max(min_pot, min(gpc_m1.u(k),max_pot));
    gpc_m2.u(k) = max(min_pot, min(gpc_m2.u(k),max_pot));

    % Mandar sinal de controle para os Motores
    u0 = [num2str(gpc_m1.u(k)),',',num2str(gpc_m2.u(k)),'\n'];
    daqduino_write(u0,ts);

    if(angulo_sensor(k)<=0 || angulo_sensor(k)>90)
        angulo_sensor(k) = angulo_sensor(k-1);  % Tratar os dados errados
    end


end


% Limpar a Serial
daqduino_read
u0 = [num2str(0),',',num2str(0),'\n'];
daqduino_write(u0,ts);


%% Resultados
metodo = 'GPC';
ajuste = '';

t = 0:ts:(nit-1)*ts;
figure(1)
set(gcf,'position',[100 100 600 500])
p = plot(t,angulo_sensor,'r',t,angulo_ref(1:end-Ny1),'--k');grid
title(["Controle " metodo ajuste])

ylim([0,90])

indices = input("Calcular indices? ","s");
if indices == "y"
    % Settling criteria
    desired_range = [0.95*angulo_sensor(end), 1.05*angulo_sensor(end)];  % Desired steady-state range
    tolerance = 0.01;             % Tolerance for settling range

    % Find the indices where the response is within the desired range
    settling_indices = find(angulo_sensor > desired_range(1) & angulo_sensor < desired_range(2));

    % Calculate the settling time
    settling_time = t(settling_indices(1)) - t(1);

    % Rise time calculation

    rise_9_index = find(angulo_sensor>=0.9*angulo_sensor(end));
    rise_1_index = find(angulo_sensor >=0.1*angulo_sensor(end));
    rise_time = t(rise_9_index(1)) - t(rise_1_index(1));

    % Plot the settling time point
    hold on;


    plot(t(settling_indices(1)), angulo_sensor(settling_indices(1)), 'b.','MarkerSize',20);
    text(t(settling_indices(1)), angulo_sensor(settling_indices(1))+5,rise_time, ['Settling Time = ', num2str(settling_time)],...
        'VerticalAlignment', 'bottom','HorizontalAlignment','center','Color','b');
    % Plot the rise time point

    plot(t(rise_9_index(1)), angulo_sensor(rise_9_index(1)), 'k.','MarkerSize',20);
    text(t(rise_9_index(1))+0.2, angulo_sensor(rise_9_index(1))+0.5, ['Rise Time = ', num2str(rise_time)], 'VerticalAlignment', 'top','HorizontalAlignment','left');
    ylabel("Ângulo (º)")
    xlabel("Tempo (s)")

    legend('Real','Referência')

end

% Potência dos Motores
figure(2)
set(gcf,'position',[800 100 500 600])
subplot(211)
plot(t,gpc_m1.u)
yticks(6:16)
ylabel("PWM Bandwidth (%)")
xlabel("Tempo (s)")
ylim([6,16])
grid
title(['Potência do Motor 1 - ' metodo ajuste])

subplot(212)
plot(t,gpc_m1.u)
ylim([6,16])
yticks(6:16)
ylabel("PWM Bandwidth (%)")
xlabel("Tempo (s)")
grid
title(['Potência do Motor 2 - ' metodo ajuste])


