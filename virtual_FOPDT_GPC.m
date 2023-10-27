%% ***********************************************************************
%	Controlador Preditivo Generalizado
% ***********************************************************************

clear; clc; 

%% ----- Condições inciais
nit = 2400; ts = 0.01;

Ny1 = 3; Nu1 = 1; lambda1 = 1;
output_sensor = zeros(1,nit);

% -------- Saturações de potência
max_pot = 15;
min_pot = 7;
    
%% ----- Discretização do Modelo
%Coefiientes do Modelo Smith motor 1
Kpsmith1 = 0.999;
thetasmith1 = 0.65;
tausmith1 = 1.827;


% Função de transferência motor 1
Gm1 = tf(Kpsmith1,[tausmith1 1],'InputDelay',thetasmith1);

% Discretização do modelo
Gmz1 = c2d(Gm1,ts);
Bm1 = Gmz1.num{1}; Am1= Gmz1.den{1};
nbm1 = length(Bm1)-1; nam1 = length(Am1)-1;

% Coeficientes do Modelo
b0m1 = Bm1(2); a1m1 = Am1(2);


%% ----- Referência ----- Degrau
angulo_ref = 50*ones(1,nit+Ny1);
mudref1 = nit/3;
mudref2 = 2*nit/3;
% angulo_ref(1:mudref1) = 20;
% angulo_ref(mudref1+1:mudref2) = 50;
% angulo_ref(mudref2+1:nit+Ny1) = 80;

%% Inicialização do GPC para cada motor
gpc_m1 = GeneralizedPredictiveController(nit,Ny1,Nu1,lambda1,ts,Am1,Bm1);

gpc_m1.calculateController();

%% Iniciar o Protótipo

tbegin = 3; %+ max([1+nb,1+na]);
ent_fut = 1;

%% Calculando os sinais de controle a partir do preditor e saída
for k = tbegin:nit

    % ----- Saída da planta
    output_sensor(k) = -Am1(2:end)*output_sensor(k-1:-1:k-nam1)' + Bm1(2:end)*gpc_m1.u(k-1:-1:k-nbm1)';

    if (ent_fut==0)
        % ---- Referência futura desconhecida
        aux_ref = angulo_ref(k)*ones(1,Ny1)';
    elseif(ent_fut==1)
        % ---- Referência futura conhecida
        aux_ref = angulo_ref(k:k+Ny1-1)';
    end

    fim1 = gpc_m1.H*gpc_m1.du(k-1) + gpc_m1.F(1:end,:)*output_sensor(k:-1:k-nam1)';

    gpc_m1.du(k) = gpc_m1.Kgpc*(aux_ref - fim1);

    gpc_m1.u(k) = gpc_m1.u(k-1) + gpc_m1.du(k);



end



%% Resultados
metodo = 'GPC';
ajuste = '';

t = 0:ts:(nit-1)*ts;
figure(1)
set(gcf,'position',[100 100 600 500])
p = plot(t,output_sensor,'r',t,angulo_ref(1:end-Ny1),'--k');grid
title(["Controle " metodo ajuste])

ylim([0,90])

indices = input("Calcular indices? ","s");
if indices == "y"
    % Settling criteria
    desired_range = [0.95*output_sensor(end), 1.05*output_sensor(end)];  % Desired steady-state range
    tolerance = 0.01;             % Tolerance for settling range

    % Find the indices where the response is within the desired range
    settling_indices = find(output_sensor > desired_range(1) & output_sensor < desired_range(2));

    % Calculate the settling time
    settling_time = t(settling_indices(1)) - t(1);

    % Rise time calculation

    rise_9_index = find(output_sensor>=0.9*output_sensor(end));
    rise_1_index = find(output_sensor >=0.1*output_sensor(end));
    rise_time = t(rise_9_index(1)) - t(rise_1_index(1));

    % Plot the settling time point
    hold on;


    plot(t(settling_indices(1)), output_sensor(settling_indices(1)), 'b.','MarkerSize',20);
    text(t(settling_indices(1)), output_sensor(settling_indices(1))+5,rise_time, ['Settling Time = ', num2str(settling_time)],...
        'VerticalAlignment', 'bottom','HorizontalAlignment','center','Color','b');
    % Plot the rise time point

    plot(t(rise_9_index(1)), output_sensor(rise_9_index(1)), 'k.','MarkerSize',20);
    text(t(rise_9_index(1))+0.2, output_sensor(rise_9_index(1))+0.5, ['Rise Time = ', num2str(rise_time)], 'VerticalAlignment', 'top','HorizontalAlignment','left');
    ylabel("Ângulo (º)")
    xlabel("Tempo (s)")

    legend('Real','Referência')

end

% Potência dos Motores
figure(2)
% set(gcf,'position',[800 100 500 600])
plot(t,gpc_m1.u)
% yticks(6:16)
ylabel("PWM Bandwidth (%)")
xlabel("Tempo (s)")
% ylim([6,16])
grid
title(['Potência do Motor 1 - ' metodo ajuste])




