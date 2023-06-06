%% Nelson Yamaguti - GMV
clear; clc; close all

t1 = 2.5; t2 = 5; t3 = 7.5; Tfinal = 10;
Ts = 1e-2; nit = round(Tfinal/Ts);

% Identified model
a1 = -1.058159163655783; a2 = 0.118039461669947; A = [1, a1, a2];
b10 = 0.142340774939283; b11 = -1.056560480873788; 
b20 = 0.330202747032585; b21 = 0.193618508701437; 

%% GMVC incremental

% Controller Gc
Delta = [1 -1]; % Delta = 1-z^-1
DA = conv(Delta, A); % Delta*Az
da1 = DA(2); da2 = DA(3); da3 = DA(4);
f0 = -da1; f1 = -da2; f2 = -da3;

% Weighting of control effort
q10 = -200; % Controller Gc1
q20 = 200; % Controller Gc2

% Initial simulation conditions
y = zeros(nit, 1); u = zeros(nit, 2); du = zeros(nit, 2);

% Reference
yr(1 : round(t1/Ts)) = 20;
yr(round(t1/Ts) + 1 : round(t2/Ts)) = -10;
yr(round(t2/Ts) + 1 : round(t3/Ts)) = 15;
yr(round(t3/Ts) + 1 : nit + 1) = 0;

%% Control loop

daqduino_start('COM4', 250000) % Open a Serial Port Object

figure; subplot(211)
plot(1:nit+1, yr, ':k', 'linewidth', 2); hold on; grid

for k = 3 : nit
    y(k) = daqduino_read; % Measured Output
%     y(k) = - a1*y(k-1) - a2*y(k-2)...
%            + b10*u(k-1, 1) + b11*u(k-2, 1)...
%            + b20*u(k-1, 2) + b21*u(k-2, 2);...
    
%     Controller 1
    du(k, 1) = (1/(b10+q10))*(yr(k+1) - f0*y(k) - f1*y(k-1)...
               - f2*y(k-2) - b11*du(k-1, 1)); % Control Law
    u(k, 1) = u(k-1, 1) + du(k, 1); % Incremental control
    
%     Controller 2
    du(k, 2) = (1/(b20+q20))*(yr(k+1) - f0*y(k) - f1*y(k-1)...
               - f2*y(k-2) - b21*du(k-1, 2)); % Control Law
    u(k, 2) = u(k-1, 2) + du(k, 2); % Incremental control
    
    % Anti wind-up
    if u(k, 1) >= 5
        u(k, 1) = 5;
    elseif u(k, 1) <= 0
        u(k, 1) = 0;
    end
    
    % Anti wind-up
    if u(k, 2) >= 5
        u(k, 2) = 5;
    elseif u(k, 2) <= 0
        u(k, 2) = 0;
    end

    subplot(211)
    plot(1:k, y(1:k), 'b', 'linewidth', 2)
    subplot(212)
    plot(1:k, u(1:k, 1), 'b', 1:k, u(1:k, 2), 'r', 'linewidth', 2);
    grid
    axis([0 nit 0 Inf])
    drawnow
  
    daqduino_write_Mimo(u(k, 1), u(k, 2), Ts);  % Control Signal
end

daqduino_end % Close a Serial Port Object

%% Plot results

t = 0 : Ts : nit*Ts - Ts;
figure(2)
subplot(211)
    plot(t, yr(1:nit), ':k', t, y, 'b', 'linewidth', 2); grid
    title('Closed-Loop Response')
    ylabel('Angle (°)'); xlabel('Time (s)')
    legend({'Reference (y_r)', 'Angle (°)'}, 'FontSize', 10)
subplot(212)
    plot(t, u(:, 1), 'b', t, u(:, 2), 'r', 'linewidth', 2); grid;
    title('Control Action')
    ylabel('Amplitude'); xlabel('Time (s)')
    legend({'Control (u1)', 'Control (u2)'}, 'FontSize', 10)

%% Performance indices

e = yr(1:nit)' - y ;
ISE = sum(e.^2);
disp(['ISE = ', num2str(ISE)])
ISU = sum(sum(u.^2));
disp(['ISU = ', num2str(ISU)])

sigm_e = sum((e - mean(e)).^2)/nit;
disp(['sigm_e = ', num2str(sigm_e)])
sigm_u(1) = sum((u(:, 1) - mean(u(:, 1))).^2)/nit;
sigm_u(2) = sum((u(:, 2) - mean(u(:, 2))).^2)/nit;
disp(['sigm_u = ', num2str(sum(sigm_u(:)))])