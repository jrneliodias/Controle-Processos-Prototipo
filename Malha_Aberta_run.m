
%% ----- Processamento - Estimação
for k = 2:nit
    % ----- Saída da planta
    try   
        angulo(k) = daqduino_read;
      if(angulo(k)<=0 || angulo(k)>70)
        angulo(k) = angulo(k-1);  % Tratar os dados errados
      end
            u = [num2str(u1(k)),',',num2str(u2(k)),'\n'];
            daqduino_write(u,ts); % D/A to analog output, and delaying Ts seconds.
        
    catch ME

    end
    
end



u = [num2str(0),',',num2str(0),'\n'];
daqduino_write(u,ts);

finish = input("Close Daqduino? ","s");
if finish =="y"
    daqduino_end; % End the connection to the DaqDuino device.
end



%% ----- Plotar sinais
t = 0:ts:(nit-1)*ts;
figure(1)
plot(t(5:nit),angulo(5:nit),'r','linewidth',1),grid
title("Simulação Malha Aberta")
legend('Real')
