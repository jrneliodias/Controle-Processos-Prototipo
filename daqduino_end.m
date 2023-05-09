%% DAQ-Duino, 2013-2016
% Author: Prof. Dr. Antonio Silveira (asilveira@ufpa.br)
% Laboratory of Control and Systems (LACOS), UFPA (www.ufpa.br)

function [] = daqduino_end
   global s;
   fprintf(s,'%s','0,0'); % Sends u(k) = 0
   fclose(s); % Disconnect the serial link and put the PWM to zero
disp('DaqDuino ended! Connection was ended on serial port.');