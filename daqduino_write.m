%% DAQ-Duino: Matlab side code. Make sure Arduino side is up and running.
%  Author: Prof. Dr. Antonio Silveira (asilveira@ufpa.br)
%  Laboratory of Control and Systems, UFPA (www.ufpa.br)
%  Group of Control and Systems, UDESC (www.udesc.br)
%
% DAQDUINO_WRITE  Updates the DaqDuino PWM output in use.
%
%     daqduino_write(u,Ts) sets u Volts, where u range is 0V to 5V,
%     and Ts is the sampling time (a delay) given in seconds.

function [] = daqduino_write(u,Ts)
   global s;
   % WRITE TO ARDUINO (it is going to update its PWM state and send back
   %                   a past state of y(k) - causal system).
   fprintf(s,'%s',u); % Sends u(k) as a string
pause(Ts); % Sampling time interval
               % This is a very important part in order to guarantee
               % all the mathematical background on discrete-time systems.