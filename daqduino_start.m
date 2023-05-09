%% DAQ-Duino, 2013-2016
% Author: Prof. Dr. Antonio Silveira (asilveira@ufpa.br)
% Laboratory of Control and Systems (LACOS), UFPA (www.ufpa.br)

function [] = daqduino_start(commport)
   delete(instrfindall);
   global s;
   s = serial(commport,'BaudRate',250000); % Check you Arduino COMM port
   fopen(s);
   pause(5); % Wait 5 second to stabilize the link
   fprintf(s,'%s','0,0'); % sets the PWM in use (Arduino side) to zero.
                          % Also, Arduino will send the current state of
                          % y(k) to the Matlab buffer. 
disp(['DaqDuino started! Connection is open on port ' +commport]);