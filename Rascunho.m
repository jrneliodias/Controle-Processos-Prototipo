 
daqduino_start('COM6'); % Starts DaqDuino board connected to COM7


ts = 0.01;
u0 = [num2str(0),',',num2str(0),'\n']
daqduino_write(u0,ts);
y = daqduino_read



u0 = [num2str(0),',',num2str(7),'\n']
daqduino_write(u,ts);




daqduino_end
