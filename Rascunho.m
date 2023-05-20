 
daqduino_start('COM6'); % Starts DaqDuino board connected to COM7


ts = 0.01;
u0 = [num2str(0),',',num2str(0),'\n']
daqduino_write(u0,ts);
y = daqduino_read



u0 = [num2str(0),',',num2str(7),'\n']
daqduino_write(u,ts);




daqduino_end

str = '-2,-7599';
numbers = strsplit(str, ',');
number1 = str2double(numbers{1});
number2 = str2double(numbers{2});