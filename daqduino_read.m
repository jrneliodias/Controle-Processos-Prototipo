%% DAQ-Duino: Matlab side code. Make sure Arduino side is up and running.
%  Author: Prof. Dr. Antonio Silveira (asilveira@ufpa.br)
%  Laboratory of Control and Systems, UFPA (www.ufpa.br)
%  Group of Control and Systems, UDESC (www.udesc.br)
%
% DAQDUINO_READ  Single-shot acquisition by A/D conversion of a single
%                sample from the analog input in use.
%   Example:
%     y = daqduino_read;
%         Stores to y a single sample from the analog channel in use. 
%         The range of the analog input is 0V to 5V.

function [y] = daqduino_read
    global s;
    % READ FROM ARDUINO (data is already in the Matlab buffer)
    y = str2num(fscanf(s,'%s',4));
    flushinput(s);
    % Description: strings shared between Matlab and Arduino
    %              are converted, at both sides, to numbers (floats).
