% Real time data plot of Xbee RSSI and Heading
%  
% This matlab script is for ploting a graph by accessing serial port data in
% real time. This script calls a function, XbeePlot(), that takes data from the
% user specified serial port for a user specified amount of samples. The
% function then plots the data and shows the Max RSSI/Heading pair.
%
% This scipt will create two variables, RSSI and Heading. Each will be the
% an array of length Samples. The user can use these arrays for
% further data analysis.
%
% Author: Adam St. Amand.
%%

%Clear all variables
clear all;

%Create variables for function call from user input. Serial Port works 
%for different platforms. Below are EXAMPLES of the FORMAT for each 
%platform. User input likely need to be adjusted for specific machince.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Linux and Linux 64            Serial Port = /dev/ttyS0;
%Mac OS X 64                   Serial Port = /dev/tty.KeySerial1;
%Windows 32 and Windows 64     Serial Port = com1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

SerialPort = input('\nWhat is your Serial Port? ', 's');
Samples = input('\nHow many samples would you like to take? ');
pause(5);
%Retreive arrays of length Samples for RSSI and Heading
[RSSI, Heading] = XbeePlot(SerialPort, Samples);

%Cleanup Variables
clearvars SerialPort Samples;
