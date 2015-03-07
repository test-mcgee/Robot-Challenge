%%real time data plot from a serial port 
% This matlab script is for ploting a graph by accessing serial port data in
% real time. Change the com values and all variable values accroding to
% your requirements. Dont forget to add terminator in to your serial device program.
% This script can be modified to be used on any platform by changing the
% serialPort variable. 
% Author: Moidu thavot.

%%Clear all variables

clear all;
%%Variables (Edit yourself)

SerialPort='com8'; %serial port
TimeInterval=0.03;%time interval between each input.
loop=1500;%count values
%%Set up the serial port object
s = serial(SerialPort)
fopen(s);



time =now;
voltage = 0;
%% Set up the figure 
figureHandle = figure('NumberTitle','off',...
    'Name','RSSI Levels',...
    'Color',[0 0 0],'Visible','off');

% Set axes
axesHandle = axes('Parent',figureHandle,...
    'YGrid','on',...
    'YColor',[0.9725 0.9725 0.9725],...
    'XGrid','on',...
    'XColor',[0.9725 0.9725 0.9725],...
    'Color',[0 0 0]);

hold on;

plotHandle = plot(axesHandle,time,voltage,'.');


% Create xlabel
xlabel('Vector','FontWeight','bold','FontSize',14,'Color',[1 1 0]);

% Create ylabel
ylabel('RSSI','FontWeight','bold','FontSize',14,'Color',[1 1 0]);

% Create title
title('Real Time Data','FontSize',15,'Color',[1 1 0]);




%% Initializing variables

RSSI(1)=0;
vector(1)=0;
count = 2;
k=1;
while ~isequal(count,loop)
 
    %%Serial data accessing 
    vector(count) = fscanf(s,'%f');
     RSSI(count) = fscanf(s,'%f');
   

%      if (vector(count)> 180)
%          vector(count)=vector(count-1);
%      end 

    set(plotHandle,'YData',RSSI,'XData',vector);
    set(figureHandle,'Visible','on');
    
    pause(TimeInterval);
    count = count +1;
end



%% Clean up the serial port
fclose(s);
delete(s);
clear s;