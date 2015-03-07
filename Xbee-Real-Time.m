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
SerialPort='com7'; %serial port
TimeInterval=0.3;%time interval between each input.
loop=2000;%count values

%%Set up the serial port object
s = serial(SerialPort)
fopen(s);

RSSI = 0;
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

plotHandle = plot(axesHandle,RSSI,voltage,'.');


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

try
while ~isequal(count,loop)
    
    %%Serial data accessing 
    vector(count) = fscanf(s,'%f');
    RSSI(count) = fscanf(s,'%f');

    set(plotHandle,'YData',RSSI,'XData',vector);
    set(figureHandle,'Visible','on');
    
    pause(TimeInterval);
    count = count +1;
end

catch
    warning('Script terminated: Serial connection with %s closed', SerialPort);
    fclose(s);
    delete(s);
    clear s;
    return;
end

%% Clean up the serial port

fclose(s);
delete(s);
clear s;