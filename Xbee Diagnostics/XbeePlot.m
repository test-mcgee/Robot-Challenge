
function [RSSI, Heading] = XbeePlot(SerialPort,Samples);

% Time interval between each input.
TimeInterval=0.03;
% Set up the serial port object
s = serial(SerialPort)
fopen(s);
% Adjusts the sensitivity of the fitted curve.
rng = 5;

% Initial Variables
error = 0;
RSSI = 0;
Heading = 0;
%% Set up the figure 
figureHandle = figure('NumberTitle','off',...
    'Name','RSSI Levels (Currently Gathering Data)',...
    'Color',[0 0 0],'Visible','off');

% Set axes
axesHandle = axes('Parent',figureHandle,...
    'YGrid','on',...
    'YColor',[0.9725 0.9725 0.9725],...
    'XGrid','on',...
    'XColor',[0.9725 0.9725 0.9725],...
    'Color',[.1 .1 .1]);

hold on;

plotHandle = plot(axesHandle,RSSI,Heading, 'r.');
xlim(axesHandle,[min(0) max(180)]);

% Create xlabel
xlabel('Heading','FontWeight','bold','FontSize',14,'Color',[.8 .8 .8]);

% Create ylabel
ylabel('RSSI','FontWeight','bold','FontSize',14,'Color',[.8 .8 .8]);

% Create title
title('XBee Serial Output','FontSize',15,'Color',[1 1 1]);




%% Reads values from Xbee and plots them 
% Two values are read from serial. First is the Heading and second the RSSI
% These values are stored on arrays. Each Heading/RSSI pair that is
% received is plotted as a point before another pair is received.

count = 1;

% Everything within 'try' will execute as long as there are no errors
try
    
% Will execute Samples amount of times.
while ~isequal(count,Samples+1)
    %%Serial data accessing 
    Heading(count) = fscanf(s,'%f');
    RSSI(count) = fscanf(s,'%f');
    
    %%data pair is plotted
    set(plotHandle,'YData',RSSI,'XData',Heading);
    set(figureHandle,'Visible','on');
    
    %%small pause to allow next transmission
    pause(TimeInterval);
    count = count +1;
end

%% Basic Analysis of data

% If there was no error above, change the label to Finished
if (error == 0) 
    set(figureHandle, 'Name', 'RSSI Levels (Finished)');
end

% Finds the Heading that corresponds to the Maximum RSSI value
[Y, I] = max(RSSI);
fprintf('\nMaximum RSSI and Heading: %d at %d degrees\n', max(RSSI), Heading(I))
    
%% Average Curve
% Takes the average of RSSI values per heading and then averages them
% with the adjecent headings to make a smoother curve.

AverageHeading = 0;
AverageRSSI = 0;

% Take the average of all RSSI values that correspond to each Heading
counter = 1;
for i = 1:180;
    A = Heading == i;
    if (any(A(:) > 0)) ;
        AverageHeading(counter) = i;
        AverageRSSI(counter) = sum(RSSI(A))/size(RSSI(A),2);
        counter = counter + 1;
    end
end

% Take the averaged RSSI values and average them again with the RSSI values 
% of the two adjacent Headings. Provides a smoother curve.
Limit = size(AverageHeading, 2);
lowLimit = (Limit + 1) - rng;
highLimit = 1 + rng;
for i = 1:Limit
    
    if (lowLimit == Limit +1)
        lowLimit = lowLimit-Limit;
    end
    if (highLimit == Limit + 1)
        highLimit = highLimit-Limit;
    end
    
    % Makes adjustment to be able to average end of the arrays
    if (lowLimit>highLimit)
        AverageRSSI(i) = (sum(AverageRSSI(lowLimit:Limit))+sum(AverageRSSI(1:highLimit)))/(rng*2+1);
    end
    % Averages values in the middle of the array
    if (lowLimit<highLimit)
        AverageRSSI(i) = sum(AverageRSSI(lowLimit:highLimit))/(rng*2+1);
    end
    
    lowLimit = lowLimit + 1;
    highLimit = highLimit +1;
end

% Plot the finished product.
plot(AverageHeading,AverageRSSI,'o-');


%% Clean up the serial port

% If there was an error above, it will skip down to here.
catch ME
    error = 1;
    errorMessage = sprintf('Error in function XBee_RealTime().\n\nError Message: %s', ME.message);
	fprintf(1,'%s', errorMessage);
    fprintf('\nSerial communication with Serial Port %s has been closed\n', SerialPort)
end
% Close the Serial Port
fclose(s);
delete(s);
clear s;

end