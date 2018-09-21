% Read and Plot proximity data in real time
% Via ST 53L1A1 proximity sensor
% Based on Lu Li's Template for Serial reading
% Based on Madhu Govindarajan's temperature logging webinar element
% Lavender Jiang, September 2018

%% RESET ALL

clear all; % Clear Workspace
clc; % Clear Command Window
close all; % Close plot windows

%% Prepare to Run

% Global Variables
left = 0;
mid = 1;
right = 2;

% Configurable parameter
sensor = left;
timeLimit = 30;
size = 300;

% counting the number of loops, error receive, correct receive
counter_data_right = 0;
counter_data_error = 0;

% saving the output
center_data = zeros(1,size);
left_data = zeros(1,size);
right_data = zeros(1,size);

% 3*size 2D array
sensor_data = [left_data, center_data, right_data];

timePassed = 0;

%% Serial Config

% to find device for linux/mac: 1) cd /dev 2) ls 3)check tty/cu
% Lavender's: second usb port from left
s = serial('/dev/tty.usbmodem14123'); 
set(s,'BaudRate',115200);
set(s,'InputBufferSize',1024);
set(s, 'Terminator', 'LF'); %terminator: line feed \n
%set(s, 'Terminator', 'CR/LF');

% Open Serial
fopen(s);

%% Main Block

%initialize plot components
range_mm = zeros(1,size);
t = zeros(1,size); %tmp x axis
% Plot range versus time

%uncomment to make figure full screen
%figure('units','normalized','outerposition',[0 0 1 1])
%figure %normal size figure
figure('position', [300, 300, 2000, 300]);
%plot(t,range_mm,'-o')
xlabel('Elapsed time (sec)')
ylabel('Range (mm)')
title('Proximity Sensor Output')
set(gca,'xlim',[1 size])

subplot(1,3,1);
h = animatedline(t,range_mm,'Color','b','LineWidth',3);
ax = gca;
ax.YGrid = 'on';

hold on
subplot(1,3,2);
h2 = animatedline(t,range_mm,'Color','r','LineWidth',3);
ax1 = gca;
ax1.YGrid = 'on';


hold on
subplot(1,3,3);
h3 = animatedline(t,range_mm,'Color','g','LineWidth',3);
ax3 = gca;
ax3.YGrid = 'on';
% read start time
startTime = datetime('now');
%ax.YLim = [65 85];
%startTime = datetime('now');

% Main Loop
while(timePassed < timeLimit)
  
    % Get current time
    t =  datetime('now') - startTime;
    
    % read serial for ASCII
    serial_out = fgetl(s);
    readings = strsplit(serial_out,","); % should get 5 readings
    % data format: sensor indicator, range status, range mm, signal rate,
    % ambient rate
    
    %%Trying to plot range mm
    %if data from desired sensor received, draw; otherwise don't add points
    if (length(readings) == 5) 
        r = readings(3);
        r_num = str2double(r);
        sensorIndex = str2double(readings(1));
        % r = readings(3); %3rd entry in current thing
        % disp("reading is"); disp(readings);
        % disp("r is "); disp(r);
         
        if (sensorIndex == left)
            % Add points to animation
            addpoints(h,datenum(t),r_num);
            % Update axes
            ax.XLim = datenum([t-seconds(15) t]);
            datetick('x','keeplimits');
            drawnow
            title("left");
            %hold on
        end
        
        if (sensorIndex == mid)
            addpoints(h2,datenum(t),r_num);
            ax1.XLim = datenum([t-seconds(15) t]);
            datetick('x','keeplimits');
            drawnow
        end 
        %hold on
        
        if (sensorIndex == right)
            addpoints(h3,datenum(t),r_num);
            ax3.XLim = datenum([t-seconds(15) t]);
            datetick('x','keeplimits');
            drawnow
        end 
        
        counter_data_right = counter_data_right + 1;
    else
        counter_data_error = counter_data_error + 1;
   
    end
       
    %pause(0.1);
    timePassed = timePassed + 1;
end

%% Plot the recorded data
figure('position', [300, 300, 2000, 300]);
subplot(1,3,1);
% left
[timeLogs,tempLogs] = getpoints(h);
timeSecs = (timeLogs-timeLogs(1))*24*3600;
plot(timeSecs,tempLogs,'Color','b','LineWidth',3)
xlabel('Elapsed time (sec)')
ylabel('Left Range (\circF)')
title("left sensor")

% center
subplot(1,3,2);
[timeLogs,tempLogs] = getpoints(h2);
timeSecs = (timeLogs-timeLogs(1))*24*3600;
plot(timeSecs,tempLogs,'Color','r','LineWidth',3)
xlabel('Elapsed time (sec)')
ylabel('Middle Range (\circF)')
title("mid sensor")


%right
subplot(1,3,3);
[timeLogs,tempLogs] = getpoints(h3);
timeSecs = (timeLogs-timeLogs(1))*24*3600;
plot(timeSecs,tempLogs,'Color','g','LineWidth',3)
xlabel('Elapsed time (sec)')
ylabel('Right Range (\circF)')
title("right sensor")


%% Exit Block
fclose(s);
delete(s);
clear s;