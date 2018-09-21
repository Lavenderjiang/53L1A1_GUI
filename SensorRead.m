% Read and Plot proximity data in real time
% Via ST 53L1A1 proximity sensor
% Based on Lu Li's Template for Serial reading
% Lavender Jiang

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
timeLimit = 100;
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
s = serial('/dev/cu.usbmodem14123'); 
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
figure
plot(t,range_mm,'-o')
xlabel('Elapsed time (sec)')
ylabel('Range (mm)')
title('Proximity Sensor Output')
set(gca,'xlim',[1 size])

subplot(2,1,1);
h = animatedline;
ax = gca;
ax.YGrid = 'on';

hold on
subplot(2,1,2);
h2 = animatedline;
ax1 = gca;
ax1.YGrid = 'on';
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
    if ((length(readings) == 5) && (str2double(readings(1)) == sensor) ) 
        r = readings(3);
        % r = readings(3); %3rd entry in current thing
        % disp("reading is"); disp(readings);
        % disp("r is "); disp(r);
        r_num = str2double(r); 

        % Add points to animation
        addpoints(h,datenum(t),r_num);
        % Update axes
        ax.XLim = datenum([t-seconds(15) t]);
        datetick('x','keeplimits');
        drawnow
        
        hold on
        addpoints(h2,datenum(t),r_num);
        ax.XLim = datenum([t-seconds(15) t]);
        datetick('x','keeplimits');
        drawnow
        counter_data_right = counter_data_right + 1;
    else
        counter_data_error = counter_data_error + 1;
    end
    
    
    %disp(length(readings));
    %serial_out = fscanf(s);
    %disp("out is");disp(serial_out);
  %{ 
    % convert reading from string to number
    data_num_raw = str2num(serial_out);
    size_data_num_raw = size(data_num_raw);
    
    if (size_data_num_raw(2) == message_length)
        data_num = data_num_raw;
        counter_data_right = counter_data_right+1;
        
        %save data
        log_data_num = [log_data_num; data_num];
    
    else
        counter_data_error = counter_data_error+1;
    end
        
        
    % Plot image
    %if(mod(Counter_loop,5)==0)
        %figure (1);    
        %subplot(2,1,1);

        
        %subplot(2,1,2);


    %end
    
    %end while 1
    %end
    
    % counter increment
    Counter_loop = Counter_loop+1;
    disp(Counter_loop);
    % check loop conditions
    if (Counter_loop > Counter_loop_max)
        flag_loop_enable = 0;        
    end
    
    
    %}
    
    
    
    %pause
    %pause(0.1);
    timePassed = timePassed + 1;
end

%% Plot the recorded data

[timeLogs,tempLogs] = getpoints(h);
timeSecs = (timeLogs-timeLogs(1))*24*3600;
figure
plot(timeSecs,tempLogs)
xlabel('Elapsed time (sec)')
ylabel('Range (\circF)')

%% Exit Block
fclose(s);
delete(s);
clear s;