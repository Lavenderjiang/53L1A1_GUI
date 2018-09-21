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

% Configurable parameter
timeLimit = 20;
writefile = true;
size = 3; %init vector size

% Global Variables
left = 0;
mid = 1;
right = 2;
timePassed = 0;
rate_scale_factor = 65536.0;
error_counts = 0;

% Plot Init
%initialize plot components
range_mm = zeros(1,size);
t = zeros(1,size); %tmp x axis
% Plot range versus time

%uncomment to make figure full screen
%figure('units','normalized','outerposition',[0 0 1 1])
%figure [bottom, left, width, height]
figure('position', [800, 800, 2000, 300]);
xlabel('Elapsed time (sec)')
ylabel('Range (mm)')
title('Proximity Sensor Output')
set(gca,'xlim',[1 size])

%left init
subplot(1,3,1);
title("left");
h = animatedline(t,range_mm,'Color','b','LineWidth',3);
ax = gca;
ax.YGrid = 'on';
hold on

%mid init
subplot(1,3,2);
title("mid");
h2 = animatedline(t,range_mm,'Color','r','LineWidth',3);
ax1 = gca;
ax1.YGrid = 'on';
hold on

%right init
subplot(1,3,3);
title("right");
h3 = animatedline(t,range_mm,'Color','g','LineWidth',3);
ax3 = gca;
ax3.YGrid = 'on';
% read start time
startTime = datetime('now');
%ax.YLim = [65 85];

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

% Main Loop
while(timePassed < timeLimit)
  
    % Get current time
    t =  datetime('now') - startTime;
    
    % read serial for ASCII
    serial_out = fgetl(s);
    readings = strsplit(serial_out,","); % should get 5 readings
    % data format: 
    % (1) sensor indicator
    % (2) range status
    % (3) range mm
    % (4) signal rate
    % (5) ambient rate
    
    if (length(readings) == 5) 
        %range_reading = readings(3);
        range_status = readNum(readings,2);
        
        signal_rate = readNum(readings,4)*rate_scale_factor; %in Mcps
        ambient_rate = readNum(readings,5)*rate_scale_factor; %in Mcps
        range_number_mm = readNum(readings,3);
        range_number_cm = range_number_mm/10;
        sensorIndex = str2double(readings(1));
        if (range_status ~= 0)
            % disp("error reading!"); disp(range_status); disp(sensorIndex);
            error_counts = error_counts + 1;
        end
         
        if (sensorIndex == left)
            % Add points to animation
            addpoints(h,datenum(t),range_number_cm);
            % Update axes
            ax.XLim = datenum([t-seconds(15) t]);
            datetick('x','keeplimits');
            drawnow
        end
        
        if (sensorIndex == mid)
            addpoints(h2,datenum(t),range_number_cm);
            ax1.XLim = datenum([t-seconds(15) t]);
            datetick('x','keeplimits');
            drawnow
        end 
        
        if (sensorIndex == right)
            addpoints(h3,datenum(t),range_number_cm);
            ax3.XLim = datenum([t-seconds(15) t]);
            datetick('x','keeplimits');
            drawnow
        end      
    end
       
    %pause(0.1);
    timePassed = timePassed + 1;
end

%% Plot the recorded data
figure('position', [800, 800, 2000, 300]);
subplot(1,3,1);
% left
[timeLogs1,tempLogs1] = getpoints(h);
timeSecs1 = (timeLogs1-timeLogs1(1))*24*3600;
plot(timeSecs1,tempLogs1,'Color','b','LineWidth',3)
xlabel('Elapsed time (sec)')
ylabel('Left Range (cm)')
title("left sensor")

% center
subplot(1,3,2);
[timeLogs2,tempLogs2] = getpoints(h2);
timeSecs2 = (timeLogs2-timeLogs2(1))*24*3600;
plot(timeSecs2,tempLogs2,'Color','r','LineWidth',3)
xlabel('Elapsed time (sec)')
ylabel('Middle Range (cm)')
title("mid sensor")

%right
subplot(1,3,3);
[timeLogs3,tempLogs3] = getpoints(h3);
timeSecs3 = (timeLogs3-timeLogs3(1))*24*3600;
plot(timeSecs3,tempLogs3,'Color','g','LineWidth',3)
xlabel('Elapsed time (sec)')
ylabel('Right Range (cm)')
title("right sensor")


%% Save results to a file
if writefile == true
    % to fix: zero entries in xlsw file
    T = table(timeSecs1',tempLogs1','VariableNames',{'Time_sec','Range_cm'});
    filename = 'Left_Range_Data.xlsx';
    % Write table to file 
    writetable(T,filename)
    % Print confirmation to command line
    fprintf('Results table with %g range measurements saved to file %s\n',...
        length(timeSecs1)-size,filename)
    %fprintf('%d errors in total. Results table with %g range measurements saved to file %s\n',...
        %error_counts,length(timeSecs1)-size,filename)
end

%% Exit Block
fclose(s);
delete(s);
clear s;