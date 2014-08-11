%%%% Just some MatLab code for the following sensors (in the respective orders): Luminosity, Hydrogen,
%%%% Methane, Humidity & Temperature.
%% for QuAIL team
%% Create the serial object
serialPort = '/dev/tty.usbserial-DA00STAW';
serialObject = serial(serialPort);
fopen(serialObject);

%% Set the instrument in remote mode
fprintf(serialObject,'SYSTEM:REMOTE');

%% Set up the figure window
time = now;
voltage = 0;

%creates new window for lux
figureHandle = subplot(2,3,1);

%set axes for lux

hold on;

plotGLux = plot(time,voltage,'Marker','.','LineWidth',1,'Color',[0 1 0]); %green lux

%%%%plots new graph for hydro
%creates new window for hydro
figureHandle2 = subplot(2,3,2);

%set axes for hydro

hold on;


plotYHydro = plot(time,voltage,'Marker','.','LineWidth',1,'Color',[1 1 0]); % yellow hydro%%
%%plots methane sensor graph
%creates new window for meth
figureHandle3 = subplot(2,3,3);
%set axes for meth

hold on;
plotMMeth = plot(time,voltage,'Marker','.','LineWidth',1,'Color',[1 0 1]);% magenta meth


%%plots humidity sensor graph
%creates new window for hum
figureHandle4 = subplot(2,3,4);

%set axes for hum

hold on;

plotWHum = plot(time,voltage,'Marker','.','LineWidth',1,'Color',[0 1 1]); %cyan hum

%%plots temp sensor graph
%creates new window for lux
figureHandle5 = subplot(2,3,5);


hold on;
plotRTemp = plot(time,voltage,'Marker','.','LineWidth',1,'Color',[1 0 0]); %red temp
 
xlim(figureHandle,[min(time) max(time+0.001)]);
xlim(figureHandle2,[min(time) max(time+0.001)]);
xlim(figureHandle3,[min(time) max(time+0.001)]);
xlim(figureHandle4,[min(time) max(time+0.001)]);
xlim(figureHandle5,[min(time) max(time+0.001)]);


% Create xlabel
xlabel(figureHandle, 'Time','FontWeight','bold','FontSize',14,'Color',[0 0 1]);
xlabel(figureHandle2, 'Time','FontWeight','bold','FontSize',14,'Color',[0 0 1]);
xlabel(figureHandle3, 'Time','FontWeight','bold','FontSize',14,'Color',[0 0 1]);
xlabel(figureHandle4, 'Time','FontWeight','bold','FontSize',14,'Color',[0 0 1]);
xlabel(figureHandle5, 'Time','FontWeight','bold','FontSize',14,'Color',[0 0 1]);

% Create ylabel
ylabel(figureHandle,'Lux','FontWeight','bold','FontSize',14,'Color',[0 0 1]); % Luminosity
ylabel(figureHandle2, 'Parts Per Milliom (ppm)','FontWeight','bold','FontSize',14,'Color',[0 0 1]); % Hydrogen
ylabel(figureHandle3, 'Parts Per Milliom (ppm)','FontWeight','bold','FontSize',14,'Color',[0 0 1]); % Methane
ylabel(figureHandle4, 'Percentage','FontWeight','bold','FontSize',14,'Color',[0 0 1]); % Humidity
ylabel(figureHandle5, 'Termperature (C)','FontWeight','bold','FontSize',14,'Color',[0 0 1]); % Temperature


% Create title
title(figureHandle, 'Luminosity Sensor Data','FontSize',15,'Color',[0 0 1]);
title(figureHandle2, 'Hydrogen Sensor Data','FontSize',15,'Color',[0 0 1]);
title(figureHandle3, 'Methane Sensor Data','FontSize',15,'Color',[0 0 1]);
title(figureHandle4, 'Humidity Sensor Data','FontSize',15,'Color',[0 0 1]);
title(figureHandle5, 'Temperature Sensor Data','FontSize',15,'Color',[0 0 1]);

%% Set the time span and interval for data collection
stopTime = '10/07 21:53';
timeInterval = 0.005;

%% Collect data
count = 1;

while ~isequal(datestr(now,'mm/DD HH:MM'),stopTime)
    time(count) = datenum(clock); 
    fprintf(serialObject,'MEASURE:VOLTAGE:DC?'); % To measure current the command is MEASURE:CURRENT:DC?
    rawStringData = fscanf(serialObject);
    C = strsplit(rawStringData,'|');
    var1 = C{1}; %Luminosity

    var2 = C{2}; % Hydrogen,
    var3 = C{3}; %%%% Methane
    var4 = C{4}; %Humidity 
    var5 = C{5}; % Temperature
    lux(count) = str2double(var1);
    hydro(count) = str2double(var2);
    meth(count) = str2double(var3);
    hum(count) = str2double(var4);
    temp(count) = str2double(var5);
    %fprintf('This is first: %d and this second %d', lux,hydro, meth, hum, temp);
    %voltage(count) = fscanf(serialObject,'%f');  %#ok<SAGROW>
    set(plotGLux,'YData',lux,'XData',time); %sets object properties for lux
    set(plotYHydro,'YData',hydro,'XData',time); %sets object properties for hydro
    set(plotMMeth,'YData',meth,'XData',time); %sets object properties for meth
    set(plotWHum,'YData',hum,'XData',time);%sets object properties for hum
    set(plotRTemp,'YData',temp,'XData',time);%sets object properties for temp
    set(figureHandle,'Visible','on');
    set(figureHandle2,'Visible','on');
    set(figureHandle3,'Visible','on');
    set(figureHandle4,'Visible','on');
    set(figureHandle5,'Visible','on');
    datetick(figureHandle, 'x','mm/DD HH:MM');
    datetick(figureHandle2, 'x','mm/DD HH:MM');
    datetick(figureHandle3, 'x','mm/DD HH:MM');
    datetick(figureHandle4, 'x','mm/DD HH:MM');
    datetick(figureHandle5, 'x','mm/DD HH:MM');
    
    pause(timeInterval);
    count = count +1;
end

%% Put the instrument in local mode
fprintf(serialObject,'SYSTEM:LOCAL');

%% Clean up the serial object
fclose(serialObject);
delete(serialObject);
clear serialObject;
