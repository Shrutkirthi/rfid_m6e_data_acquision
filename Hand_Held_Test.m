clc
clear all
close all

% Parameters for Lines
lineWidth = 3;  % Thickness of the lines

% Line 1: (0,0) -> (0,240)
x1 = [0, 0];
y1 = [0, 800];

% Line 2: (210,0) -> (210,240)
x2 = [210, 210];
y2 = [0, 800];

% Circle Locations
circlesLine1 = [0, 0; 0, 120; 0, 240; 0, 360;0, 480; -20, 600; 0, 765];       % (x, y) positions for Line 1
circlesLine2 = [210, 60; 210, 180; 210, 300; 210, 420; 210, 540; 210, 660; 210, 780];          % (x, y) positions for Line 2
circleRadius = 11;                          % Radius of the circles (cm)

% Initialize Figure
figure(1);
hold on;
grid on;
axis equal;
xlabel('X (cm)');
ylabel('Y (cm)');
title('Interactive Tag and Moving Reader');
xlim([-350, 560]);
ylim([-50, 950]);

% Plot Thick Lines
plot(x1, y1, 'black', 'LineWidth', lineWidth);
plot(x2, y2, 'black', 'LineWidth', lineWidth);

% Initialize Circles
hCirclesLine1 = gobjects(1, size(circlesLine1, 1));
hCirclesLine2 = gobjects(1, size(circlesLine2, 1));

% Plot Circles on Line 1
for i = 1:size(circlesLine1, 1)
    hCirclesLine1(i) = rectangle('Position', ...
        [circlesLine1(i, 1) - circleRadius, circlesLine1(i, 2) - circleRadius, 2 * circleRadius, 2 * circleRadius], ...
        'Curvature', [1, 1], 'FaceColor', '#808080');
end

% Plot Circles on Line 2
for i = 1:size(circlesLine2, 1)
    hCirclesLine2(i) = rectangle('Position', ...
        [circlesLine2(i, 1) - circleRadius, circlesLine2(i, 2) - circleRadius, 2 * circleRadius, 2 * circleRadius], ...
        'Curvature', [1, 1], 'FaceColor', '#808080');
end

% Initialize Moving Object (Red Circle)
hMovingObject = plot(0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
currentPos = [0, 100]; % Starting position of the red dot

%RaspberryPi Configuration
rpi = raspi('piRFID.local', 'pi', 'raspberry');
serialPort = '/dev/ttyACM0';
baudRate = 115200;
serialObj = serialdev(rpi, serialPort, baudRate);
serialObj.Timeout = 5;

userChar = input('Enter a character to send to the serial port: ', 's');

write(serialObj, userChar, 'char'); % Sends the user-entered character

buffer = '';
P(1:3,1:10)=0;
S(1:10)=0;
smooth_win=5;
triX(1:smooth_win)=0;
triY(1:smooth_win)=0;

disp('Reading USB serial data line by line...');
preTagL=1;
preTagR=1;
flag=0;
k=1;
smoothX=0;
smoothY=0;
Nsol1(1,1)=0;
Nsol1(2,1)=0;
oldPos=[0 0];
posCh=100;

while true
    try
        % 100 bytes at a time)
        chunkSize = 200;
        data = read(serialObj, chunkSize, 'uint8');

        buffer = [buffer, data];

        lines = split(buffer, newline);



        for i = 1:10
            flag=1;
            %disp(lines{1});
            values = split(lines{i}, ',');
            if numel(values) == 4
                if values{1}(1) == "L"
                    tx=circlesLine1(str2double(values{1}(2)),1);

                    ty=circlesLine1(str2double(values{1}(2)),2);
                elseif values{1}(1) == "R"
                    tx=circlesLine2(str2double(values{1}(2)),1);

                    ty=circlesLine2(str2double(values{1}(2)),2);
                end
                %                 tx = str2double(values{2});
                %                 ty = str2double(values{3});
                trssi = str2double(values{4});
                tdis = 0.6064*exp(-0.0915*trssi);

                P(1,i)=tx;
                P(2,i)=ty;
                P(3,i)=0;
                S(i)=tdis;

                %                 fprintf('Parsed Values: ');
                %                 disp([P(:,i)',S(i)]);

                W = diag(ones(1,length(S)));

                if trssi>-59&&length(S)>=3
                    %                     clear N1;
                    %                     clear N2;
                    [N1, N2] = Trilateration(P,S,W);
                    fprintf('\nDirect:');
                    Nsol1 = N1(2:4,1);
                    disp(Nsol1')

                    %Animate Red Dot
                    targetPos = [Nsol1(1,1), Nsol1(2,1)];
                    delPos=targetPos-oldPos;
                    if (-posCh<delPos(1)&&delPos(1)<posCh&&-posCh<delPos(2)&&delPos(1)<posCh&&i>=5)
                        set(hMovingObject, 'XData', targetPos(1), 'YData', targetPos(2));

                    end
                    oldPos=targetPos;



                    smoothX=smooth(triX);
                    smoothY=smooth(triY);
                end



                %                 if(flag>=50)
                %                     Nmat0 = RecTrilateration(P,S,W);
                %                     Nmat = Nmat0(:,1:5);
                %                     fprintf('\n Recursive solutions \n');
                %                     disp(Nmat(2:4,:))
                %                 end

                %Vizualize
                % Check Line 1 Circles
                set(hCirclesLine1(preTagL), 'FaceColor', '#808080'); % Revert to gray
                if values{1}(1) == "L"
                    preTagL=str2double(values{1}(2));
                    set(hCirclesLine1(preTagL), 'FaceColor', 'green'); % Change to green

                end

                % Check Line 2 Circles
                set(hCirclesLine2(preTagR), 'FaceColor', '#808080'); % Revert to gray
                if values{1}(1) == "R"
                    preTagR=str2double(values{1}(2));
                    set(hCirclesLine2(preTagR), 'FaceColor', 'green'); % Change to green
                end


            end
        end

        buffer = lines{end};

    catch ME
        disp('Error reading USB data:');
        disp(ME.message);
        break;
    end
end

% Cleanup
clear serialObj;
disp('USB serial communication ended.');