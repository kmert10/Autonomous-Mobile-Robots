%% AMR HW 1: Cruise Controller

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Mert Karakas (UVA)
% AMR 2019 
% Date: 09/15/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
close all
clc

%% Parameters

% Workspace Size
xlim([0 200])
ylim([0 200]) 

%Initialize a vector of positions for the robot
x=[]; 
y=[];

%% Robot Initial Pose

M = 13; % M
K = 11; % K
Group8 = pi/(5 + 8); % Group 8

x(1) = M; % Given in HW Doc
y(1) = K;

% Initial Orientation 
theta(1) = Group8; % random theta in radians

% Build Robot Model
robot = RectangularRobot(x,y,theta(1));

plot(robot(:,1),robot(:,2),'-');
xlim([0 200])
ylim([0 200])
    
%% Move Robot

% Goal
xg = 200*rand(1,1);
yg = 200*rand(1,1);
plot(xg, yg, 'r*');

% PID
Kp = 2;
Ki = 1;
Kd = 0.1;

vref = 3;
previous_error = 0;
integral = 0;

vel=[];
u=[];
vel(1) = 0;

%number of steps of the simualtion
nstep = 300;

%time step
t=[0];
dt = 0.1;
i = 1;
count = 0;

while count < 3
    t(i+1) = i*dt;
    
    error = vref - vel(i);
    integral = integral + error * dt;
    derivative = (error - previous_error)/dt;
    u(i) = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    vel(i+1) = vel(i) + u(i)*dt - 0.01*vel(i);
    
    %robot non-holonomic dynamics (as seen in class)
    x(i+1) = x(i) + vel(i) * cos(theta(i)) * dt;
    y(i+1) = y(i) + vel(i) * sin(theta(i)) * dt;
    theta(i+1) = theta(i) +  Kp*(atan2(yg - y(i), xg - x(i)) - theta(i))*dt;
    
    robot = RectangularRobot(x(i),y(i),theta(i));
    plot(robot(:,1),robot(:,2),'-',x,y,'-',xg, yg, 'r*');
    xlim([0 200])
    ylim([0 200])
    pause(0.01)
    if (euclidean_distance(x(i), y(i), xg, yg) <= 1)
        xg = 200*rand(1,1);
        yg = 200*rand(1,1);
        count = count + 1;
    end
    i = i + 1;
end

figure(2);
plot(t,vel);


