%% AMR HW 1: Follow a Line

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
vel=[];
dl=[];

%% Robot Initial Pose

M = 13; % M 13
K = 11; % K 11
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

% Line definition
a = -1;b = 1;c = -50;
xl = 0:200;
yl = -(c + a * xl)/b;

plot(xl, yl)

Kt = .078; % .09
Kh = .001; % .001

%number of steps of the simualtion
nstep = 500;

%time step
dt = 0.1;
t=[0];
dl(1) = (a*x(1) + b*y(1) + c)/sqrt(a^2 + b^2);
 
for i = 1:nstep
    t(i+1) = i*dt;
    
    % First Controller
    d = (a*x(i) + b*y(i) + c)/sqrt(a^2 + b^2);
    dl(i+1) = d;
    % Second Controller
    thetaD = atan2(-a, b);
    
    vel(i) = 3;
    theta(i+1) = Kh * (thetaD - theta(i)) - Kt * d;
    
    %robot non-holonomic dynamics (as seen in class)
    x(i+1) = x(i) + vel(i) * cos(theta(i)) * dt;
    y(i+1) = y(i) + vel(i) * sin(theta(i)) * dt;
    
    robot = RectangularRobot(x(i),y(i),theta(i));
    
    plot(robot(:,1),robot(:,2),'-',x,y,'-',xl, yl);
    xlim([0 200])
    ylim([0 200])
    pause(0.01)
    
end

figure(2);
plot(t,-1*dl);


