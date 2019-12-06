%% AMR HW 3: Kalman Filter & Particle Filer

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Mert Karakas (UVA)
% AMR 2019 
% Date: 11/04/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Kalman Filter Solution
close all;
 
dt=0.3; 
vel_ref=5;
A=[1 dt  0  0;
   0  1  0  0;
   0  0  1  dt;
   0  0  0  1];
v_sigma=1;
B=[1/2*dt^2 0; dt 0;0 1/2*dt^2; 0 dt];
u=[0; 0];
Q=diag([0,v_sigma,0,v_sigma]);
C1=[1 0 0 0; 0 0 1 0]; 
u1_sigma=6;
C2=[1 0 0 0; 0 0 1 0]; 
u2_sigma=4;
C=[C1; C2];
R=[u1_sigma*eye(2), zeros(2,2); zeros(2,2), u2_sigma*eye(2)];
fig=figure(1);
set(fig,'position', [100 100 1400 1400]);
sub_left=subplot(5,3,[1,2,3,4,5,6]);title('Workspace');
axis([0 100 0 100]);
grid on;
hold on;
daspect([10 10 1]);
start=[25,75];
filledCircle(start,1,20,'r');
goal=[25+50*cos(-0.5),50+50*sin(-0.5)];
filledCircle(goal,1,20,'g');
sub_vel=subplot(5,3,[10,11,12]);
title('Estimated Linear Velocity');
axis([0 60 0 15]);
grid on;hold on;
daspect([80 60 1]);
sub_xy=subplot(5,3,[7,8,9]);
title('Estimated Position');
axis([0 60 0 100]);
grid on;
hold on;
daspect([100 1100 1]);
sub_d=subplot(5,3,[13,14,15]);
title('Distance to the Goal');
axis([0 60 0 100]);
grid on;
hold on;
daspect([100 1100 1]);
orient=atan2((goal(2)-start(2)),((goal(1)-start(1))));
vel_refx=vel_ref*cos(orient);
vel_refy=vel_ref*sin(orient);
X_true=[start(1); 0; start(2); 0];
X_est=X_true + [normrnd(0,sqrt(u2_sigma)); normrnd(0,sqrt(v_sigma)); normrnd(0,sqrt(u2_sigma)); normrnd(0,sqrt(v_sigma))];
P_est=zeros(4,4);
Kpv =1.5;
Kiv =0.01;
Kdv =0.01;
error_oldx=0;
Input_v_Ix=0;
error_newx=vel_refx-X_est(2);
Input_v_Px=error_newx;
Input_v_Ix=Input_v_Ix+error_newx*dt;
Input_v_Dx=(error_newx-error_oldx)/dt;
ax=Kpv*Input_v_Px+Kiv*Input_v_Ix+Kdv*Input_v_Dx;
error_oldx=error_newx;
error_oldy=0;
Input_v_Iy=0;
error_newy=vel_refy-X_est(4);
Input_v_Py=error_newy;
Input_v_Iy=Input_v_Iy+error_newy*dt;
Input_v_Dy=(error_newy-error_oldy)/dt;
ay=Kpv*Input_v_Py+Kiv*Input_v_Iy+Kdv*Input_v_Dy;
error_oldy=error_newy;
d=sqrt((goal(1)-X_est(1))^2+(goal(2)-X_est(3))^2);
X_true_query = [X_true];
X_estimation_query = [X_est];
Y_measured_query=[X_true(1);X_true(3);X_true(1);X_true(3);];
X_true_old=X_true;
X_est_old=X_est;
d_old=d;
t=1;
while(d > 1.5)
    ut=[ax; ay;];
    X_true=A*X_true + B*(ut + [normrnd(0,sqrt(v_sigma));  normrnd(0,sqrt(v_sigma))]); 
    Y_measured=[X_true(1)+normrnd(0,sqrt(u1_sigma));
                         X_true(3)+normrnd(0,sqrt(u1_sigma)); 
                        X_true(1)+normrnd(0,sqrt(u2_sigma));
                          X_true(3)+normrnd(0,sqrt(u2_sigma))];
    subplot(5,3,[1,2,3,4,5,6]);Measured_robot1 = plot(Y_measured(1),Y_measured(2),'.g');
    subplot(5,3,[1,2,3,4,5,6]);Measured_robot2 = plot(Y_measured(3),Y_measured(4),'.b');
    X_prd= A * X_est + B*ut; 
    P_prd=A * P_est * A' + Q;
    K = ( P_prd * C' ) / ( C * P_prd * C'+R);
    X_est = X_prd + K * (Y_measured - C * X_prd);
    P_est = P_prd - K * C * P_prd;
    subplot(5,3,[1,2,3,4,5,6]);
    real=plot([X_true_old(1),X_true(1)],[X_true_old(3),X_true(3)],'-black');
    est=plot([X_est_old(1),X_est(1)],[X_est_old(3),X_est(3)],'-r');
    legend([Measured_robot1,Measured_robot2,real,est],'sensor1','sensor2','real state','estimation'); 
    error_newx=vel_refx-X_true(2);
    Input_v_Px=error_newx;
    Input_v_Ix=Input_v_Ix+error_newx*dt;
    Input_v_Dx=(error_newx-error_oldx)/dt;
    ax=Kpv*Input_v_Px+Kiv*Input_v_Ix+Kdv*Input_v_Dx;
    error_oldx=error_newx;
    error_newy=vel_refy-X_true(4);
    Input_v_Py=error_newy;
    Input_v_Iy=Input_v_Iy+error_newy*dt;
    Input_v_Dy=(error_newy-error_oldy)/dt;
    ay=Kpv*Input_v_Py+Kiv*Input_v_Iy+Kdv*Input_v_Dy;
    error_oldy=error_newy; 
    orient=atan2((goal(2)-X_est(3)),((goal(1)-X_true(1))));
    if orient>pi
        orient=orient-2*pi;
    elseif orient<-pi
        orient=orient+2*pi;
    end
    vel_refx=vel_ref*cos(orient);
    vel_refy=vel_ref*sin(orient);
    subplot(5,3,[7,8,9]);plotx=plot([t-1,t],[X_est_old(1),X_est(1)],'r');ploty=plot([t-1,t],[X_est_old(3),X_est(3)],'b');legend([plotx,ploty],'estimated x','estimated y');
    subplot(5,3,[10,11,12]);plotv=plot([t-1,t],[sqrt(X_est_old(2)^2+X_est_old(4)^2),sqrt(X_est(2)^2+X_est(4)^2)],'r');legend(plotv,'estimated velocity');
    d=sqrt((goal(1)-X_est(1))^2+(goal(2)-X_est(3))^2); % robot's distance from the goal
    subplot(5,3,[13,14,15]);plotd=plot([t-1,t],[d_old,d],'r');legend(plotd,'distance to the goal');
    t=t+1;
    X_true_old=X_true;
    X_est_old=X_est;
    d_old=d;
    X_true_query=[X_true_query, X_true ];
    X_estimation_query = [X_estimation_query, X_est ];
    Y_measured_query = [Y_measured_query, Y_measured ];
    pause(0.001)
end


fig5=figure(5);
set(fig5,'position',[1 1 1000 300]);
grid on;
hold on;
axis([0 t+6 20 90]);
tt=0:1:(t-1);

sensor1_x=plot(tt,Y_measured_query(1,:),'.-g','MarkerSize',6);
sensor2_x=plot(tt,Y_measured_query(3,:),'.-b','MarkerSize',6);
estimation_x=plot(tt,X_estimation_query(1,:),'.-r','MarkerSize',6);
legend([sensor1_x,sensor2_x,estimation_x],'sensor1','sensor2','estimated x');


fig6=figure(6);
set(fig6,'position',[1 1 1000 300]);
grid on;
hold on;
axis([0 t+6 20 90]);
sensor1_y=plot(tt,Y_measured_query(2,:),'.-g','MarkerSize',6);
sensor2_y=plot(tt,Y_measured_query(4,:),'.-b','MarkerSize',6);
estimation_y=plot(tt,X_estimation_query(3,:),'.-r','MarkerSize',6);
legend([sensor1_y,sensor2_y,estimation_y],'sensor1','sensor2','estimated y');

fig7=figure(7);
set(fig7,'position',[1 1 600 600]);
tt=0:1:(t-1);
axis([0 100 0 100]);
grid on;
hold on;
sensor1=plot(Y_measured_query(1,:),Y_measured_query(2,:),'.g');
sensor2=plot(Y_measured_query(3,:),Y_measured_query(4,:),'.b');
estimation=plot(X_estimation_query(1,:),X_estimation_query(3,:),'.-r');
legend([sensor1,sensor2,estimation],'Sensor1','Sensor2','Estimate');

%% Particle Filter

forward_sigma = 0.5;
rotation_sigma = pi*0.5/180;
measurement_sigma= 8;

color_real_line=1*[0 0 0];
color_est_line=1*[0.7 0.2 0.9];

turning_speed=0.08;
move_forward_speed=1.5;

fig=figure(8);
set(fig,'position',[100 100 1000 1200]);
subplot(3,2,[1,2,3,4]);title('Workspace');axis([0 100 0 100]);
grid on;
hold on;
daspect([40 40 1]);
landmarks =[25 25 70 70 10 80; 25 70 25 70 40 60]';

Robot_real0 = [50+(20*rand(1)-10), 50+(20*rand(1)-10), 2*pi*rand(1)];
real_pose=tri([Robot_real0(1);Robot_real0(2)],Robot_real0(3),4,'black');

N = 1000;
Particles= zeros(N,3);  % poses of Particles 
Particles(:,1:2)=100*rand(N,2); %  random pose - x,y
Particles(:,3)=2*pi*rand(N,1); %  random pose - theta


Robot_est0=[100*rand(1,2),2*pi*rand];% 
Robot_est0 = mean(Particles);
est_pose=tri([Robot_est0(1);Robot_est0(2)],Robot_est0(3),4,color_est_line);

iteration=60;

Robot_actual = zeros(iteration+1,3);
Robot_est = zeros(iteration+1,3);
Robot_actual(1,:) = Robot_real0;% robot true pose at each step
Robot_est(1,:) = Robot_est0;% robot pose at each step

error=zeros(1,iteration+1);
error(1)=sqrt( (Robot_est(1,1)-Robot_actual(1,1))^2 +(Robot_est(2,1)-Robot_actual(2,1))^2 );  
errorx=zeros(1,iteration+1);
errorx(1)=abs(Robot_est(1,1)-Robot_actual(1,1));  
errory=zeros(1,iteration+1);
errory(1)=abs(Robot_est(1,2)-Robot_actual(1,2));
errortheta=zeros(1,iteration+1);
errortheta(1)=abs(Robot_est(1,3)-Robot_actual(1,3));
subplot(3,2,[5,6]);
axis([0 iteration+5 -20 20]);
grid on;
hold on;
daspect([30 40 1])

subplot(3,2,[1,2,3,4]);
plot_s = plot(Robot_actual(1,1), Robot_actual(1,2), '.-black','Markersize',0.1); hold on;
plot_p = plot(Particles(:,1), Particles(:,2), '.r','Markersize',0.1);
plot_e = plot(Robot_est(1,1), Robot_est(1,2), '.-','Color',color_est_line,'Markersize',0.1);
plot(landmarks(:,1), landmarks(:,2), 'bp');

for i=1:iteration
    delete(real_pose);
    [Robot_actual(i+1,:),Particles]  =MotionUpdate(Robot_actual(i,:) , Particles, turning_speed, move_forward_speed, forward_sigma, rotation_sigma);
    real_pose=tri(Robot_actual(i+1,1:2),Robot_actual(i+1,3),4,color_real_line);
    measurements = measure_landmarks(Robot_actual(i+1,:), landmarks, measurement_sigma);
    weights = computing_weights(Particles, measurements, landmarks, measurement_sigma);
    weights=weights/sum(weights);
    index = randi(N,1);
    beta = 0;
    Particles_resampled = zeros(size(Particles));
    for j=1:N
        beta = beta + rand() * 2*max(weights);
        while beta > weights(index)
            beta = beta - weights(index);
            index = mod(index+1, N)+1;
        end
        Particles_resampled(j,:) = Particles(index,:);
    end
    Particles=Particles_resampled;
    Robot_est(i+1,:)= mean(Particles);
    delete(est_pose);      
    est_pose=tri(Robot_est(i+1,1:2),Robot_est(i+1,3),4, color_est_line);
    error(i+1) = sqrt( (Robot_est(i+1,1)-Robot_actual(i+1,1))^2 +(Robot_est(i+1,2)-Robot_actual(i+1,2))^2 ); 
    errorx(i+1) = (Robot_est(i+1,1)-Robot_actual(i+1,1));
    errory(i+1) = (Robot_est(i+1,2)-Robot_actual(i+1,2));
    errortheta(i+1) = (Robot_est(i+1,3)-Robot_actual(i+1,3));
    subplot(3,2,[5,6]);
    errorpplt=plot([i-1,i],[error(i),error(i+1)],'r');
    errorppltx=plot([i-1,i],[errorx(i),errorx(i+1)],'g');
    errorpplty=plot([i-1,i],[errory(i),errory(i+1)],'b');
    errorpplttheta=plot([i-1,i],[errortheta(i),errortheta(i+1)],'y');
    legend([errorpplt,errorppltx,errorpplty,errorpplttheta],'Distance error','x-error','y-error','thera-error');
    subplot(3,2,[1,2,3,4]);
    set(plot_p, 'XData', Particles(:,1)); set(plot_p, 'YData', Particles(:,2)); set(plot_s, 'XData', [get(plot_s, 'XData') Robot_actual(i+1,1)]); set(plot_s, 'YData', [get(plot_s, 'YData') Robot_actual(i+1,2)]); set(plot_e, 'XData', [get(plot_e, 'XData') Robot_est(i+1,1)]); set(plot_e, 'YData', [get(plot_e, 'YData') Robot_est(i+1,2)]);
    pause(0.6)
end

function distances = measure_landmarks(robot_true, landmarks, measurement_sigma)
    distances=zeros(size(landmarks,1),1);
    for i=1:size(landmarks,1)
        distances(i) = sqrt((landmarks(i,1)-robot_true(1,1))^2 + (landmarks(i,2)-robot_true(1,2))^2 );
        distances(i) = distances(i) +normrnd(0,sqrt(measurement_sigma));
    end
    landmarks=landmarks';
    measure1=plot([landmarks(1,1),robot_true(1)],[landmarks(2,1),robot_true(2)],'--','Color',[0.6,0.0,0.8]);
    measure2=plot([landmarks(1,2),robot_true(1)],[landmarks(2,2),robot_true(2)],'--','Color',[0.6,0.7,0.0]);  
    measure3=plot([landmarks(1,3),robot_true(1)],[landmarks(2,3),robot_true(2)],'--','Color',[0.0,0.7,0.8]);
    measure4=plot([landmarks(1,4),robot_true(1)],[landmarks(2,4),robot_true(2)],'--','Color',[0.3,0.5,0.6]);  
    measure5=plot([landmarks(1,5),robot_true(1)],[landmarks(2,5),robot_true(2)],'--','Color',[0.6,0.5,0.3]);
    measure6=plot([landmarks(1,6),robot_true(1)],[landmarks(2,6),robot_true(2)],'--','Color',[0.7,0.1,0.9]);  
    pause(0.1)
    delete(measure1);delete(measure2);delete(measure3);delete(measure4);delete(measure5);delete(measure6)
end

function [robot_next, Particles_next] = MotionUpdate(robot , Particles, turning_speed, forward_speed, translation_sigma, rotation_sigma)
    turn = mod(turning_speed + robot(1,3) + normrnd(0,sqrt(rotation_sigma)), 2*pi);
    forward = forward_speed + normrnd(0,sqrt(translation_sigma));
    robot_next = [robot(1,1) + cos(turn)*forward, robot(1,2) + sin(turn)*forward, turn];
    Particles_next=zeros(size(Particles));
    for p=1:size(Particles,1)
        turn = mod(turning_speed + Particles(p, 3) + normrnd(0,sqrt(rotation_sigma)), 2*pi);
        forward = forward_speed + normrnd(0,sqrt(translation_sigma));
        Particles_next(p,:) =[ Particles(p, 1) + cos(turn)*forward, Particles(p, 2) + sin(turn)*forward, turn];
    end
end

function Obj=tri(P,theta,L,color)
    x=P(1);y=P(2);
    P1=[x+L/2*cos(theta); y+L/2*sin(theta)];
    P2=[x+L/5*sin(theta)-L/2*cos(theta); y-L/5*cos(theta)-L/2*sin(theta)];
    P3=[x-L/5*sin(theta)-L/2*cos(theta); y+L/5*cos(theta)-L/2*sin(theta)]; 
    X=[P1(1) P2(1) P3(1)];
    Y=[P1(2) P2(2) P3(2)];
    Obj=patch(X,Y,color);
end

function graph=drawCircle(x0,y0,radius)
    theta=0:pi/50:2*pi;
    x=x0+radius*cos(theta);
    y=y0+radius*sin(theta);
    graph=plot(x,y,'--b');
end

function weights = computing_weights(Particles, measurement, landmarks, measurement_sigma)
    N = size(Particles,1);
    weights = zeros(N,1);
    for i=1:N      
        x = Particles(i,1);
        y = Particles(i,2);
        prob = 1;
        for l=1:size(landmarks,1)
            distance_i =sqrt( (x - landmarks(l,1))^2 + (y - landmarks(l,2))^2);
            gaussian=exp(-0.5 * ((measurement(l) - distance_i)/measurement_sigma)^2) / (sqrt(2*pi) * measurement_sigma);
            prob = prob *gaussian;
        end
        weights(i)=prob;
    end
end

function h = filledCircle(center,r,N,color)
    THETA=linspace(0,2*pi,N);
    RHO=ones(1,N)*r;
    [X,Y] = pol2cart(THETA,RHO);
    X=X+center(1);
    Y=Y+center(2);
    h=fill(X,Y,color);
    axis square;
end