%% AMR HW 2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Mert Karakas (UVA)
% AMR 2019 
% Date: 10/11/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
close all
clc
hold on
grid on
%% Parameters

% Workspace Size
xlim([0 100])
ylim([0 100])

Qgoal = [80 20];

ObstacleO = [40, 60];

%% Attractive Potential Field 


for i = 1:100
    for j = 1:100
        rg = euclidean_distance(Qgoal(1), Qgoal(2), j, i); 
        U_att(i, j) = 0.5*rg^2;
    end
end

F_att = -U_att;

figure(1);
contour(F_att, 100);

%% Plot Obstacle & Repulsive Potential Field

obs = RectangleObstacle(ObstacleO(1),ObstacleO(2),0);
plot(obs(:,1),obs(:,2),'r-');

% Extra Credit Test
obsSquare = []; % (30 50)
obsSquare = cat(1, obsSquare, obs);
% for i = 1:20
%     obsSquare = cat(1, obsSquare, [i+30 50]);
% end
% for i = 1:20
%     obsSquare = cat(1, obsSquare, [50 i+50]);
% end
% for i = 1:20
%     obsSquare = cat(1, obsSquare, [50-i 70]);
% end
% for i = 1:20
%     obsSquare = cat(1, obsSquare, [30 70-i]);
% end
for i = 1:21
    for j = 1:21
        obsSquare = cat(1, obsSquare, [29 + j 71 - i]);
    end
end


rho_0 = 15; % Distance of influence of the object

F_rep = zeros(100, 100);

for i = 1:100
    for j = 1:100
        [rd, point] = rho([j i], obsSquare);
        if(rd < rho_0)
            U_rep(i, j) = 0.5*(1/rd - 1/rho_0)^2;
            % F_rep(i, j) = ((1/rd - 1/rho_0) * (1/rd^2) * ([j i] - point)/rd);
        else
            U_rep(i, j) = 0;
            % F_rep(i, j) = [0 0];
        end
    end
end
for i = 1:100
    for j = 1:100
        if(U_rep(i, j) == 1/0) % 30 <= j && j <= 50 && 50 <= i && i <= 70
            U_rep(i, j) = 1;
        end
    end
end

U_total = U_att + 3000*U_rep;

Qstart = [10 80];
Qcurrent = Qstart;
path = [Qstart];
itr = 1;
while (Qcurrent ~= Qgoal)
    Qcurrent = minPotential(U_total, Qcurrent);
    path = cat(1, path, Qcurrent);
    itr = itr + 1;
    if (itr > 100*100)
        break;
    end
end
