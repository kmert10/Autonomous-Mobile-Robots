%% minPotential(U, pi): Lowest Potential closest to point

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Mert Karakas (UVA)
% AMR 2019 
% Date: 10/14/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  Lowest Potential closest to point
%  U - potential field
%  pi - point [x y]
%  pf - point [x y]


function [pf] = minPotential(U, pi)


    % (i - 1, j - 1)  (i, j - 1) (i + 1, j - 1)
    % (i - 1, j)      (i, j)     (i + 1, j)
    % (i - 1, j + 1)  (i, j + 1) (i + 1, j + 1)
    
    Cx = pi(1); % Current X
    Cy = pi(2); % Current Y
    
    if (Cx == 1 && Cy == 1) % Corner Cases
        points = [Cx Cy - 1; Cx + 1 Cy - 1; Cx + 1 Cy;];
        potentials = [U(Cx, Cy - 1) U(Cx + 1, Cy - 1) U(Cx + 1, Cy)];
    elseif(Cx == 100 && Cy == 1)
        points = [Cx - 1 Cy - 1; Cx Cy - 1; Cx - 1 Cy;];
        potentials = [U(Cx - 1, Cy - 1) U(Cx, Cy - 1) U(Cx - 1, Cy)];
    elseif(Cx == 100 && Cy == 100)
        points = [Cx - 1 Cy; Cx - 1 Cy + 1; Cx Cy + 1;];
        potentials = [U(Cx - 1, Cy) U(Cx - 1, Cy + 1) U(Cx, Cy + 1)];
    elseif(Cx == 1 && Cy == 100)
        points = [Cx + 1 Cy; Cx Cy + 1; Cx + 1 Cy + 1;];
        potentials = [U(Cx + 1, Cy) U(Cx, Cy + 1) U(Cx + 1, Cy + 1)];
    elseif(Cx == 1) % Edge Cases
        points = [Cx Cy - 1; Cx + 1 Cy - 1; Cx + 1 Cy; Cx Cy + 1; Cx + 1 Cy + 1;];
        potentials = [U(Cx, Cy - 1) U(Cx + 1, Cy - 1) U(Cx + 1, Cy) U(Cx, Cy + 1) U(Cx + 1, Cy + 1)];
    elseif(Cy == 1)
        points = [Cx - 1 Cy - 1; Cx Cy - 1; Cx + 1 Cy - 1; Cx - 1 Cy; Cx + 1 Cy;];
        potentials = [U(Cx - 1, Cy - 1) U(Cx, Cy - 1) U(Cx + 1, Cy - 1) U(Cx - 1, Cy) U(Cx + 1, Cy)];
    elseif(Cx == 100)
        points = [Cx - 1 Cy - 1; Cx Cy - 1; Cx - 1 Cy; Cx - 1 Cy + 1; Cx Cy + 1;];
        potentials = [U(Cx - 1, Cy - 1) U(Cx, Cy - 1) U(Cx - 1, Cy) U(Cx - 1, Cy + 1) U(Cx, Cy + 1)];
    elseif(Cy == 100)
        points = [Cx - 1 Cy; Cx + 1 Cy; Cx - 1 Cy + 1; Cx Cy + 1; Cx + 1 Cy + 1;];
        potentials = [U(Cx - 1, Cy) U(Cx + 1, Cy) U(Cx - 1, Cy + 1) U(Cx, Cy + 1) U(Cx + 1, Cy + 1)];
    else
        % General Case
        points = [Cx - 1 Cy - 1; Cx Cy - 1; Cx + 1 Cy - 1; Cx - 1 Cy; Cx + 1 Cy; Cx - 1 Cy + 1; Cx Cy + 1; Cx + 1 Cy + 1;];
        potentials = [U(Cx - 1, Cy - 1) U(Cx, Cy - 1) U(Cx + 1, Cy - 1) U(Cx - 1, Cy) U(Cx + 1, Cy) U(Cx - 1, Cy + 1) U(Cx, Cy + 1) U(Cx + 1, Cy + 1)];
    end
    index = potentials == min(potentials);
    if (sum(index) > 1)
        t = points(index, :);
        pf = t(1, :);
    else
        pf = points(index, :);
    end
   
end

