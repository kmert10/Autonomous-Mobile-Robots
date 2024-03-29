%% Rho(q): Distance to the closest obstacle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Mert Karakas (UVA)
% AMR 2019 
% Date: 10/11/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  return the euclidian distance between 2 points
%  src - one distinct point: [x y]
%  dst - coords of obstacle / destination:
%  [ x1 y1;
%    x2 y2;
%    x3 y3 ]

function [dist, point] = rho(src, dst)
    dists = [];
    points = [];
    sz = size(dst);
    for i = 1:sz(1)
        dx = 0; dy = 0;
        dy = src(2) - dst(i, 2);
        dx = src(1) - dst(i, 1);
        points = cat(1, points, dst(i, :));
        dists(i) = sqrt(dy^2 + dx^2);
    end
    dist = min(dists);
    point = dst(find(dists == dist), :);
end

