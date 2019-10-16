%% AMR HW 2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Mert Karakas (UVA)
% AMR 2019 
% Date: 10/11/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc; close all;

%% set up field

M=zeros(100, 100);
goal = [ 20 80 ];
start = [ 80 10 ];
sizeOfM = size(M);
ty = [1:sizeOfM(1)];
tx = [1:sizeOfM(2)];

%% repulsive potential field
M = repulsive_potential_field( M );
M = normalize( M );
figure(1);
mesh( tx, ty, M );
title("Repulsive Potential Field");

%% attractive potential field
M = attractive_potential_field( M, goal );
M = normalize( M );
figure(2);
mesh( tx, ty, M );
title("Attractive Potential Field");

%% visualization
figure(3);
mesh( tx, ty, M );
title("Path towards goal");
hold on
solution = solve( M, start, goal );
plot3( solution(:,2), solution(:,1), solution(:,3)+0.1, "r." );
hold off

%% Functions

function [ dist ] = distance( src, dst )
  dists = [];
  sizeOfDST = size(dst);
  for i=1:sizeOfDST(1)
    dx=0; dy=0;
    dy = src(1) - dst(i,1);
    dx = src(2) - dst(i,2);
    dists(i) = sqrt( dy^2 + dx^2 );
  end
  dist = min( dists );
end

function [ M_normalized ] = normalize( M_raw )
  M_normalized = 1/max(max( M_raw )) * M_raw;
end

function [ ret ] = isinbacktrack( backtrack, y, x )
  sizeOfBacktrack = size(backtrack);
  for i=1:sizeOfBacktrack(1)
    if y == backtrack(i,1)
      if x == backtrack(i,2)
        ret = 1;
        return;
      end
    end
  end
  ret = 0;
end

function [ solution ] = solve( M, start, goal )

  y = start(1);
  x = start(2);
  solution = [ y x M(y,x)];
  backtrack = [];
  idx = 1;
  U = 0;

  while 1
    backtrack = [ backtrack ; y x ];

    y = solution(idx,1);
    x = solution(idx,2);
    idx = idx + 1;

    U = M(y,x);

    sizeOfM = size(M);
    if( (sizeOfM(1) <= y) || 1 == isinbacktrack( backtrack, y+1, x) )
      U_up    = 10;
    else
      U_up    = M( y+1, x );
    end

    if( (1 >= y) || 1 == isinbacktrack( backtrack, y-1, x) )
      U_down  = 10;
    else
      U_down  = M( y-1, x );
    end

    if( (1 >= x) || 1 == isinbacktrack( backtrack, y, x-1) )
      U_left  = 10;
    else
      U_left  = M( y, x-1 );
    end

    if( (sizeOfM(2) <= x) || 1 == isinbacktrack( backtrack, y, x+1) )
      U_right = 10;
    else
      U_right = M( y, x+1 );
    end

    U_next = min( [ U_up U_down U_left U_right ] );
    if( U_next == U_down )
      y = y-1;
    elseif( U_next == U_right )
      x = x+1;
    elseif( U_next == U_up )
      y = y+1;
    elseif( U_next == U_left )
      x = x-1;
    else
      break
    end

    if( y == goal(1,1) && x == goal(1, 2) )
      break;
    end
    solution = [ solution ; y x M(y,x) ];
  end
end

function [ M_ng ] = set_obstacle( M, obs )
  k_rep = 5;
  rho_zero = 10;
  U_rep = 0; U_obs = 0;
  sizeOfM = size(M);
  for yval=1:sizeOfM(1)
    for xval=1:sizeOfM(2)
      dobs = distance( [yval xval], obs);
      if ( 0 == dobs )
        U_rep = k_rep;
      else
        U_rep = M(yval,xval) + 1/2 * k_rep * ( 1/dobs - 1/rho_zero )^2;
        if k_rep < U_rep
          U_rep = k_rep;
        end
      end
      M_ng(yval,xval) = U_rep;
    end
  end
end

function [ M_ng ] = repulsive_potential_field( M )
  fact=5;
  obs = [];
  for i = 1:21
      for j = 1:21
          obs = cat(1, obs, [71 - j 29 + i]);
      end
  end
  M = set_obstacle( M, obs );
  M_ng = M;
end

function [ M_ng ] = attractive_potential_field( M, goal )
  k_att = 10;
  U_att = 0;
  sizeOfM = size(M);
  for yval=1:sizeOfM(1)
    for xval=1:sizeOfM(2)
      % base potential
      rho_goal = distance( [yval xval], goal );

      % attractive
      U_att = 1/2 * k_att * rho_goal^2;

      % potential field
      U(yval, xval) = U_att;
    end
  end
  U = normalize( U );
  M_ng = M + U;
end
