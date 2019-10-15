%% Normalize values between 0.0 and 1.0

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Mert Karakas (UVA)
% AMR 2019 
% Date: 10/11/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [normalized] = normalize(raw)
    normalized = 1/max(max(raw)) * raw;
end