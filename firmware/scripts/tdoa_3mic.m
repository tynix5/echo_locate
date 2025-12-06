function [x_source, y_source] = tdoa_3mic(mic1, mic2, mic3, tdoa12, tdoa13, c)
% tdoa_3mic calculates the 2D coordinates of a sound source using TDOA
%
% Inputs:
%   mic1, mic2, mic3 - [x,y] coordinates of 3 microphones
%   tdoa12 - TDOA between mic1 and mic2 (mic2 - mic1)
%   tdoa13 - TDOA between mic1 and mic3 (mic3 - mic1)
%   c - speed of sound in m/s
%
% Outputs:
%   x_source, y_source - estimated coordinates of the sound source

% Distances difference from TDOA
d12 = c * tdoa12;
d13 = c * tdoa13;

% Microphone coordinates
x1 = mic1(1); y1 = mic1(2);
x2 = mic2(1); y2 = mic2(2);
x3 = mic3(1); y3 = mic3(2);

% Setup hyperbola equations (distance difference squared)
% (x - xi)^2 + (y - yi)^2 - (x - x1)^2 - (y - y1)^2 = di^2

syms x y;

eq1 = (x - x2)^2 + (y - y2)^2 - (x - x1)^2 - (y - y1)^2 == d12^2;
eq2 = (x - x3)^2 + (y - y3)^2 - (x - x1)^2 - (y - y1)^2 == d13^2;

sol = solve([eq1, eq2], [x, y]);
x_sol = double(sol.x);
y_sol = double(sol.y);

% Choose the solution closest to the microphones' centroid (if two solutions)
centroid = [(x1 + x2 + x3)/3, (y1 + y2 + y3)/3];
distances = sqrt((x_sol - centroid(1)).^2 + (y_sol - centroid(2)).^2);
[~, idx] = min(distances);

x_source = x_sol(idx);
y_source = y_sol(idx);

fprintf('Estimated source coordinates: x = %.3f, y = %.3f\n', x_source, y_source);

end