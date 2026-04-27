% Herbinator
% Landon Wardle
% 4/26/2026
% Graphing data from first test elapsed 8 hours.

T = readmatrix("LogFile.csv");
t = 0:1:size(T, 1)-1;

moisture_data = transpose(T(:, 4));

plot(t, moisture_data);
title("Moisture vs Time");
xlabel("Time (s)");
ylabel("Moisture (%)");
