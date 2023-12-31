% Operating frequency (Hz)
fc = 77.0e9;

% Transmitted power (W)
Ps = 3e-3;

% Antenna Gain (linear)
G = 10000;

% Minimum Detectable Power
Pe = 1e-10;

% RCS of a car
RCS = 100;

% Speed of light
c = 3*10^8;

% TODO: Calculate the wavelength
lambda = c / fc;  % wavelength calculation using speed of light and frequency

% TODO : Measure the Maximum Range a Radar can see.
% Radar range equation
range = ((Ps * G^2 * lambda^2 * RCS) / (Pe * (4 * pi)^3))^0.25; 

% Print the range to screen
disp(range);
