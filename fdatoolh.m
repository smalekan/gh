function Hd = fdatoolh(Fstop)
%FDATOOLH Returns a discrete-time filter object.

% MATLAB Code
% Generated by MATLAB(R) 8.5 and the Signal Processing Toolbox 7.0.
% Generated on: 13-Jan-2018 16:32:06

% Equiripple Highpass filter designed using the FIRPM function.

% All frequency values are in Hz.
Fs = 48000;  % Sampling Frequency

Fpass = 1200;            % Passband Frequency
Dstop = 0.0001;          % Stopband Attenuation
Dpass = 0.057501127785;  % Passband Ripple
dens  = 20;              % Density Factor

% Calculate the order from the parameters using FIRPMORD.
[N, Fo, Ao, W] = firpmord([Fstop, Fpass]/(Fs/2), [0 1], [Dstop, Dpass]);

% Calculate the coefficients using the FIRPM function.
b  = firpm(N, Fo, Ao, W, {dens});
Hd = dfilt.dffir(b);

% [EOF]
