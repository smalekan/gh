function part1
clear all; close all; clc
%% parameters
Fs =12000;                                  % sampling frequency
Ts = 1/Fs;                                  % sampling duration
L = 1024;                                     % length of the signal
t = (0:L-1)*Ts;                             % sampling times
NFFT = 2^nextpow2(L);                       % number of frequency samples
f = Fs/2*linspace(0,1,NFFT/2+1);            % frequency axis

%% part 1-alef
fo = 1000;                                  % signal frequency
x1 = sin(2*pi*fo*t);                        % signal
                                            % plot signal
f1=figure; ax1=axes('Parent',f1);xlim(ax1,[0 0.005]);hold(ax1,'on');stem(t,x1,'LineWidth',2);plot(t,x1,'LineWidth',2);
                            % compute and plot the FFT of the signal
X1 = fft(x1,NFFT)/L;
figure;plot(f,2*abs(X1(1:NFFT/2+1)));title('Single-Sided Amplitude Spectrum of x1(t)');xlabel('Frequency (Hz)');ylabel('|X1(f)|');


%% part 1-be
fo = 1800;                                  % signal frequency
x2 = sin(2*pi*fo*t);                        % signal
                                            % plot signal
f2=figure; ax2=axes('Parent',f2);xlim(ax2,[0 0.005]);hold(ax2,'on');stem(t,x2,'LineWidth',2);plot(t,x2,'LineWidth',2);
% compute and plot the FFT of the signal
X2 = fft(x2,NFFT)/L;
figure;plot(f,2*abs(X2(1:NFFT/2+1)));title('Single-Sided Amplitude Spectrum of x2(t)');xlabel('Frequency (Hz)');ylabel('|X2(f)|');

%% part 1-pe
fo = 4500;                                  % signal frequency
x3 = sin(2*pi*fo*t);                        % signal
                                            % plot signal
f3=figure; ax3=axes('Parent',f3);xlim(ax3,[0 0.005]);hold(ax3,'on');stem(t,x3,'LineWidth',2);plot(t,x3,'LineWidth',2);

 % compute and plot the FFT of the signal
X3 = fft(x3,NFFT)/L;
figure;plot(f,2*abs(X3(1:NFFT/2+1)));title('Single-Sided Amplitude Spectrum of x3(t)');xlabel('Frequency (Hz)');ylabel('|X3(f)|');


%% part1 1-te
xt = x1+x2+x3;
xto = xt;
ft=figure; axt=axes('Parent',ft);xlim(axt,[0 0.005]);hold(axt,'on');stem(t,xt,'LineWidth',2);plot(t,xt,'LineWidth',2);

 % compute and plot the FFT of the signal
Xt = fft(xt,NFFT)/L;
figure;plot(f,2*abs(Xt(1:NFFT/2+1)));title('Single-Sided Amplitude Spectrum of xt(t)');xlabel('Frequency (Hz)');ylabel('|Xt(f)|');



%% part 1-se
Fs =6000;                                  % sampling frequency
Ts = 1/Fs;                                  % sampling duration
L = 1024;                                     % length of the signal
t = (0:L-1)*Ts;                             % sampling times
NFFT = 2^nextpow2(L);                       % number of frequency samples
f = Fs/2*linspace(0,1,NFFT/2+1);            % frequency axis

fo = 1000;                                  % signal frequency
x1 = sin(2*pi*fo*t);                        % signal
                                            % plot signal
f1=figure; ax1=axes('Parent',f1);xlim(ax1,[0 0.005]);hold(ax1,'on');stem(t,x1,'LineWidth',2);plot(t,x1,'LineWidth',2);
                            % compute and plot the FFT of the signal
X1 = fft(x1,NFFT)/L;
figure;plot(f,2*abs(X1(1:NFFT/2+1)));title('Single-Sided Amplitude Spectrum of x1(t)');xlabel('Frequency (Hz)');ylabel('|X1(f)|');



fo = 1800;                                  % signal frequency
x2 = sin(2*pi*fo*t);                        % signal
                                            % plot signal
f2=figure; ax2=axes('Parent',f2);xlim(ax2,[0 0.005]);hold(ax2,'on');stem(t,x2,'LineWidth',2);plot(t,x2,'LineWidth',2);
% compute and plot the FFT of the signal
X2 = fft(x2,NFFT)/L;
figure;plot(f,2*abs(X2(1:NFFT/2+1)));title('Single-Sided Amplitude Spectrum of x2(t)');xlabel('Frequency (Hz)');ylabel('|X2(f)|');

fo = 4500;                                  % signal frequency
x3 = sin(2*pi*fo*t);                        % signal
                                            % plot signal
f3=figure; ax3=axes('Parent',f3);xlim(ax3,[0 0.005]);hold(ax3,'on');stem(t,x3,'LineWidth',2);plot(t,x3,'LineWidth',2);

 % compute and plot the FFT of the signal
X3 = fft(x3,NFFT)/L;
figure;plot(f,2*abs(X3(1:NFFT/2+1)));title('Single-Sided Amplitude Spectrum of x3(t)');xlabel('Frequency (Hz)');ylabel('|X3(f)|');



xt = x1+x2+x3;
ft=figure; axt=axes('Parent',ft);xlim(axt,[0 0.005]);hold(axt,'on');stem(t,xt,'LineWidth',2);plot(t,xt,'LineWidth',2);

 % compute and plot the FFT of the signal
Xt = fft(xt,NFFT)/L;
figure;plot(f,2*abs(Xt(1:NFFT/2+1)));title('Single-Sided Amplitude Spectrum of xt(t)');xlabel('Frequency (Hz)');ylabel('|Xt(f)|');


%% part 1-jim
Fs =12000;                                  % sampling frequency
Ts = 1/Fs;                                  % sampling duration
L = 1024;                                     % length of the signal
t = (0:L-1)*Ts;                             % sampling times
NFFT = 2^nextpow2(L);                       % number of frequency samples
f = Fs/2*linspace(0,1,NFFT/2+1);            % frequency axis
XT = fft(xto);
XT(0.25*L:0.75*L)=0;
xt = real(ifft(XT));
ft=figure; axt=axes('Parent',ft);xlim(axt,[0 0.005]);hold(axt,'on');stem(t,xt,'LineWidth',2);plot(t,xt,'LineWidth',2);

 % compute and plot the FFT of the signal
Xt = fft(xt,NFFT)/L;
figure;plot(f,2*abs(Xt(1:NFFT/2+1)));title('Single-Sided Amplitude Spectrum of xt(t)');xlabel('Frequency (Hz)');ylabel('|Xt(f)|');

%% part 1-che
Fs =6000;                                  % sampling frequency
Ts = 1/Fs;                                  % sampling duration
L = 1024/2;                                     % length of the signal
t = (0:L-1)*Ts;                             % sampling times
NFFT = 2^nextpow2(L);                       % number of frequency samples
f = Fs/2*linspace(0,1,NFFT/2+1);            % frequency axis
xt = downsample(xt,2);
ft=figure; axt=axes('Parent',ft);xlim(axt,[0 0.005]);hold(axt,'on');stem(t,xt,'LineWidth',2);plot(t,xt,'LineWidth',2);

 % compute and plot the FFT of the signal
Xt = fft(xt,NFFT)/L;
figure;plot(f,2*abs(Xt(1:NFFT/2+1)));title('Single-Sided Amplitude Spectrum of xt(t)');xlabel('Frequency (Hz)');ylabel('|Xt(f)|');
