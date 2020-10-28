function part2
filename = 'JohnLennon-Imagine.mp3';
[x,f]=audioread(filename);
x = mean(x')';
soundsc(x,f);
HL = fdatooll(1000);
yl = filter(HL,x);
soundsc(y1,f);
audiowrite('outoutLowPass.wav',yl,f);

HH = fdatoolh(1000);
yh = filter(HH,x);
soundsc(yh,f);
audiowrite('outoutHighPass.wav',yh,f);

