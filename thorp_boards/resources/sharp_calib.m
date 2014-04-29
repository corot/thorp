R = load('~/.ros/READINGS');

s = size(R)
cov(R(:,1))
avg = 0;
for i = 1:s(2)
    cov(R(:,i))
    avg = avg + cov(R(:,i));
end
avg/s(2)

format long

% GP2D120, Kobuki's analog port
V = [  3000 2720 2330 2010 1780 1580 1400 1280 1080  920  800  720  620  510  410  380  310 ];
D = [ 0.038 0.04 0.05 0.06 0.07 0.08 0.09 0.10 0.12 0.14 0.16 0.18 0.20 0.25 0.30 0.35 0.40 ];

p = polyfit(V,D,6)
f = polyval(p,V);

figure
plot(V,D,'o',V,f,'-')

% GP2Y0A21YK, Kobuki's analog port
V = [  3140 3000 2300 1610 1310  930  740  610  510  450  410 ];
D = [  0.06 0.08 0.10 0.15 0.20 0.30 0.40 0.50 0.60 0.70 0.80 ];

p = polyfit(V,D,6)
f = polyval(p,V);

figure
plot(V,D,'o',V,f,'-')

% GP2Y0A21YK, Arduino's analog port
V = [   78  84  93 105 120 146 191 276 360  506  600  637 ];
D = [  0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.15 0.1 0.08 0.06 ];
%D = D.*100

%p = polyfit(V,D,6)
f = polyval(p,V);

figure
plot(V,D,'o',V,f,'-')
