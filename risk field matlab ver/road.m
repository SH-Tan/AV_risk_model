% 边界
r = 1/(exp(30));
% W = 0:0.001:16;
W=linspace(0,40,100);
x=linspace(0,480,100);
[ww,xx]=meshgrid(W,x);

Yl = 25;
Yr = 15;
Yd = 20;
k = 0.5;
Eb = r*(exp(abs(ww-Yl)/k)-1) + r*(exp(abs(ww-Yr)/k)-1);

% 车道�?
a = 2;
b = 30*pi/180; % 航向�?
y = 2; % 假设自车y坐标
Ay = exp(7);
Aw = exp(5);
Ely = Ay*exp(-(ww-Yd).^2/(2*a^2));
Elw = Aw*exp(-(ww-Yl).^2/(2*a^2))+Aw*exp(-(ww-Yr).^2/(2*a^2));
a = (ww-Yl).^2/(2*a^2);
E = Eb;


mesh(ww,xx,Eb);
grid on;
