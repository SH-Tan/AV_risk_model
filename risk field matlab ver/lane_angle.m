clear
clc


% W = 0:0.001:16;
map = zeros(100,14);
angle = 30/180*pi;
Yl = 14-10.5*cos(angle)*cos(angle);
Yr = 14-3.5*cos(angle)*cos(angle);
Yd = 7;
k = 0.4;
%b = ww-tan(angle);
%Eb_2 = r*(exp(abs(W-Yl)/k)-1);
%Eb = r*(exp(abs(ww-Yl)/k)-1);
%Eb_both = r*(exp(abs(ww-Yl)/k)-1)+r*(exp(abs(ww-Yr)/k)-1);
r = 1/exp(Yl);
for i = 0:14
    b = i - tan(angle)*0;
    cur = 14-i*cos(angle)*cos(angle)
    tmp = (exp(abs(cur-Yl)/k)-1)*r + (exp(abs(cur-Yr)/k)-1)*r;
    for j = 1 : 100*cos(angle)
        val = max(0.0,tmp);
        map(j,ceil(j*tan(angle)+b)) = val;
    end
end

figure(1)
set(gcf,'unit','centimeters','position',[10 5 20 12]); % 10cm*17.4cm
set(gcf,'ToolBar','none','ReSize','off');   % 移除工具栏
set(gcf,'color','w'); % 背景设为白色

p2 = mesh(map)

g = get(p2,'Parent');%对应p1所在的坐标轴
set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
%set(g,'ZTick',[]);
zlabel('Risk strength','FontSize',10,'FontName','Arial','FontWeight','bold');
xlabel({'Lane width','(b)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
%contour(map)