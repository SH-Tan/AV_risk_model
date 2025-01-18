clear all

% 杈圭晫
r = 1/exp(14);
% W = 0:0.001:16;
W=linspace(0,14,100);
x=linspace(0,20,100);
[ww,xx]=meshgrid(W,x);
Yl = 10.5;
Yr = 3.5;
Yd = 7;
k = 0.4;
Eb_2 = r*(exp(abs(W-Yl)/k)-1);
Eb = r*(exp(abs(ww-Yl)/k)-1);
Eb_both = r*(exp(abs(ww-Yl)/k)-1)+r*(exp(abs(ww-Yr)/k)-1);

% 杞﹂亾绾?
a = 2;
b = 30*pi/180; % 鑸悜瑙?
y = 2; % 鍋囪鑷溅y鍧愭爣
Ay = exp(11.5);
Aw = exp(10);
Ely = Ay*exp(-(ww-Yd).^2/2*a^2)*cos(b);
Elw = Aw*exp(-(ww-Yl).^2/2*a^2)*cos(b)+Aw*exp(-(ww-Yr).^2/2*a^2)*cos(b);
Ely_2 = Ay*exp(-(W-Yd).^2/2*a^2)*cos(b);
Elw_2 = Aw*exp(-(W-Yl).^2/2*a^2)*cos(b)+Aw*exp(-(W-Yr).^2/2*a^2)*cos(b);
E_2 = Ely_2+Elw_2;
E = Ely+Elw;
Et = E+Eb_both;
%plot(W,Eb_2)
%mesh(ww,xx,Et);
%pause(30);
%grid on;

figure(1)
set(gcf,'unit','centimeters','position',[10 5 20 12]); % 10cm*17.4cm
set(gcf,'ToolBar','none','ReSize','off');   % 移除工具栏
set(gcf,'color','w'); % 背景设为白色
 
subplot(2,2,1) 
 
p1 = plot(W,E_2,'b-','LineWidth',1.5);
hold on
set(gca,'Position',[0.08 0.6 0.30 0.3]);%第(1)个图的位置
g = get(p1,'Parent');%对应p1所在的坐标轴
set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
legend([p1],'\fontname{Arial}E_{l}','Orientation','horizontal');
ylabel('Risk strength','FontSize',10,'FontName','Arial','FontWeight','bold');
xlabel({'Road width','(a)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
 
subplot(2,2,2) 
p2 = mesh(ww,xx,E);
set(gca,'Position',[0.56 0.55 0.40 0.43]);%第(2)个图的位置
g = get(p2,'Parent');%对应p1所在的坐标轴
set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
%set(g,'ZTick',[]);
%ylabel('Error [m]','FontSize',10,'FontName','Arial','FontWeight','bold');
xlabel({'Road width','(b)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
 
subplot(2,2,3) 
p3 = mesh(ww, xx, Et);
hold on
set(gca,'Position',[0.32 0.1 0.48 0.43]);%第(3)个图的位置
g = get(p3,'Parent');%对应p1所在的坐标轴
set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
zlabel('Risk strength','FontSize',10,'FontName','Arial','FontWeight','bold');
xlabel({'Road width','(c)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
%h3 = legend('\fontname{Arial}E_{total}');
%set(h3,'Linewidth',1.5,'FontSize',10,'FontWeight','bold');
%set(h3,'position',[0.1,0.1,0.1,0.1]);%legend位置
 
% set(h1,'Linewidth',1.5,'FontSize',10,'FontWeight','bold');
% set(h1,'position',[0.25,0.9,0.2,0.1]);%legend位置
% set(h1,'Box','off');

%exportgraphics(gcf,'peaks.png','Resolution',300);%输出分辨率为300的PNG图片
%exportgraphics(gcf,'peaks.pdf','ContentType','vector');%输出矢量pdf图片
%exportgraphics(gcf,'peaks.eps','Resolution',1200);%输出矢量eps图片
 
%text(0.05,0.6,'(a)')
