clear all

% 边界
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

% 车道�?
a = 2;
b = 30*pi/180; % 航向�?
y = 2; % 假设自车y坐标
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
set(gcf,'ToolBar','none','ReSize','off');   % �Ƴ�������
set(gcf,'color','w'); % ������Ϊ��ɫ
 
subplot(2,2,1) 
 
p1 = plot(W,E_2,'b-','LineWidth',1.5);
hold on
set(gca,'Position',[0.08 0.6 0.30 0.3]);%��(1)��ͼ��λ��
g = get(p1,'Parent');%��Ӧp1���ڵ�������
set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
legend([p1],'\fontname{Arial}E_{l}','Orientation','horizontal');
ylabel('Risk strength','FontSize',10,'FontName','Arial','FontWeight','bold');
xlabel({'Road width','(a)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
 
subplot(2,2,2) 
p2 = mesh(ww,xx,E);
set(gca,'Position',[0.56 0.55 0.40 0.43]);%��(2)��ͼ��λ��
g = get(p2,'Parent');%��Ӧp1���ڵ�������
set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
%set(g,'ZTick',[]);
%ylabel('Error [m]','FontSize',10,'FontName','Arial','FontWeight','bold');
xlabel({'Road width','(b)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
 
subplot(2,2,3) 
p3 = mesh(ww, xx, Et);
hold on
set(gca,'Position',[0.32 0.1 0.48 0.43]);%��(3)��ͼ��λ��
g = get(p3,'Parent');%��Ӧp1���ڵ�������
set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
zlabel('Risk strength','FontSize',10,'FontName','Arial','FontWeight','bold');
xlabel({'Road width','(c)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
%h3 = legend('\fontname{Arial}E_{total}');
%set(h3,'Linewidth',1.5,'FontSize',10,'FontWeight','bold');
%set(h3,'position',[0.1,0.1,0.1,0.1]);%legendλ��
 
% set(h1,'Linewidth',1.5,'FontSize',10,'FontWeight','bold');
% set(h1,'position',[0.25,0.9,0.2,0.1]);%legendλ��
% set(h1,'Box','off');

%exportgraphics(gcf,'peaks.png','Resolution',300);%����ֱ���Ϊ300��PNGͼƬ
%exportgraphics(gcf,'peaks.pdf','ContentType','vector');%���ʸ��pdfͼƬ
%exportgraphics(gcf,'peaks.eps','Resolution',1200);%���ʸ��epsͼƬ
 
%text(0.05,0.6,'(a)')
