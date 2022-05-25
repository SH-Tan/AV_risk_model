clear all
clc

load('./data/car1259.mat')
load('./data/car1248.mat')
trajectories = cat(1, car1259, car1248);

% load('change.mat')
% trajectories = data;

car1 = trajectories(trajectories(:,1) == 1259,:);
car2 = trajectories(trajectories(:,1) == 1248,:);
car3 = trajectories(trajectories(:,1) == 1274,:);

v1 = car1(:,12);
a1 = car1(:,13);
v2 = car2(:,12);
a2 = car2(:,13);
a = a2-a1;
v3 = car3(:,12);
a3 = car3(:,13);
THW1 = car1(:,18);
THW2 = car3(:,18);
diff_v = v2-v1;
% yaw1 = car1(:,19);
% yaw2 = car2(:,19);
% yaw3 = car3(:,19);
% y = yaw2-yaw3;

x = 23:1:28;

figure(1)
set(gcf,'unit','centimeters','position',[10 5 17.4 10]); % 10cm*17.4cm
set(gcf,'ToolBar','none','ReSize','off');   % 移除工具栏
set(gcf,'color','w'); % 背景设为白色

subplot(2,2,1) 
p2 = plot(x, THW1(23:28), 'Linewidth',1.5);
set(gca,'Position',[0.08 0.63 0.38 0.3]);%第(1)个图的位置
ylabel('THW','FontSize',10,'FontName','Arial','FontWeight','bold');
xlabel({'frames','(a)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
set(gca,'FontSize',16)
g = get(p2,'Parent');%对应p1所在的坐标轴
set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
legend('THW')


%figure(2)
subplot(2,2,2) 
p3 = plot(x,a1(23:28), 'Linewidth',1.5);
hold on
p33 = plot(x,a2(23:28), 'Linewidth',1.5);
hold on
p333 = plot(x,a(23:28),'Linewidth',1.5);

set(gca,'Position',[0.56 0.63 0.38 0.3]);%第(2)个图的位置
ylabel('acc','FontSize',10,'FontName','Arial','FontWeight','bold');
xlabel({'frames','(b)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
set(gca,'FontSize',16)
g = get(p3,'Parent');%对应p1所在的坐标轴
set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
legend('ego','obs', 'diff')


% 
%figure(3)
subplot(2,2,3) 
p1 = plot(x,v1(23:28),'Linewidth',1.5);
hold on
p11 = plot(x,v2(23:28), 'Linewidth',1.5);
set(gca,'Position',[0.08 0.15 0.38 0.3]);%第(3)个图的位置

ylabel('V [m/s]','FontSize',10,'FontName','Arial','FontWeight','bold');
xlabel({'frames','(c)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
set(gca,'FontSize',16)
g = get(p1,'Parent');%对应p1所在的坐标轴
set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
legend('ego','obs')



%figure(4)
subplot(2,2,4) 
y = zeros(size(diff_v));
hold on
p4 = plot(x, diff_v(23:28), 'Linewidth',1.5);
hold on
p44 = plot(x,y(23:28), '--', 'Linewidth',1.5);
set(gca,'Position',[0.56 0.15 0.38 0.3]);%第(4)个图的位置
ylabel('V_DIFF [m/s]','FontSize',10,'FontName','Arial','FontWeight','bold');
xlabel({'frames','(d)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
set(gca,'FontSize',16)
g = get(p4,'Parent');%对应p1所在的坐标轴
set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
legend('V_DIFF', 'baseline = 0')


% figure(1)
% set(gcf,'unit','centimeters','position',[10 5 17.4 10]); % 10cm*17.4cm
% set(gcf,'ToolBar','none','ReSize','off');   % 移除工具栏
% set(gcf,'color','w'); % 背景设为白色
% 
% subplot(2,2,1) 
% 
% p2 = plot(x, THW1(3:8), 'Linewidth',1.5);
% hold on
% p22 = plot(x, THW2(3:8), 'Linewidth',1.5);
% % p6 = plot(x,yaw1(3:8),'Linewidth',1.5);
% % hold on
% % p66 = plot(x,yaw2(3:8), 'Linewidth',1.5);
% % hold on
% ylabel('THW','FontSize',10,'FontName','Arial','FontWeight','bold');
% xlabel({'frames','(a)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
% set(gca,'FontSize',16)
% g = get(p2,'Parent');%对应p1所在的坐标轴
% set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
% legend('1267to1259', '1274to1267')
% set(gca,'Position',[0.08 0.63 0.38 0.3]);%第(1)个图的位置
% 
% subplot(2,2,2) 
%  
% % p6 = plot(x,yaw1(3:8),'Linewidth',1.5);
% % hold on
% p66 = plot(x,yaw2(3:8), 'Linewidth',1.5);
% hold on
% p666 = plot(x,y(3:8), 'Linewidth',1.5);
% set(gca,'Position',[0.56 0.63 0.38 0.3]);%第(2)个图的位置
% ylabel('yaw [°]','FontSize',10,'FontName','Arial','FontWeight','bold');
% xlabel({'frames','(b)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
% set(gca,'FontSize',16)
% g = get(p666,'Parent');%对应p1所在的坐标轴
% set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
% legend('cut-in car', 'diff yaw')
% 
% 
% subplot(2,2,3) 
% 
% p5 = plot(x,v1(3:8),'Linewidth',1.5);
% hold on
% p55 = plot(x,v2(3:8), 'Linewidth',1.5);
% hold on
% p555 = plot(x,v3(3:8), 'Linewidth',1.5);
% set(gca,'Position',[0.08 0.15 0.38 0.3]);%第(3)个图的位置
% ylabel('V [m/s]','FontSize',10,'FontName','Arial','FontWeight','bold');
% xlabel({'frames','(c)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
% set(gca,'FontSize',16)
% g = get(p5,'Parent');%对应p1所在的坐标轴
% set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
% legend('car_{front}','cut-in car', 'car_{back}')
% 
% subplot(2,2,4) 
%  
% p3 = plot(x,a1(3:8), 'Linewidth',1.5);
% hold on
% p33 = plot(x,a2(3:8), 'Linewidth',1.5);
% hold on
% p333 = plot(x,a3(3:8),'Linewidth',1.5);
% ylabel('acc','FontSize',10,'FontName','Arial','FontWeight','bold');
% xlabel({'frames','(b)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
% set(gca,'FontSize',16)
% g = get(p3,'Parent');%对应p1所在的坐标轴
% set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
% legend('car_{front}','cut-in car', 'car_{back}')
% set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
% set(gca,'Position',[0.56 0.15 0.38 0.3]);%第(4)个图的位置




