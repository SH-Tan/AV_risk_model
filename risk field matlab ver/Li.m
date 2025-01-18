clear
clc

%load('lab_test.mat')
%trajectories = testdata;
load('./data/car1259.mat')
load('./data/car1248.mat')
trajectories = cat(1, car1259, car1248);
% 
% load('change.mat')
% trajectories = data;
% % 
%load('d30.mat')
%trajectories = data;

% Section limits can be adjusted, here it is 200-800 feets (600 feets long)
% Section limits can be adjusted, here it is 200-800 feets (600 feets long)
%sectionLimits = [180 280];
sectionLimits = [215 240];
%sectionLimits = [120 145];

% Limits of x-y axis
xLimit = [sectionLimits(1) sectionLimits(2)];
yLimit = [0 10];

col = xLimit(1):1:xLimit(2);
row = yLimit(1):1:yLimit(2);
map = zeros(length(row),length(col)+xLimit(1));
size(map);
Frames = unique(trajectories(:,2));
figure(6)
legend_id = [];

for i=27:27
    legend_id(i-2,:) = num2str(i);
    
    frameData = trajectories(trajectories(:,2)==Frames(i) & ...
        trajectories(:,6)>=sectionLimits(1) & ...
        trajectories(:,6)<=sectionLimits(2),:);
    
    if isempty(frameData)
       continue; 
    end
    
    % Get needed fields
    lateralPos = frameData(:,5);
    longitudePos = frameData(:,6);
    id_all = frameData(:,1);
    len = frameData(:,9);
    width = frameData(:,10);
    class = frameData(:,11);
    id1 = num2str(frameData(:,1));
    
    % Construct vehicle bounding boxes
    boundingBoxArr = [longitudePos-len lateralPos-width/2 len width];
    
    ego_data = trajectories(trajectories(:,2)==Frames(i) & trajectories(:,1) == 1259,:);
    obs_data = trajectories(trajectories(:,2)==Frames(i) & trajectories(:,1) == 1248,:);
%     
    %ego_data = frameData(frameData(:,1) == 2484,:);
    %obs_data = frameData(frameData(:,1) ~= 2484,:);

    % ego_data = trajectories(trajectories(:,2)==Frames(i) & trajectories(:,1) == 1274,:);
    % obs_data = trajectories(trajectories(:,2)==Frames(i) & trajectories(:,1) == 1267,:);
    
%    ego_data = trajectories(trajectories(:,2)==Frames(i) & trajectories(:,1) == 2484,:);
%     obs_data = frameData(frameData(:,1) == 2505,:);
    % y x l w v a lane_id
    ego = [ego_data(:,6)-ego_data(:,9)/2 ego_data(:,5) ego_data(:,9) ego_data(:,10) ego_data(:,12) ego_data(:,13) ego_data(:,14) ego_data(:,19)];
    obs = [obs_data(:,6)-obs_data(:,9)/2 obs_data(:,5) obs_data(:,9) obs_data(:,10) obs_data(:,12) obs_data(:,13) obs_data(:,14) obs_data(:,1) obs_data(:,19)];
    
    % fun = @(x)fun_carrisk(ego,obs);
    max_E = 0.0;
    max_E_2 = 0.0;
    l_obs = size(obs,1);
    ego_lane = ego(7);  % lane id
    for i1=1:l_obs
        
        tmp = zeros(length(row),length(col)+xLimit(1));
        tmp_cdf = zeros(length(row),length(col)+xLimit(1));
        o = obs(i1,:);
        id = o(8);
        %legend_id(i1,:) = num2str(id);
        obs_a = o(6);       
        ego_v = ego(5);       
        o_lane = o(7);
        yaw = o(9);
        for i2 = 1 : length(row)
            val = 0.0;
             for j = 1 : length(col)+xLimit(1)
%                 j_c = cos(yaw)*j+sin(yaw)*i2;
%                 i2_c = -sin(yaw)*j+cos(yaw)*i2;
                k = sqrt(((j-o(1))*2.727/exp(0.029*o(5)))^2 + ((i2-o(2))*2.727)^2);
                l1 = i2-o(2); %y
                l2 = j-o(1); %x
                val = 0.056*exp(0.169*obs_a*(l2/sqrt(l1^2+l2^2)))/k;
                tmp(i2,j) = tmp(i2,j)+real(val);
                if tmp(i2,j) > max_E
                    max_E = tmp(i2,j);
                end
                if tmp(i2,j) < max_E
                    if tmp(i2,j) > max_E_2
                        max_E_2 = tmp(i2,j);
                    end
                end
            end 
        end
        % update map

        for i4 = 1 : length(row)
            for j = 1 : length(col)+xLimit(1)
                if abs(max_E - max_E_2) > 0.45
                   if tmp(i4,j) == max_E
                        tmp_cdf(i4,j) = (tmp(i4,j)/max_E);
                        tmp(i4,j) = 255*(tmp(i4,j)/max_E);
                    else
                        tmp_cdf(i4,j) = (tmp(i4,j)/max_E_2);
                        tmp(i4,j) = 255*(tmp(i4,j)/max_E_2);
                    end
                else
                    tmp_cdf(i4,j) = (tmp(i4,j)/max_E);
                    tmp(i4,j) = 255*(tmp(i4,j)/max_E);
                end
            end
        end
        map = map+tmp;
        map_cdf = tmp_cdf;
        max_E = 0.0;
        max_E_2 = 0.0;
        
        %CCDF
        rl = min(floor(ego(1)-ego(3)), floor(o(1)-o(3)-10));
        rr = max(ceil(ego(1)+ego(3)), ceil(o(1)+o(3)+10));
        cu = min(floor(ego(2)-ego(4)), floor(o(2)-o(4)-10));
        cl = max(ceil(ego(2)+ego(4)), ceil(o(2)+o(4)+10));
        rl = max(xLimit(1),rl);
        rr = min(xLimit(2),rr);
        cl = min(yLimit(2),cl);
        cu = max(1,cu);
        t_cdf = map_cdf([cu:cl],[rl:rr]);
        %t_cdf = map_cdf;
        map1 = t_cdf';
        map1 = map1(1:end);
        [f,xx] = ecdf(map1);
        f = 1-f;     
        %ccdf_x(i) = xx;
        p = plot(xx,f, 'LineWidth', 1.5);
        g = get(p,'Parent');%对应p1所在的坐标轴
        set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
        hold on
        %legend
    end
%     xlabel('a');
%     ylabel('CCDF');
%     char(legend_id)
%     lgd = legend(char(legend_id));
%     set(lgd,'FontSize',12);
    %boundingBoxArr1 = [ego(1)-ego(3)/2-xLimit(1) ego(2)-ego(4)/2 ego(3) ego(4)];
    
     t = strcat('Frames', num2str(i));
%     
%     %f = figure('name',t, 'color', 'w');
     figure(3)
% %     line(xLimit,[-5 -5],'Color','blue','LineStyle','--')
% %     line(xLimit,[20 20],'Color','blue','LineStyle','--')
%     %boundingBoxArr1 = [ego(1)-ego(3)/2-xLimit(1) ego(2)-ego(4)/2 ego(3) ego(4)];
%     %rectangle('Position', boundingBoxArr1(1,:), 'FaceColor', [1 1 0])
%     %hold on
%     % image(map(:,[xLimit(1):xLimit(2)]))
     [c,h] = contour(col, row, map(:,[xLimit(1):xLimit(2)]), 'LineWidth', 1.5);
     g = get(h,'Parent');%对应p1所在的坐标轴
     set(g,'Linewidth',1.5,'FontSize',12,'FontName','Arial','FontWeight','bold');
     ylabel('Width [m]','FontSize',12,'FontName','Arial','FontWeight','bold');
     xlabel('Length [m]','FontSize',12,'FontName','Arial','FontWeight','bold');
     title(t, 'FontSize',13,'FontName','Arial','FontWeight','bold')
%     % contour(row1,col1, map_cdf)
%     % pcolor(col,row,map);
%     % surf(x,y,z); view(0,90); %等效的写法
%     % shading interp; 
     colorbar; colormap(jet);
%     % xlabel('X');ylabel('Y');
%     % set(gca,'FontSize',16)
     set(gca,'Ydir','reverse')
     grid on
     n = strcat(t,'.jpg');
%     saveas(f, strcat('./res/Li/',n));
%     

%figure(1)
%     line(xLimit,[2 2],'Color','blue','LineStyle','--')
%     line(xLimit,[13 13],'Color','blue','LineStyle','--')
    %boundingBoxArr1 = [ego(1)-ego(3)/2-xLimit(1) ego(2)-ego(4)/2 ego(3) ego(4)];
   
%     subplot(1,2,1)
    %rectangle('Position', boundingBoxArr1(1,:), 'FaceColor', [1 1 0])
    %hold on
    % image(map(:,[xLimit(1):xLimit(2)]))
    %[c,h]= contour(col, row, map(:,[xLimit(1):xLimit(2)]),'LineWidth', 1.5);
    %h= mesh(col, row, map(:,[xLimit(1):xLimit(2)]))
    %g = get(h,'Parent');%对应p1所在的坐标轴
    %set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
    %xlabel('Length [m]')
    %ylabel('Width [m]')
%     col1 = cu:1:cl;
%     row1 = rl:1:rr;
    % contour(row1,col1, map_cdf)
    % pcolor(col,row,map);
    % surf(x,y,z); view(0,90); %等效的写法
    % shading interp; 
    %colorbar; colormap(jet);
    % xlabel('X');ylabel('Y');
    % set(gca,'FontSize',16)
    %set(gca,'Ydir','reverse')
    %grid on
     %figure(1)
     %subplot(1,2,1)
     %size(map);
%     %rectangle('Position', boundingBoxArr1(1,:), 'FaceColor', [1 1 0])
%     %hold on
%     %image(map(:,[xLimit(1):xLimit(2)]))
     %contour(col, row, map(:,[xLimit(1):xLimit(2)]), 'LineWidth', 1.5)
%     %contour(col, row, map(:,[xLimit(1):xLimit(2)]))
%     % pcolor(col,row,map);
%     % surf(x,y,z); view(0,90); %等效的写法
%     % shading interp; 
%     colorbar; colormap(jet);
%     % xlabel('X');ylabel('Y');
%     % set(gca,'FontSize',16)
%     set(gca,'Ydir','reverse')
%     grid on
%     
%     subplot(1,2,2)
%     for j=1:length(boundingBoxArr(:,1))
%         hold on
%         if(id_all(j) == 2484)
%             rectangle('Position', boundingBoxArr(j,:), 'FaceColor', [1 0 0])
%         elseif(id_all(j) == 2476)
%             rectangle('Position', boundingBoxArr(j,:), 'FaceColor', [1 1 0])
%         end
%     end
%     xlim(xLimit)
%     ylim(yLimit)
%     xlabel('Longitude (feet)')
%     ylabel('Lateral (feet)')
%     set(gca,'Ydir','reverse')
%     grid on
%     % rectangle('Position', boundingBoxArr(1,:), 'FaceColor', [1 0 0])
%     figure(1)
%     line(xLimit,[2 2],'Color','blue','LineStyle','--')
%     line(xLimit,[18 18],'Color','blue','LineStyle','--')
%     %boundingBoxArr1 = [ego(1)-ego(3)/2-xLimit(1) ego(2)-ego(4)/2 ego(3) ego(4)];
%    
%     %subplot(1,2,1)
%     %rectangle('Position', boundingBoxArr1(1,:), 'FaceColor', [1 1 0])
%     %hold on
%     % image(map(:,[xLimit(1):xLimit(2)]))
%     [c,h] = contour(col, row, map(:,[xLimit(1):xLimit(2)]));
%     g = get(h,'Parent');%对应p1所在的坐标轴
%     set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
%     xlabel('Length [m]')
%     ylabel('Width [m]')
% %     col1 = cu:1:cl;
% %     row1 = rl:1:rr;
%     % contour(row1,col1, map_cdf)
%     % pcolor(col,row,map);
%     % surf(x,y,z); view(0,90); %等效的写法
%     % shading interp; 
%     colorbar; colormap(jet);
%     % xlabel('X');ylabel('Y');
%     % set(gca,'FontSize',16)
%     set(gca,'Ydir','reverse')
%     grid on
    
%     subplot(1,2,2)
%     for j=1:length(boundingBoxArr(:,1))
%         hold on
%         if(id_all(j) == 2484)
%             p1 = rectangle('Position', boundingBoxArr(j,:), 'FaceColor', [1 0 0]);
%         else
%             p11 = rectangle('Position', boundingBoxArr(j,:), 'FaceColor', [1 1 0]);
%         end
%     end
%     g = get(p11,'Parent');%对应p1所在的坐标轴
%     set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
%     text(longitudePos-2*len/3,lateralPos,id1, 'color', 'b', 'Margin', 0.1, 'FontSize',8, 'clipping','on')
%     
%     h = zeros(3, 1);
%     h(1) = plot(NaN,NaN,'sr', 'MarkerFaceColor',[1,0,0]);
%     h(2) = plot(NaN,NaN,'sy', 'MarkerFaceColor',[1,1,0]);
%     h(3) = plot(NaN,NaN,'sg', 'MarkerFaceColor',[0,1,0]);
%     lgd = legend(h, 'ego','nearby car', 'Location','northeast');
%     set(lgd,'FontSize',8)
%     
%     xlim(xLimit)
%     ylim(yLimit)
%     xlabel('Longitude (m)')
%     ylabel('Lateral (m)')
%     set(gca,'Ydir','reverse')
%     grid on

    %pause(0.1)
    %clf('reset')
end

%xlabel('a');
%ylabel('CCDF');
%char(legend_id);
%lgd = legend(char(legend_id));
%set(lgd,'FontSize',12);