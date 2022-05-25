clear
clc

% Section limits can be adjusted, here it is 200-800 feets (600 feets long)
sectionLimits = [1 25];

% Limits of x-y axis
xLimit = [sectionLimits(1) sectionLimits(2)];
yLimit = [0 14];

col = xLimit(1):1:xLimit(2);
row = yLimit(1):1:yLimit(2);
map = zeros(length(row),length(col));
map_cdf = zeros(length(row),length(col));
size(map);

for i=25:25
    
    % Construct vehicle bounding boxes
    % boundingBoxArr = [longitudePos-len lateralPos-width/2 len width];
    
    % y x l w v a lane_id
    ego = [10 8.5 3.5 1.5 20 1.5 3 0];
    obs1 = [15 8.5 3.5 1.5 25 0.3 3 2 0];
    obs2 = [5 8.5 3.5 1.5 25 0 3 3 0];
    obs3 = [8 5.5 3.5 1.5 18 0.1 2 1 -10*pi/180];
    obs = [obs3;obs1;obs2];
    boundingBoxArr = [obs(:,1)-obs(:,3)/2 obs(:,2)-obs(:,4)/2 obs(:,3) obs(:,4);ego(:,1)-ego(:,3)/2 ego(:,2)-ego(:,4)/2 ego(:,3) ego(:,4)];
    % fun = @(x)fun_carrisk(ego,obs);
    max_E = 0.0;
    l_obs = size(obs,1);
    %l_obs = 2;
    ego_lane = ego(7);  % lane id
    legend_id = [];
    figure(3)
    for i1=1:l_obs
        tmp = zeros(length(row),length(col));
        tmp_cdf = zeros(length(row),length(col));
        o = obs(i1,:);
        id = o(8);
        legend_id(i1,:) = ['car',num2str(id)];
        re_loca = [ego(1)-o(1) ego(2)-o(2)];
        v_re = o(5)-ego(5);
        r = o(9); % 没有数据
        sin(r);
        yaw_sign = 1;
        obs_a = o(6);
        a_r = normrnd(obs_a, 0.15, [10,1]);
        yaw_vec = [1 0];
        simi = dot(re_loca,yaw_vec)/(norm(re_loca)*norm(yaw_vec));
        if simi >= 0
            simi_sign = 1;
        else
            simi_sign = -1;
        end
        
        k_d = 1.0;
        sign = 0;
        ego_v = ego(5);
        if ego_v <= abs(v_re)
            ego_v = abs(v_re);
        end
        o_lane = o(7);
        
        if o_lane == ego_lane
            sign = 1; % follow
            v_r = v_re*simi_sign;
            
            if sign == 0 || v_r < 0
                ratio = abs(v_re)/ego_v;
                k_d = (1+exp(-ratio))*(o(4)/o(3));
            else
                k_d = 1 + log(1+v_r);
            end
        else
            d_sign = 0;
            if re_loca(1) < 0
                d_sign = 1;
            end
            sign = ~xor(d_sign, yaw_sign);
            if sign == 0
                ratio = abs(v_re)/ego_v;
                k_d = (1+exp(-ratio))*(o(4)/o(3));
            else
                k_d = 1 + log(1+v_re);
            end
        end
        
        % build field
        a_n = length(a_r);
        delta_v = zeros([a_n, 1]);
        max_de = 0.0;
        for i2 = 1 : a_n
            delta_v(i2) = (o(5)*0.1+0.5*a_r(i2)*0.1*0.1);
            max_de = max(max_de,delta_v(i2));
        end
        
        val = 0.0;
        
        kl_1 = o(3)*k_d;
        kl_2 = o(4);
        for i3 = 1 : length(row)
            for j = 1 : length(col)
                x = cos(r)*(j-o(1)) - sin(r)*(i3-o(2)) + o(1);
                y = sin(r)*(j-o(1)) + cos(r)*(i3-o(2)) + o(2);
                for k = 1 : a_n
                    x_d1 = ((o(1)-x)/kl_1)^2;
                    x_d2 = ((o(2)-y)/kl_2)^2;
                    k_first = sqrt(x_d1+x_d2);
                    if k_first > delta_v(k)
                        k_first = exp(k_first);
                    end
                    val = (val + exp(a_r(k)*simi)/k_first)/a_n;
                end
                tmp(i3,j) = tmp(i3,j)+real(val);
                if tmp(i3,j) > max_E
                    max_E = tmp(i3,j);
                end
                val = 0.0;
            end
        end
        % update map
        for i4 = 1 : length(row)
            for j = 1 : length(col)
                tmp_cdf(i4,j) = (tmp(i4,j)/max_E);
                tmp(i4,j) = 255*(tmp(i4,j)/max_E);
            end
        end
        map = map + tmp;
        map_cdf = tmp_cdf;
        max_E = 0.0;
        
        %CCDF
        rl = min(floor(ego(1)-ego(3)), floor(o(1)-o(3)-max_de-3));
        rr = max(ceil(ego(1)+ego(3)), ceil(o(1)+o(3)+max_de+3));
        cu = min(floor(ego(2)-ego(4)), floor(o(2)-o(4)-max_de-3));
        cl = max(ceil(ego(2)+ego(4)), ceil(o(2)+o(4)+max_de+3));
        rl = max(1,rl);
        rr = min(25,rr);
        cl = min(15,cl);
        cu = max(1,cu);
        map_cdf = map_cdf([cu:cl],[rl:rr]);
        map1 = map_cdf';
        map1 = map1(1:end);
        [f,xx] = ecdf(map1);
        % ecdf(map1)
        % cdfplot(map1);
        f = 1-f;
        p = plot(xx,f,'LineWidth',2);
        hold on
    end
    ylabel('CCDF','FontSize',10,'FontName','Arial','FontWeight','bold');
    xlabel({'a','(a)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
    char(legend_id);
    legend(char(legend_id));
    set(gca,'FontSize',16)
    g = get(p,'Parent');%对应p1所在的坐标轴
    set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
 
%     figure(2)
%     set(gca,'Ydir','reverse')
%     line(xLimit,[-5 -5],'Color','blue','LineStyle','--')
%     line(xLimit,[20 20],'Color','blue','LineStyle','--')
%     %boundingBoxArr1 = [ego(1)-ego(3)/2-xLimit(1) ego(2)-ego(4)/2 ego(3) ego(4)];
%    
%     %subplot(1,2,1)
%     %rectangle('Position', boundingBoxArr1(1,:), 'FaceColor', [1 1 0])
%     %hold on
%     % image(map(:,[xLimit(1):xLimit(2)]))
%     %contour(col, row, map)
%     %mesh(map)
%     col1 = cu:1:cl;
%     row1 = rl:1:rr;
%     % contour(row1,col1, map_cdf)
%     % pcolor(col,row,map);
%     % surf(x,y,z); view(0,90); %等效的写法
%     % shading interp; 
%     colorbar; colormap(jet);
%     % xlabel('X');ylabel('Y');
%     % set(gca,'FontSize',16)
%     %set(gca,'Ydir','reverse')
%     %grid on
%     hold on
    
%     for j=1:length(boundingBoxArr(:,1))
%         hold on
%         if(j == length(boundingBoxArr(:,1)))
%             rectangle('Position', boundingBoxArr(j,:), 'FaceColor', [1 0 0])
%         else
%             rectangle('Position', boundingBoxArr(j,:), 'FaceColor', [1 1 0])
%         end
%     end
    
    figure(1)
    set(gcf,'unit','centimeters','position',[10 5 22 12]); % 10cm*17.4cm
    set(gcf,'ToolBar','none','ReSize','off');   % 移除工具栏
    set(gcf,'color','w'); % 背景设为白色

    subplot(2,2,1) 

    [c,p1] = contour(map);
    set(gca,'Position',[0.08 0.25 0.42 0.55]);%第(1)个图的位置
    g = get(p1,'Parent');%对应p1所在的坐标轴
    set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
    ylabel('Risk distribution','FontSize',10,'FontName','Arial','FontWeight','bold');
    xlabel({'Lane width','(a)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
    colorbar; colormap(jet);

    subplot(2,2,2) 
    p2 = mesh(map);
    set(gca,'Position',[0.56 0.25 0.42 0.55]);%第(2)个图的位置
    g = get(p2,'Parent');%对应p1所在的坐标轴
    set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
    %set(g,'ZTick',[]);
    %zlabel('Risk strength','FontSize',10,'FontName','Arial','FontWeight','bold');
    xlabel({'Lane width','(b)'},'FontSize',10,'FontName','Arial','FontWeight','bold');
    colorbar; colormap(jet);

end