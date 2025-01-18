clear all
clc

 load('./data/car1259.mat')
 load('./data/car1248.mat')
 trajectories = cat(1, car1259, car1248);

%load('change.mat')
%trajectories = data;

% Section limits can be adjusted, here it is 200-800 feets (600 feets long)
%sectionLimits = [120 145];
%sectionLimits = [150 270];
sectionLimits = [215 240];

% Limits of x-y axis
xLimit = [sectionLimits(1) sectionLimits(2)];
yLimit = [0 10];

col = xLimit(1):1:xLimit(2);
row = yLimit(1):1:yLimit(2);
map = zeros(length(row),length(col)+xLimit(1));
map_cdf = zeros(length(row),length(col)+xLimit(1));
size(map);
Frames = unique(trajectories(:,2));
ccdf = [];
ccdf_x = [];

% xlabel('x');
% ylabel('CCDF');
legend_id = [];
for i=23:23
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
    
    % Construct vehicle bounding boxes
    boundingBoxArr = [longitudePos-len lateralPos-width/2 len width];
    
     ego_data = trajectories(trajectories(:,2)==Frames(i) & trajectories(:,1) == 1259,:);
     obs_data = trajectories(trajectories(:,2)==Frames(i) & trajectories(:,1) == 1248,:);
%     obs_data = frameData(frameData(:,1) == 2476,:);

    %ego_data = trajectories(trajectories(:,2)==Frames(i) & trajectories(:,1) == 1274,:);
    %obs_data = trajectories(trajectories(:,2)==Frames(i) & trajectories(:,1) == 1267,:);

    % y x l w v a lane_id
    ego = [ego_data(:,6)-ego_data(:,9)/2 ego_data(:,5) ego_data(:,9) ego_data(:,10) ego_data(:,12) ego_data(:,13) ego_data(:,14) ego_data(:,19)];
    obs = [obs_data(:,6)-obs_data(:,9)/2 obs_data(:,5) obs_data(:,9) obs_data(:,10) obs_data(:,12) obs_data(:,13) obs_data(:,14) obs_data(:,1) obs_data(:,19)];
    
    % fun = @(x)fun_carrisk(ego,obs);
    max_E = 0.0;
    max_E_2 = 0.0;
    l_obs = size(obs,1);
    ego_lane = ego(7);  % lane id
    
    count = 0;
    for i1=1:l_obs
        tmp = zeros(length(row),length(col)+xLimit(1));
        tmp_cdf = zeros(length(row),length(col)+xLimit(1));
        o = obs(i1,:);
        id = o(8);
        re_loca = [ego(1)-o(1) ego(2)-o(2)];
        v_re = o(5)-ego(5);
        yaw_re = o(9)-ego(8); % 没有数据
        yaw = o(9);
        
        yaw_sign = 1;
        if (yaw_re < 0)
            yaw_sign = 0;
        end
        
        obs_a = o(6);
        a_r = normrnd(obs_a, 1.912, [10,1]);
        yaw_vec = [cos(yaw_re*pi/180) sin(yaw_re*pi/180)];
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
            if (abs(yaw_re) > 10)
                sign = 0;
            end
            
            v_r = v_re*simi_sign;
            yaw_re;
            
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
                k_d = 1 + log(1+abs(v_re));
            end
        end
        
        % build field
        a_n = length(a_r);
        delta_v = zeros([a_n, 1]);
        max_de = 0.0;
        for i2 = 1 : a_n
            delta_v(i2) = (o(5)*0.1+0.5*a_r(i2)*0.1*0.1);
            max_de = max(max_de,abs(delta_v(i2)));
        end
        val = 0.0;
        kl_1 = o(3)*real(k_d);
        kl_2 = o(4);
        for i3 = 1:length(row)
            for j = 1:length(col)+xLimit(1)
                i3_c = sin(yaw*pi/180)*(j-o(1))+cos(yaw*pi/180)*(i3-o(2))+o(2);
                j_c = cos(yaw*pi/180)*(j-o(1))-sin(yaw*pi/180)*(i3-o(2))+o(1);
                for k = 1 : a_n
                    x_d1 = ((o(1)-j_c)/kl_1)^2;
                    x_d2 = ((o(2)-i3_c)/kl_2)^2;
                    k_first = sqrt(x_d1+x_d2);
                    if k_first > abs(delta_v(k))
                        k_first = exp(k_first);
                    end
                    l1 = i3-o(2); %y
                    l2 = j-o(1); %x
                    val = (val + exp(-0.13*a_r(k)*simi*(l2/sqrt(l1^2+l2^2)))/(k_first))/a_n;
%                     if (val > 4)
%                         i3
%                         j
%                         count = count+1;
%                         k_first
%                         k
%                         a_r(k)
%                         val
%                     end
                end
                tmp(i3,j) = tmp(i3,j)+real(val);
                if tmp(i3,j) > max_E
                    max_E = tmp(i3,j);
                end
                if tmp(i3,j) < max_E
                    if tmp(i3,j) > max_E_2
                        max_E_2 = tmp(i3,j);
                    end
                end
                val = 0.0;
            end
         end
%         max_E
%         max_E_2
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
        map = map + tmp;
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
        %[f,xx] = ecdf(map1);
        %ecdf(map1)
        %cdfplot(map1);
        %f = 1-f;
       % p = plot(xx,f, 'LineWidth', 1.5);
        %g = get(p,'Parent');%对应p1所在的坐标轴
        %set(g,'Linewidth',1.5,'FontSize',10,'FontName','Arial','FontWeight','bold');
        %hold on
    end
%     xlabel('a');
%     ylabel('CCDF');
%     char(legend_id)
%     lgd = legend(char(legend_id));
%     set(lgd,'FontSize',12);
    
    t = strcat('Frames', num2str(i));
%     
     %f = figure('name',t, 'color', 'w');
    
     figure(5)
%     
% %     line(xLimit,[-5 -5],'Color','blue','LineStyle','--')
% %     line(xLimit,[20 20],'Color','blue','LineStyle','--')
%     %boundingBoxArr1 = [ego(1)-ego(3)/2-xLimit(1) ego(2)-ego(4)/2 ego(3) ego(4)];
%     %rectangle('Position', boundingBoxArr1(1,:), 'FaceColor', [1 1 0])
%     %hold on
%     % image(map(:,[xLimit(1):xLimit(2)]))
     [c,p11] = contour(col, row, map(:,[xLimit(1):xLimit(2)]), 'LineWidth', 1.5);
     g = get(p11,'Parent');%对应p1所在的坐标轴
     set(g,'Linewidth',1.5,'FontSize',12,'FontName','Arial','FontWeight','bold');
     ylabel('Width [m]','FontSize',12,'FontName','Arial','FontWeight','bold');
     xlabel('Length [m]','FontSize',12,'FontName','Arial','FontWeight','bold');
     title(t, 'FontSize',10,'FontName','Arial','FontWeight','bold')
%     % contour(row1,col1, map_cdf)
%     % pcolor(col,row,map);
%     % surf(x,y,z); view(0,90); %等效的写法
%     % shading interp; 
     colorbar; colormap(jet);
%     % xlabel('X');ylabel('Y');
%     % set(gca,'FontSize',16)
     set(gca,'Ydir','reverse')
     grid on
%     n = strcat(t,'.jpg');
    %saveas(f, strcat('./res/my/',n));
    
%     figure(2)
%     title(t)
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
    %pause()
    %clf('reset')
end

%xlabel('a');
%ylabel('CCDF');
%char(legend_id);
%lgd = legend(char(legend_id));
%set(lgd,'FontSize',12);