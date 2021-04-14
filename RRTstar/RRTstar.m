clear all; close all; clc;
%% ������ʼ��
x_I = 1; y_I = 1;           % ���ó�ʼ��
x_G = 750; y_G = 750;       % ����Ŀ���
GoalThreshold = 30;         % ����Ŀ�����ֵ
Delta = 30;                 % ������չ���� default:30
% r ̫С��7����ÿ���Ż����ھӵ���٣��Ż�����̫С��·�������� RRT
% r ̫��700����ÿ���Ż����ھӵ�ǳ���(����������)���Ż�����̫��
% �Բ��� 1 ��Ӱ�죺�����²����� x_new �ĸ��ڵ����Ǳ��Ż�Ϊ��㣨��Ϊ�ܳɱ����õ�����ŷʽ����������²����ĵ㵽�����ܳɱ�����С�ģ�����֮��ֱ����̣�
% �Բ��� 2 ��Ӱ�죺��Χ̫���¶����е�����¼��㵽������ʹ��ۣ��������������ֱ�ӰѺܶ������ĸ��ڵ��Ż�Ϊ��㣬��Ϊ����������Կ���ֱ��ѡ�������Ϊ���ڵ�
% ��·����Ӱ�죺��������٣�����������֦���ܼ�����Ϊ�ܶ������ĸ��ڵ㱻�Ż�Ϊ�����
RadiusForNeib = 70;          % RRT* �µĲ�����rewire �ķ�Χ,�뾶 r
MaxIterations = 1000;       % ����������
UpdateTime = 50;            % ����·����ʱ����
DelayTime = 0.0;            % ��ͼ�ӳ�ʱ��
%% ������ʼ��:T ����,v �ǽڵ�
T.v(1).x = x_I;             % ����ʼ�ڵ���뵽T��
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;         % �ڵ�ĸ��ڵ�����:���ĸ��ڵ����䱾��
T.v(1).yPrev = y_I;
T.v(1).totalCost = 0;       % ����ʼ�ڵ㿪ʼ���ۼ� cost������ȡŷ�Ͼ���
T.v(1).indPrev = 0;         % ���ڵ������
%% ��ʼ������
figure(1);
ImpRgb = imread('map.png');
Imp = rgb2gray(ImpRgb);
imshow(Imp)
xL = size(Imp,1);   % ��ͼ x �᳤��
yL = size(Imp,2);   % ��ͼ y �᳤��
hold on
plot(x_I, y_I, 'mo', 'MarkerSize',10, 'MarkerFaceColor','m');   % ��������Ŀ���
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
count = 1;
% ��������ѡ��ĸ��ڵ㵽�²����ڵ�Ļ�ͼ���
pHandleList = [];
% �����²�����Ļ�ͼ���
lHandleList = [];
resHandleList = [];
findPath = 0;
update_count = 0;
path.pos = [];

% RRT* �ﵽ���������Ž����㷨
for iter = 1 : MaxIterations
    % Step 1: �ڵ�ͼ���������һ���� x_rand (Sample)
    x_rand = [unifrnd(0, xL),unifrnd(0, yL)];
    
    % Step 2: ���������������ҵ�����ڽ��� x_near (Near)
    minDis = sqrt((x_rand(1) - T.v(1).x)^2 + (x_rand(2) - T.v(1).y)^2);
    minIndex = 1;
    
    % T.v ���������洢��size(T.v, 2)��ýڵ�����
    for i = 2 : size(T.v,2)	
    	distance = sqrt((x_rand(1) - T.v(i).x)^2 + (x_rand(2) - T.v(i).y)^2);   % ���ڵ�����
        if(distance < minDis)
            minDis = distance;
            minIndex = i;
        end
    end
    
    % �ҵ��˵�ǰ������ x_rand ����Ľڵ�
    x_near(1) = T.v(minIndex).x;    
    x_near(2) = T.v(minIndex).y;
    temp_parent = minIndex;                        % ��ʱ���ڵ� x_near ������
    temp_cost = T.v(minIndex).totalCost + Delta;   % ��ʱ�ۼƴ��� x_i ->(totalCost) x_near ->(Delta) x_rand

    % Step 3: ��չ�õ� x_new �ڵ� (Steer)
    theta = atan2((x_rand(2) - x_near(2)), (x_rand(1) - x_near(1)));
    x_new(1) = x_near(1) + cos(theta) * Delta;
    x_new(2) = x_near(2) + sin(theta) * Delta;  
    %plot(x_rand(1), x_rand(2), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
    %plot(x_new(1), x_new(2), 'bo', 'MarkerSize',10, 'MarkerFaceColor','b');
    
    % ���ڵ��Ƿ��� collision-free
    if ~collisionChecking(x_near, x_new, Imp) 
        continue;   % ���ϰ���
    end

    % Step 4: ���� x_new ΪԲ��,�뾶Ϊ R ��Բ�������ڵ� (NearC)
    % ÿ��ѭ��Ҫ�Ѷ������
    disToNewList = [];      % �����ھӵ㵽 x_new ��ŷʽ���루cost�� 
    nearIndexList = [];     % �����ھӵ������
    % count �ǲ����������������������еĲ�����
    for index_near = 1 : count
        disTonew = sqrt((x_new(1) - T.v(index_near).x)^2 + (x_new(2) - T.v(index_near).y)^2);
        if(disTonew < RadiusForNeib)                        % ��������: ŷ�Ͼ���С�� R
            disToNewList = [disToNewList disTonew];         % �������������нڵ㵽 x_new ��ŷʽ���� cost
            nearIndexList = [nearIndexList index_near];     % ���������������ھӽڵ������ T ������
        end
    end
    
    % Step 5: ����ѡ�� x_new �ĸ��ڵ㣬ʹ x_new ���ۼ� cost ��С (ChooseParent)
    % cost_index ���ھӽڵ������
    for cost_index = 1 : length(nearIndexList)
        % ����ÿ���ھӽڵ���Ϊ x_new ���ڵ���ܴ��ۣ�x_i -> x_near -> x_new
        costToNew = T.v(nearIndexList(cost_index)).totalCost + disToNewList(cost_index);
        % temp_cost Ϊ������ x_new ʱ�ĳ�ʼ����
        if(costToNew < temp_cost)
            % �ھӽڵ㵽 x_new ���ܳɱ���С�͸���Ϊ x_new �ĸ��ڵ�
            x_mincost(1) = T.v(nearIndexList(cost_index)).x;
            x_mincost(2) = T.v(nearIndexList(cost_index)).y;
            
            % ��⸸�ӽڵ�������Ƿ�������ϰ���
            if ~collisionChecking(x_mincost, x_new, Imp) 
            	continue;   % ���ϰ���
            end
            
        	temp_cost = costToNew;                      % ������С�ɱ�
        	temp_parent = nearIndexList(cost_index);    % ������С�ɱ����ھӽڵ�
        end
    end
    
    % Step 6: �� x_new ������ T (AddNodeEdge)��count Ҳ�����½ڵ������
    count = count + 1;
    
    T.v(count).x = x_new(1);          
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = T.v(temp_parent).x;      % ���½ڵ�ĸ��ڵ�Ϊ����ѡ����ھӸ��ڵ� temp_parent
    T.v(count).yPrev = T.v(temp_parent).y;      
    T.v(count).totalCost = temp_cost;           % ���½ڵ���ܳɱ�Ϊ����ѡ����ھӸ��ڵ�����³ɱ� costToNew
    T.v(count).indPrev = temp_parent;           % ����ѡ��ĸ��ڵ� x_near �� index
    
    % ������ѡ��ĸ��ڵ㵽 x_new ֮�仭һ����ɫ����
    % T.v(count).xPrev Ϊ��һ������ѡ��ĸ��ڵ�
    l_handle = plot([T.v(count).xPrev, x_new(1)], [T.v(count).yPrev, x_new(2)], 'b', 'Linewidth', 2);
    % ���²����� x_new ��������ɫ k ԲȦ o ��� Marker
    p_handle = plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 4, 'MarkerFaceColor','k');
    
    % ��ͼ�ľ��������Ϊ count
    pHandleList = [pHandleList p_handle];    
    lHandleList = [lHandleList l_handle];
    pause(DelayTime);
    
    % Step 7: ���ھӽڵ����²��� (rewire)�������Ƿ���Ҫ�� x_new �����ھӽڵ�ĸ��ڵ�
    for rewire_index = 1 : length(nearIndexList)
        % temp_parent Ϊ x_new �����и��ڵ㣬�������²���
        if(nearIndexList(rewire_index) ~= temp_parent)
            % ������� -> x_new -> �ھӽڵ���ܴ���
            newCost = temp_cost + disToNewList(rewire_index);
            % �� x_new ��Ϊ�ھӽڵ���ܳɱ� newCost С��ԭʼ�ھӽڵ���ܳɱ������滻�ھӽڵ�ĸ��ڵ�Ϊ x_new
            if(newCost < T.v(nearIndexList(rewire_index)).totalCost)
                % �õ������Ż����ھӽڵ�����
                x_neib(1) = T.v(nearIndexList(rewire_index)).x;
                x_neib(2) = T.v(nearIndexList(rewire_index)).y;
                
                % �жϸ��ӽڵ������Ƿ�������ϰ���
                if ~collisionChecking(x_neib, x_new, Imp) 
                    continue;   % ���ϰ���
                end
                
                T.v(nearIndexList(rewire_index)).xPrev = x_new(1);      % ���ھӽڵ�ĸ��ڵ��滻Ϊ x_new
                T.v(nearIndexList(rewire_index)).yPrev = x_new(2);
                T.v(nearIndexList(rewire_index)).totalCost = newCost;   % ���ھӽڵ���ܳɱ�Ҳ�滻Ϊ�µ�
                T.v(nearIndexList(rewire_index)).indPrev = count;       % �����Ż����ھӽڵ�ĸ��ڵ�����Ϊ x_new ������

                %delete(pHandleList());
                %delete(lHandleList(nearIndexList(rewire_index)));
                % �� x_new ���Ż����ڵ�Ϊ x_new ���ھӽڵ�֮����ƺ�ɫ���߶�
                lHandleList(nearIndexList(rewire_index)) = plot([T.v(nearIndexList(rewire_index)).x, x_new(1)], [T.v(nearIndexList(rewire_index)).y, x_new(2)], 'r', 'Linewidth', 2);

                %pHandleList = [pHandleList p_handle];    %��ͼ�ľ��������Ϊcount
                %lHandleList = [lHandleList l_handle];
            end
        end
    end
    
    % Step 8: ����Ƿ񵽴�Ŀ��㸽�� 
    disToGoal = sqrt((x_new(1) - x_G)^2 + (x_new(2) - y_G)^2);
    if(disToGoal < GoalThreshold && ~findPath)    % �ҵ�Ŀ��㣬������ֻ����һ��
        % �����Ѿ��ҵ�·��
        findPath = 1;

        count = count + 1;    % �ֶ��� Goal ���뵽����
        Goal_index = count;
        T.v(count).x = x_G;          
        T.v(count).y = y_G; 
        T.v(count).xPrev = x_new(1);     % Ŀ��ڵ�ĸ��ڵ�Ϊ���µ� x_new
        T.v(count).yPrev = x_new(2);
        T.v(count).totalCost = T.v(count - 1).totalCost + disToGoal; % Ŀ��ڵ���ܳɱ�Ϊ���ڵ�ɱ� + ���յ�ĳɱ�
        T.v(count).indPrev = count - 1;     % �丸�ڵ� x_near �� index
    end
    
    % �ҵ�·���ͻ��ݣ����ǲ����� RRT* �㷨
    if(findPath == 1)
        update_count = update_count + 1;
        % UpdateTime = 50 �θ���һ��·��
        if(update_count == UpdateTime)
            update_count = 0;
            j = 2;
            % path �ĵ�һ��Ԫ��Ϊ·�����յ�
            path.pos(1).x = x_G; 
            path.pos(1).y = y_G;
            pathIndex = T.v(Goal_index).indPrev;
            
            while 1
                path.pos(j).x = T.v(pathIndex).x;
                path.pos(j).y = T.v(pathIndex).y;
                pathIndex = T.v(pathIndex).indPrev;    % ���յ���ݵ����
                if pathIndex == 0
                    break
                end
                j = j + 1;
            end  
            
            % ÿ�λ��� RRT* �Ż���·��ǰ�����ϴ��Ż���·��������
            for delete_index = 1:length(resHandleList)
            	delete(resHandleList(delete_index));
            end
            
            % ����ÿ�ε����Ż�����ɫ RRT* ·�������յ���������·����
            % path �ĵ�һ��Ԫ��Ϊ·�����յ㣬���һ��Ԫ��Ϊ·�������
            for j = 2 : length(path.pos)
                res_handle = plot([path.pos(j).x; path.pos(j - 1).x;], [path.pos(j).y; path.pos(j - 1).y], 'g', 'Linewidth', 4);
                resHandleList = [resHandleList res_handle];
            end
        end
    end
	pause(DelayTime); % ��ͣ DelayTime s,ʹ�� RRT* ��չ�������׹۲�
end

% �ٴ����·����Ϊ��һ�����»��������Ż���·����׼��
for delete_index = 1:length(resHandleList)
	delete(resHandleList(delete_index));
end

% ���»�������·�� path����֤���ᱻ������ɫ��·���ڵ�
for j = 2 : length(path.pos)
	res_handle = plot([path.pos(j).x; path.pos(j - 1).x;], [path.pos(j).y; path.pos(j - 1).y], 'g', 'Linewidth', 4);
	resHandleList = [resHandleList res_handle];
end
            
disp('The path is found!');

