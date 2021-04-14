clear all; close all; clc;
%% ������ʼ��
x_I = 1; y_I = 1;           % ���ó�ʼ��
x_G = 750; y_G = 750;       % ����Ŀ���
% ��ֵ̫С��3�������Ҳ����⣬���յ�λ������
Thr = 30;                   % ����Ŀ�����ֵ��Χ
% ̫���������������400�����Ҳ�����
% ̫С���²��������ܼ���4�������ܳ�����������Ҳ�޷��ҵ���
Delta = 40;                 % ������չ����
%% ������ʼ�� T.v �����������洢 [v1 v2 v3 ... vn]
T.v(1).x = x_I;           	% T ������Ҫ��������v �ǽڵ㣬�����Ȱ���ʼ����뵽 T ������
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;         % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
T.v(1).yPrev = y_I;
T.v(1).dist = 0;            % �Ӹ��ڵ㵽�ýڵ�ľ��룬�����ȡŷ�Ͼ���
T.v(1).indPrev = 0;         % ���ڵ������
%% ��ʼ���������
figure(1);           % �����´���
ImpRgb = imread('map.png');
Imp = rgb2gray(ImpRgb);
imshow(Imp)

xL = size(Imp, 1);   % ��ͼ x �᳤�� 800
yL = size(Imp, 2);   % ��ͼ y �᳤�� 800

hold on              % ������ǰ��ͼ
plot(x_I, y_I, 'mo', 'MarkerSize',10, 'MarkerFaceColor','m');   % ������� m ��ʾƷ��ɫ
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');   % ����Ŀ��� g ��ʾ��ɫ

% ���������ĸ���
count = 1;

% ���� 3000 �Σ��ڼ�����ҵ�·�������������㷨
for iter = 1 : 3000
    % Step 1: unifrnd �ڵ�ͼ���� [0, 800] ���������һ���� x_rand
    x_rand = [unifrnd(0, 800), unifrnd(0, 800)];
    
    % Step 2: ���������������ҵ�����ڽ��� x_near
    % �ȼ��������ľ���
    minDis = sqrt((x_rand(1) - T.v(1).x)^2 + (x_rand(2) - T.v(1).y)^2);
    minIndex = 1;
    
    % T.v ���������洢��size(T.v, 2) ��ýڵ�����
    % �ӵڶ����ڵ㿪ʼ������ x_near �ľ���
    for i = 2 : size(T.v, 2)
        % ���ڵ�����
    	distance = sqrt((x_rand(1) - T.v(i).x)^2 + (x_rand(2) - T.v(i).y)^2);   
        % ÿ�α�����С���뼰����
        if(distance < minDis)
            minDis = distance;
            minIndex = i;   
        end     
    end
    
    % �ҵ���ǰ������ x_rand ����Ľڵ� x_near
    x_near(1) = T.v(minIndex).x;    
    x_near(2) = T.v(minIndex).y;
    
    % Step 3: ��չ�õ� x_new �ڵ�
    theta = atan2((x_rand(2) - x_near(2)), (x_rand(1) - x_near(1)));
    x_new(1) = x_near(1) + cos(theta) * Delta;
    x_new(2) = x_near(2) + sin(theta) * Delta;
    
    % ���ڵ��Ƿ��� collision-free
    if ~collisionChecking(x_near, x_new, Imp) 
        continue;   % ���ϰ���������ýڵ㣬������һ�β���
    end
    
    % �����ڵ����� 1��Ҳ��Ϊ�²������� T.v �е�����������
    count = count + 1;
    
    % Step 4: �� x_new ������ T 
    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2);
    T.v(count).xPrev = x_near(1);      % x_new �ĸ��ڵ��� x_near
    T.v(count).yPrev = x_near(2);      % x_new �ĸ��ڵ��� x_near
    T.v(count).dist = Delta;           % x_new �븸�ڵ� x_near �Ĳ����ǹ̶���
    T.v(count).indPrev = minIndex;     % ���游�ڵ� x_near �� index�������ҵ�·����ķ������
    
    % Step 5: ����Ƿ񵽴�Ŀ��㸽��
    disToGoal = sqrt((x_new(1) - x_G)^2 + (x_new(2) - y_G)^2);
    if(disToGoal < Thr)
        break % ����Ŀ�����ֵ��Χ�ڣ����� RRT
    end
    
    % Step 6: ÿһ�β������� x_near �� x_new ֮���·��������
    plot([x_near(1), x_new(1)], [x_near(2), x_new(2)], 'b', 'Linewidth', 2);
    plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 4, 'MarkerFaceColor','k');
   
    pause(0.02);     % ��ͣ 0.02s��ʹ�� RRT ��չ�������׹۲�
end
%% ·���Ѿ��ҵ��������ѯ
if iter < 2000
    % ·�������һ������Ϊ x_G
    path.pos(1).x = x_G;
    path.pos(1).y = y_G;
    
    % ·���ĵ����ڶ�������Ϊ RRT ���������һ���ڵ�����
    path.pos(2).x = T.v(end).x;
    path.pos(2).y = T.v(end).y;
    
    % RRT ���һ��������ĸ��ڵ�����
    pathIndex = T.v(end).indPrev;
    j = 0;
    
    % ���յ���ݵ����
    while 1
        path.pos(j + 3).x = T.v(pathIndex).x;
        path.pos(j + 3).y = T.v(pathIndex).y;
        
        % ���ϻ��ݸ��ڵ�����
        pathIndex = T.v(pathIndex).indPrev;
        
        % ���ݵ���һ���ڵ���˳�
        if pathIndex == 1
            break
        end
        j = j + 1;
    end
    
    % ������·��
    path.pos(end).x = x_I;
    path.pos(end).y = y_I;
    
    % ����·����ʵ�����Ƿ����
    % path.pos(1) �� x_G
    for j = 2 : length(path.pos)
        plot([path.pos(j).x; path.pos(j - 1).x;], [path.pos(j).y; path.pos(j - 1).y], 'g', 'Linewidth', 4);
    end
else
    disp('Error, no path found!');
end