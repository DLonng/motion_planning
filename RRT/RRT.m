clear all; close all; clc;
%% 参数初始化
x_I = 1; y_I = 1;           % 设置初始点
x_G = 750; y_G = 750;       % 设置目标点
% 阈值太小（3）可能找不到解，在终点位置跳动
Thr = 30;                   % 设置目标点阈值范围
% 太大可能来回跳动（400），找不到解
% 太小导致采样点多而密集（4），可能超过迭代次数也无法找到解
Delta = 40;                 % 设置扩展步长
%% 建树初始化 T.v 按照行向量存储 [v1 v2 v3 ... vn]
T.v(1).x = x_I;           	% T 是我们要做的树，v 是节点，这里先把起始点加入到 T 里面来
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;         % 起始节点的父节点仍然是其本身
T.v(1).yPrev = y_I;
T.v(1).dist = 0;            % 从父节点到该节点的距离，这里可取欧氏距离
T.v(1).indPrev = 0;         % 父节点的索引
%% 开始构建随机树
figure(1);           % 创建新窗口
ImpRgb = imread('map.png');
Imp = rgb2gray(ImpRgb);
imshow(Imp)

xL = size(Imp, 1);   % 地图 x 轴长度 800
yL = size(Imp, 2);   % 地图 y 轴长度 800

hold on              % 保留当前绘图
plot(x_I, y_I, 'mo', 'MarkerSize',10, 'MarkerFaceColor','m');   % 绘制起点 m 表示品红色
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');   % 绘制目标点 g 表示绿色

% 保存采样点的个数
count = 1;

% 迭代 3000 次，期间如果找到路径则立即结束算法
for iter = 1 : 3000
    % Step 1: unifrnd 在地图区间 [0, 800] 中随机采样一个点 x_rand
    % 原始 RRT
    x_rand = [unifrnd(0, 800), unifrnd(0, 800)];
    
    % Step 2: 遍历树，从树中找到最近邻近点 x_near
    % 先计算与起点的距离
    minDis = sqrt((x_rand(1) - T.v(1).x)^2 + (x_rand(2) - T.v(1).y)^2);
    minIndex = 1;
    
    % T.v 按行向量存储，size(T.v, 2) 获得节点总数
    % 从第二个节点开始计算与 x_near 的距离
    for i = 2 : size(T.v, 2)
        % 两节点间距离
    	distance = sqrt((x_rand(1) - T.v(i).x)^2 + (x_rand(2) - T.v(i).y)^2);   
        % 每次保存最小距离及索引
        if(distance < minDis)
            minDis = distance;
            minIndex = i;   
        end     
    end
    
    % 找到当前树中离 x_rand 最近的节点 x_near
    x_near(1) = T.v(minIndex).x;    
    x_near(2) = T.v(minIndex).y;
    
    % Step 3: 扩展得到 x_new 节点
    theta = atan2((x_rand(2) - x_near(2)), (x_rand(1) - x_near(1)));
    x_new(1) = x_near(1) + cos(theta) * Delta;
    x_new(2) = x_near(2) + sin(theta) * Delta;
    
    % 检查节点是否是 collision-free
    if ~collisionChecking(x_near, x_new, Imp) 
        continue;   % 有障碍物就舍弃该节点，进行下一次采样
    end
    
    % 采样节点数加 1，也作为新采样点在 T.v 中的行向量索引
    count = count + 1;
    
    % Step 4: 将 x_new 插入树 T 
    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2);
    T.v(count).xPrev = x_near(1);      % x_new 的父节点是 x_near
    T.v(count).yPrev = x_near(2);      % x_new 的父节点是 x_near
    T.v(count).dist = Delta;           % x_new 与父节点 x_near 的步长是固定的
    T.v(count).indPrev = minIndex;     % 保存父节点 x_near 的 index，用于找到路径后的反向回溯
    
    % Step 5: 检查是否到达目标点附近
    disToGoal = sqrt((x_new(1) - x_G)^2 + (x_new(2) - y_G)^2);
    if(disToGoal < Thr)
        break % 到达目标点阈值范围内，结束 RRT
    end
    
    % Step 6: 每一次采样都将 x_near 和 x_new 之间的路径画出来
    plot([x_near(1), x_new(1)], [x_near(2), x_new(2)], 'b', 'Linewidth', 2);
    plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 4, 'MarkerFaceColor','k');
   
    pause(0.02);     % 暂停 0.02s，使得 RRT 扩展过程容易观察
end
%% 路径已经找到，反向查询
if iter < 2000
    % 路径的最后一个坐标为 x_G
    path.pos(1).x = x_G;
    path.pos(1).y = y_G;
    
    % 路径的倒数第二个坐标为 RRT 采样的最后一个节点坐标
    path.pos(2).x = T.v(end).x;
    path.pos(2).y = T.v(end).y;
    
    % RRT 最后一个采样点的父节点索引
    pathIndex = T.v(end).indPrev;
    j = 0;
    
    % 沿终点回溯到起点
    while 1
        path.pos(j + 3).x = T.v(pathIndex).x;
        path.pos(j + 3).y = T.v(pathIndex).y;
        
        % 不断回溯父节点索引
        pathIndex = T.v(pathIndex).indPrev;
        
        % 回溯到第一个节点就退出
        if pathIndex == 1
            break
        end
        j = j + 1;
    end
    
    % 起点加入路径
    path.pos(end).x = x_I;
    path.pos(end).y = y_I;
    
    % 画出路径，实际上是反向的
    % path.pos(1) 是 x_G
    for j = 2 : length(path.pos)
        plot([path.pos(j).x; path.pos(j - 1).x;], [path.pos(j).y; path.pos(j - 1).y], 'g', 'Linewidth', 4);
    end
else
    disp('Error, no path found!');
end
