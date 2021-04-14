function feasible = collisionChecking(startPose, goalPose, map)
    % 默认没有碰到障碍物
    feasible = true;
    
    % 计算起点，终点之间的夹角
    theta = atan2(goalPose(2) - startPose(2), goalPose(1) - startPose(1));
    
    % 以 0.5 为步长, 从 startPose 开始递增的检查是否有障碍
    % (startPose - goalPose).^2 = (x1 - x2)^2, (y1 - y2)^2
    % sum((startPose - goalPose).^2) = (x1 - x2)^2 + (y1 - y2)^2
    for r = 0 : 0.5 : sqrt(sum((startPose - goalPose).^2))
        % delta_x = r * cos(theta)
        % delta_y = r * sin(theta)
        % posCheck.x = startPose.x + delta_x
        % posCheck.y = startPose.y + delta_y
        posCheck = startPose + r.*[cos(theta) sin(theta)];      % 直线距离增加 0.5 后的坐标

        % 将一个小数(x,y)向 4 个方向取整，只有该点的 4 个方向都没有触碰障碍才是可行点
        % 否则只要有一个方向碰到障碍物就是不可行点
        % feasiblePoint 有障碍返回 false，表示该点不可行
        % ceil 向上取整：返回大于或者等于指定数的最小整数
        % floor 向下取整：返回小于或者等于指定数的最大整数
        if ~(feasiblePoint(ceil(posCheck), map) ...
            && feasiblePoint(floor(posCheck), map) ...
            && feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))], map) ...
            && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))], map))
            feasible = false;
            break;
        end

        if ~feasiblePoint([floor(goalPose(1)), ceil(goalPose(2))], map)
            feasible = false; 
        end
    end
end

function feasible = feasiblePoint(point, map)
    feasible = true;
    % 不越界且为白色像素就不是障碍物点，否则为障碍物点（越界 + 黑色像素）
    if ~(point(1) >= 1 && point(1) <= size(map, 1) && point(2) >= 1 ...
        && point(2) <= size(map, 2) && map(point(2), point(1)) == 255)
        feasible = false;   % 有障碍
    end
end