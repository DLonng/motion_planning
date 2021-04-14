function feasible = collisionChecking(startPose, goalPose, map)
    % Ĭ��û�������ϰ���
    feasible = true;
    
    % ������㣬�յ�֮��ļн�
    theta = atan2(goalPose(2) - startPose(2), goalPose(1) - startPose(1));
    
    % �� 0.5 Ϊ����, �� startPose ��ʼ�����ļ���Ƿ����ϰ�
    % (startPose - goalPose).^2 = (x1 - x2)^2, (y1 - y2)^2
    % sum((startPose - goalPose).^2) = (x1 - x2)^2 + (y1 - y2)^2
    for r = 0 : 0.5 : sqrt(sum((startPose - goalPose).^2))
        % delta_x = r * cos(theta)
        % delta_y = r * sin(theta)
        % posCheck.x = startPose.x + delta_x
        % posCheck.y = startPose.y + delta_y
        posCheck = startPose + r.*[cos(theta) sin(theta)];      % ֱ�߾������� 0.5 �������

        % ��һ��С��(x,y)�� 4 ������ȡ����ֻ�иõ�� 4 ������û�д����ϰ����ǿ��е�
        % ����ֻҪ��һ�����������ϰ�����ǲ����е�
        % feasiblePoint ���ϰ����� false����ʾ�õ㲻����
        % ceil ����ȡ�������ش��ڻ��ߵ���ָ��������С����
        % floor ����ȡ��������С�ڻ��ߵ���ָ�������������
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
    % ��Խ����Ϊ��ɫ���ؾͲ����ϰ���㣬����Ϊ�ϰ���㣨Խ�� + ��ɫ���أ�
    if ~(point(1) >= 1 && point(1) <= size(map, 1) && point(2) >= 1 ...
        && point(2) <= size(map, 2) && map(point(2), point(1)) == 255)
        feasible = false;   % ���ϰ�
    end
end