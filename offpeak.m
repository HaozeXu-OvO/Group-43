%% 交通仿真模型
%% 初始化环境
clear all;
close all;
clc;

%% 定义常量
% 将常量放在一个结构体中，便于传递
global CONFIG
CONFIG.LANE_WIDTH = 3; % 车道宽度(m)
CONFIG.CAR_LENGTH = 4; % 汽车长度(m)
CONFIG.CAR_WIDTH = 2; % 汽车宽度(m)
CONFIG.BIKE_LENGTH = 2; % 自行车长度(m)
CONFIG.BIKE_WIDTH = 0.8; % 自行车宽度(m)
CONFIG.CAR_MAX_SPEED = 20 * 0.44704; % 汽车最大速度(英里/h转换为m/s)
CONFIG.BIKE_MAX_SPEED = 9 * 0.44704; % 自行车最大速度(英里/h转换为m/s)
CONFIG.CAR_MAX_ACCEL = 4.5; % 汽车最大加速度(m/s²)
CONFIG.BIKE_MAX_ACCEL = 3; % 自行车最大加速度(m/s²)
CONFIG.CAR_MAX_DECEL = 4.5; % 汽车最大减速度(m/s²)
CONFIG.BIKE_MAX_DECEL = 3.0; % 自行车最大减速度(m/s²)
CONFIG.SIM_TIME = 300; % 仿真总时长(s)
CONFIG.TIME_STEP = 0.5; % 仿真时间步长(s)
CONFIG.SIGNAL_CYCLE = 30; % 信号灯周期(s)
CONFIG.PED_PHASE_PROB = 0.2; % 行人通行相位触发概率
CONFIG.SAFE_DISTANCE_MIN = 0.5; % 最小安全距离(m)
CONFIG.HEADWAY_TIME = 1.5; % 车头时距(s)
CONFIG.CAR_TO_BIKE_RATIO = 4; % 汽车与自行车生成比例
CONFIG.SPAWN_RATE = 1; % 入口尝试新车辆生成概率
CONFIG.SIGNAL_ARROW_LENGTH = 4;  % 信号箭头/方向指示线的长度（单位：像素 or 地图单位）
CONFIG.LOW_SPEED_RATIO = 0.5;  % 表示低于 50% maxSpeed 时记为低速
%% 读取数据
disp('读取节点和道路数据...');
[nodes, roads] = readTrafficData('model_updated_nodes_converted.xlsx');

%% 建立模型
disp('建立模型...');
% 建立拓扑结构模型（节点、道路、路口）
trafficModel = buildTopologyModel(nodes, roads);

% 初始化信号灯
trafficLights = initializeTrafficLights(trafficModel);

% 初始化交通参与者（汽车和自行车）
% 使用结构体数组初始化，确保字段存在
vehicles = struct('id', {}, 'type', {}, 'laneIdx', {}, 'x', {}, 'y', {}, ...
                  'direction', {}, 'speed', {}, 'acceleration', {}, 'distance', {}, ...
                  'nextNodeIdx', {}, 'status', {}, 'waitingTime', {}, 'stopCount', {}, ...
                  'totalTime', {}, 'lowSpeedTime', {}, 'length', {}, 'width', {}, ...
                  'maxSpeed', {}, 'maxAccel', {}, 'maxDecel', {});

%% 初始化统计数据
numRoads = numel(roads.id);  % 正确的道路数量
stats = struct();
stats.roadCapacity = zeros(numRoads, ceil(CONFIG.SIM_TIME/2));
stats.pedestrianPhaseTime = 0;
stats.carLowSpeedTime = 0;
stats.carTotalTime = 0;
stats.carWaitingTime = 0;
stats.carStopCount = 0;
stats.carCount = 0;
stats.bikeLowSpeedTime = 0;
stats.bikeTotalTime = 0;
stats.bikeWaitingTime = 0;
stats.bikeStopCount = 0;
stats.bikeCount = 0;


%% 初始化可视化
disp('初始化可视化...');
figHandle = initializeVisualization(trafficModel);

%% 运行仿真
disp('开始仿真...');
statUpdateCounter = 0;
for time = 0:CONFIG.TIME_STEP:CONFIG.SIM_TIME
    % 更新信号灯状态
    trafficLights = updateTrafficLights(trafficLights, time);
    
    % 生成新的交通参与者
    vehicles = spawnVehicles(vehicles, trafficModel, roads, time);
    
    % 更新交通参与者位置
    [vehicles, stats] = updateVehicles(vehicles, trafficModel, trafficLights, stats, time, CONFIG.TIME_STEP);
    
    % 更新统计数据（每0.5秒更新一次）
    statUpdateCounter = statUpdateCounter + CONFIG.TIME_STEP;
    if statUpdateCounter >= 0.5
        stats = updateStatistics(vehicles, trafficModel, roads, stats, time);
        statUpdateCounter = 0;
    end
    
    if ~isgraphics(figHandle)
        disp('检测到figHandle无效，重新初始化图窗...');
        figHandle = initializeVisualization(trafficModel);
    end
    % 更新可视化
    updateVisualization(figHandle, trafficModel, vehicles, trafficLights, stats, time);
    
    % 显示进度
    if mod(time, 10) < CONFIG.TIME_STEP
        fprintf('仿真进度: %0.1f%%\n', time/CONFIG.SIM_TIME*100);
    end
    
    % 暂停以模拟实时效果（可根据需要调整）
    pause(0.01);
end

%% 显示最终统计结果
displayFinalStatistics(stats);

%% 函数: 读取交通数据
function [nodes, roads] = readTrafficData(filename)
    % 读取Excel文件中的节点和道路数据
    nodeData = readtable(filename, 'Sheet', 'Sheet1', 'VariableNamingRule', 'preserve');
    roadData = readtable(filename, 'Sheet', 'Sheet2', 'VariableNamingRule', 'preserve');
    
    % 处理节点数据
    nodes = struct();
    nodes.id = nodeData{:, 1};  % 节点ID
    nodes.x = nodeData{:, 2};   % 节点横坐标
    nodes.y = nodeData{:, 3};   % 节点纵坐标
    nodes.type = nodeData{:, 4}; % 路口类型 (X, T, A, B)
    nodes.hasSignal = strcmp(nodeData{:, 5}, 'Y'); % 是否有信号灯
    
    % 将节点ID转换为字符串格式
    if ~iscell(nodes.id)
        nodes.id = cellstr(num2str(nodes.id));
    end
    
    % 处理道路数据
    roads = struct();
    roadIds = roadData{:, 1};  % 道路ID (例如 "0;1")
    
    % 将道路ID分解为起始和结束节点
    roadSplit = cellfun(@(x) strsplit(x, ';'), roadIds, 'UniformOutput', false);
    startNodes = cellfun(@(x) x{1}, roadSplit, 'UniformOutput', false);
    endNodes = cellfun(@(x) x{2}, roadSplit, 'UniformOutput', false);
    
    roads.id = roadIds;
    roads.startNode = startNodes;
    roads.endNode = endNodes;
    roads.laneCount = roadData{:, 2};  % 车道数量
    
    % 小节点到大节点的车道
    roads.smallToBigLanes = cell(size(roadIds));
    for i = 1:size(roadData, 1)
        lanes = [];
        for j = 3:6  % sbl1 to sbl4
            if j <= size(roadData, 2) && ~isnan(roadData{i, j})
                lanes = [lanes, roadData{i, j}];
            end
        end
        roads.smallToBigLanes{i} = lanes;
    end
    
    % 大节点到小节点的车道
    roads.bigToSmallLanes = cell(size(roadIds));
    for i = 1:size(roadData, 1)
        lanes = [];
        for j = 10:13  % bsl1 to bsl4
            if j <= size(roadData, 2) && ~isnan(roadData{i, j})
                lanes = [lanes, roadData{i, j}];
            end
        end
        roads.bigToSmallLanes{i} = lanes;
    end
    
    % 小到大方向的下一个可去节点
    roads.smallToBigNextNodes = cell(size(roadIds));
    for i = 1:size(roadData, 1)
        nextNodes = [];
        for j = 7:9  % sbnn1 to sbnn3
            if j <= size(roadData, 2) && ~isnan(roadData{i, j})
                nextNodes = [nextNodes, roadData{i, j}];
            end
        end
        roads.smallToBigNextNodes{i} = nextNodes;
    end
    
    % 大到小方向的下一个可去节点
    roads.bigToSmallNextNodes = cell(size(roadIds));
    for i = 1:size(roadData, 1)
        nextNodes = [];
        for j = 14:16  % bsnn1 to bsnn3
            if j <= size(roadData, 2) && ~isnan(roadData{i, j})
                nextNodes = [nextNodes, roadData{i, j}];
            end
        end
        roads.bigToSmallNextNodes{i} = nextNodes;
    end
    
    % 汽车生成车道
    roads.carGenLanes = cell(size(roadIds));
    for i = 1:size(roadData, 1)
        lanes = [];
        for j = 17:20  % gl1 to gl4
            if j <= size(roadData, 2) && ~isnan(roadData{i, j})
                lanes = [lanes, roadData{i, j}];
            end
        end
        roads.carGenLanes{i} = lanes;
    end
end

%% 函数: 建立拓扑结构模型
function model = buildTopologyModel(nodes, roads)
    % 创建交通网络拓扑结构模型
    model = struct();
    model.nodes = nodes;
    model.roads = roads;
    
    % 计算道路长度和方向
    disp(roads.id);
    disp(class(roads.id));
    numRoads = numel(roads.id);
    model.roadLength = zeros(numRoads, 1);
    model.roadDirection = zeros(numRoads, 2); % 单位方向向量[dx, dy]
    
    for i = 1:numRoads
        startIdx = find(strcmp(nodes.id, roads.startNode{i}));
        endIdx = find(strcmp(nodes.id, roads.endNode{i}));
        
        if ~isempty(startIdx) && ~isempty(endIdx)
            dx = nodes.x(endIdx) - nodes.x(startIdx);
            dy = nodes.y(endIdx) - nodes.y(startIdx);
            roadLen = sqrt(dx^2 + dy^2);
            
            model.roadLength(i) = roadLen;
            if roadLen > 0
                model.roadDirection(i, :) = [dx/roadLen, dy/roadLen];
            end
        end
    end
    
    % 计算车道信息
    model.lanes = buildLanes(model);
    
    % 计算路口信息
    model.intersections = buildIntersections(model);
    
    % 设置X和T路口的方向组
    model = setIntersectionGroups(model);
end

%% 函数: 构建车道模型
function lanes = buildLanes(model)
    global CONFIG
    % 为每条道路创建车道
    nodes = model.nodes;
    roads = model.roads;
    numRoads = numel(roads.id);
    
    lanes = struct();
    lanes.roadIndex = []; % 所属道路索引
    lanes.laneNumber = []; % 车道编号
    lanes.direction = []; % 方向 (1=小到大, -1=大到小)
    lanes.startX = []; % 起始X坐标
    lanes.startY = []; % 起始Y坐标
    lanes.endX = []; % 结束X坐标
    lanes.endY = []; % 结束Y坐标
    lanes.nextNodes = {}; % 可去的下一个节点
    lanes.isCarGenLane = []; % 是否为车辆生成车道
    
    laneCount = 0;
    
    for i = 1:numRoads
        startIdx = find(strcmp(nodes.id, roads.startNode{i}));
        endIdx = find(strcmp(nodes.id, roads.endNode{i}));
        
        if isempty(startIdx) || isempty(endIdx)
            continue;
        end
        
        % 道路起点和终点
        startX = nodes.x(startIdx);
        startY = nodes.y(startIdx);
        endX = nodes.x(endIdx);
        endY = nodes.y(endIdx);
        
        % 道路方向向量
        dx = endX - startX;
        dy = endY - startY;
        roadLen = sqrt(dx^2 + dy^2);
        dirX = dx / roadLen;
        dirY = dy / roadLen;
        
        % 垂直于道路方向的向量
        perpX = -dirY;
        perpY = dirX;
        
        % 总车道数
        totalLanes = roads.laneCount(i);
        
        % 小到大方向的车道
        smallToBigLanes = roads.smallToBigLanes{i};
        for j = 1:numel(smallToBigLanes)
            laneCount = laneCount + 1;
            laneNum = smallToBigLanes(j);
            
            % 根据车道编号计算车道位置偏移
            offset = (laneNum - 0.5 - totalLanes/2) * CONFIG.LANE_WIDTH;
            
            lanes.roadIndex(laneCount) = i;
            lanes.laneNumber(laneCount) = laneNum;
            lanes.direction(laneCount) = 1; % 小到大方向
            
            % 车道起点和终点
            lanes.startX(laneCount) = startX + perpX * offset;
            lanes.startY(laneCount) = startY + perpY * offset;
            lanes.endX(laneCount) = endX + perpX * offset;
            lanes.endY(laneCount) = endY + perpY * offset;
            
            % 下一个可去节点
            if ~isempty(roads.smallToBigNextNodes{i})
                nextNodeIds = num2cell(roads.smallToBigNextNodes{i});
                lanes.nextNodes{laneCount} = cellfun(@num2str, nextNodeIds, 'UniformOutput', false);
            else
                lanes.nextNodes{laneCount} = {};
            end
            
            % 检查是否为汽车生成车道
            lanes.isCarGenLane(laneCount) = any(roads.carGenLanes{i} == laneNum);
        end
        
        % 大到小方向的车道
        bigToSmallLanes = roads.bigToSmallLanes{i};
        for j = 1:numel(bigToSmallLanes)
            laneCount = laneCount + 1;
            laneNum = bigToSmallLanes(j);
            
            % 根据车道编号计算车道位置偏移
            offset = (laneNum - 0.5 - totalLanes/2) * CONFIG.LANE_WIDTH;
            
            lanes.roadIndex(laneCount) = i;
            lanes.laneNumber(laneCount) = laneNum;
            lanes.direction(laneCount) = -1; % 大到小方向
            
            % 车道起点和终点（注意起点和终点与小到大方向相反）
            lanes.startX(laneCount) = endX + perpX * offset;
            lanes.startY(laneCount) = endY + perpY * offset;
            lanes.endX(laneCount) = startX + perpX * offset;
            lanes.endY(laneCount) = startY + perpY * offset;
            
            % 下一个可去节点
            if ~isempty(roads.bigToSmallNextNodes{i})
                nextNodeIds = num2cell(roads.bigToSmallNextNodes{i});
                lanes.nextNodes{laneCount} = cellfun(@num2str, nextNodeIds, 'UniformOutput', false);
            else
                lanes.nextNodes{laneCount} = {};
            end
            
            % 检查是否为汽车生成车道
            lanes.isCarGenLane(laneCount) = any(roads.carGenLanes{i} == laneNum);
        end
    end
end

%% 函数: 构建路口模型
function intersections = buildIntersections(model)
    % 为每个节点创建路口模型
    nodes = model.nodes;
    roads = model.roads;
    numNodes = numel(nodes.id);
    
    intersections = struct();
    intersections.nodeIndex = 1:numNodes;
    intersections.type = nodes.type;
    intersections.hasSignal = nodes.hasSignal;
    intersections.connectedRoads = cell(numNodes, 1);
    intersections.stopLines = cell(numNodes, 1);
    
    % 计算每个节点连接的道路
    for i = 1:numel(roads.id)
        startNodeId = roads.startNode{i};
        endNodeId = roads.endNode{i};
        
        startIdx = find(strcmp(nodes.id, startNodeId));
        endIdx = find(strcmp(nodes.id, endNodeId));
        
        if ~isempty(startIdx)
            intersections.connectedRoads{startIdx} = [intersections.connectedRoads{startIdx}, i];
        end
        
        if ~isempty(endIdx)
            intersections.connectedRoads{endIdx} = [intersections.connectedRoads{endIdx}, i];
        end
    end
    
    % 计算每个路口的停车线位置
    for i = 1:numNodes
        type = nodes.type{i};
        connectedRoads = intersections.connectedRoads{i};
        
        % 根据路口类型设置停车线距离
        if strcmp(type, 'A') || strcmp(type, 'B')
            stopDistance = 3; % A型和B型路口停车线距离为3
        else
            % X型和T型路口需要计算交点
            stopDistance = calculateStopLineDistance(model, i, connectedRoads);
        end
        
        % 为每条连接的道路设置停车线
        stopLines = zeros(numel(connectedRoads), 1);
        for j = 1:numel(connectedRoads)
            stopLines(j) = stopDistance;
        end
        
        intersections.stopLines{i} = stopLines;
    end
end

%% 函数: 计算停车线距离
function stopDistance = calculateStopLineDistance(model, nodeIdx, connectedRoads)
    % 简化实现：对于X型和T型路口，计算停车线距离
    % 实际实现应根据需求文档中的交点计算方法
    
    % 默认停车线距离
    stopDistance = 5;
    
    % 这里应该实现根据交点计算的逻辑
    % ...
    
    return;
end

%% 函数: 设置路口方向组 - 修正版，确保对向道路共享控制
function model = setIntersectionGroups(model)
    % 为X型和T型路口设置方向组，确保对向道路在同一组
    nodes = model.nodes;
    numNodes = numel(nodes.id);
    
    % 初始化方向组
    model.xaGroup = cell(numNodes, 1);
    model.xbGroup = cell(numNodes, 1);
    model.mainGroup = cell(numNodes, 1);
    model.branchGroup = cell(numNodes, 1);
    
    for i = 1:numNodes
        type = nodes.type{i};
        connectedRoads = model.intersections.connectedRoads{i};

        % ✅ 第一步：前置调试信息
        printInfoHeader(model, i);

        dirs = calculateDirectionVectors(model, i, connectedRoads);
        n = size(dirs, 1);

        % 跳过没有足够连接道路的节点
        if numel(connectedRoads) < 3
            continue;
        end
        
        if strcmp(type, 'X') && numel(connectedRoads) >= 4
            % 计算从节点到各道路的方向向量
            dirs = calculateDirectionVectors(model, i, connectedRoads);
            
            % 识别对向道路对
            roadPairs = identifyOppositeRoads(dirs, connectedRoads);
            
            % 确保X型路口有两对对向道路
            if size(roadPairs, 1) >= 2
                % 第一对对向道路归为xa组
                model.xaGroup{i} = roadPairs(1, :);
                
                % 第二对对向道路归为xb组
                model.xbGroup{i} = roadPairs(2, :);
            else
                % 如果无法识别两对对向道路，采用原来的划分方法
                fprintf('警告: 路口 %d 无法识别两对对向道路，采用备用划分方法\n', i);
                
                % 计算夹角差值
                angles = zeros(6, 1);
                idx = 1;
                for j = 1:3
                    for k = j+1:4
                        v1 = dirs(j, :);
                        v2 = dirs(k, :);
                        dot_product = dot(v1, v2);
                        angle = acosd(min(max(dot_product, -1), 1));
                        angles(idx) = angle;
                        idx = idx + 1;
                    end
                end
                
                % 找出夹角差值最接近180度的两组道路
                [~, sortIdx] = sort(abs(angles - 180));
                
                % 前两个是xa组
                pair1 = sortIdx(1);
                [road1, road2] = getPairRoads(pair1);
                model.xaGroup{i} = [connectedRoads(road1), connectedRoads(road2)];
                
                % 剩下的是xb组
                remainingRoads = setdiff(1:4, [road1, road2]);
                model.xbGroup{i} = [connectedRoads(remainingRoads(1)), connectedRoads(remainingRoads(2))];
            end
            
        elseif strcmp(type, 'T') && numel(connectedRoads) >= 3
            % 计算从节点到各道路的方向向量
            dirs = calculateDirectionVectors(model, i, connectedRoads);
            
            % 识别对向道路对
            roadPairs = identifyOppositeRoads(dirs, connectedRoads);
            
            % 对于T型路口，一对对向道路是主干道，剩下的是支路
            if ~isempty(roadPairs)
                % 第一对对向道路归为主干道组
                model.mainGroup{i} = roadPairs(1, :);
                
                % 剩余道路归为支路组
                remainingRoads = setdiff(connectedRoads, roadPairs(1, :));
                model.branchGroup{i} = remainingRoads;
            else
                % 如果无法识别对向道路，采用原来的划分方法
                fprintf('警告: 路口 %d 无法识别对向道路，采用备用划分方法\n', i);
                
                % 计算夹角
                angles = zeros(3, 1);
                idx = 1;
                for j = 1:2
                    for k = j+1:3
                        v1 = dirs(j, :);
                        v2 = dirs(k, :);
                        dot_product = dot(v1, v2);
                        angle = acosd(min(max(dot_product, -1), 1));
                        angles(idx) = angle;
                        idx = idx + 1;
                    end
                end
                
                % 找出夹角差值最接近180度的两条道路
                [~, maxIdx] = max(abs(angles - 180));
                [road1, road2] = getPairRoads(maxIdx);
                
                % 设置主干道组
                model.mainGroup{i} = [connectedRoads(road1), connectedRoads(road2)];
                
                % 剩下的是支路
                otherRoad = setdiff(1:3, [road1, road2]);
                model.branchGroup{i} = connectedRoads(otherRoad);
               
            end
        end
            % ✅ 第二步：分组信息（分组完成后再打印）
    printInfoGroups(model, i);
    end
end

%% 函数: 识别对向道路
function roadPairs = identifyOppositeRoads(dirs, connectedRoads)
    % 识别对向的道路对
    numRoads = size(dirs, 1);
    roadPairs = [];
    
    % 对每对道路，检查它们是否接近对向(夹角接近180度)
    for i = 1:numRoads-1
        for j = i+1:numRoads
            % 计算两个方向向量的夹角
            v1 = dirs(i, :);
            v2 = dirs(j, :);
            dot_product = dot(v1, v2);
            angle = acosd(min(max(dot_product, -1), 1));
            
            % 如果夹角接近180度（阈值设为150度），认为它们是对向道路
            if angle > 150
                roadPairs = [roadPairs; connectedRoads(i), connectedRoads(j)];
            end
        end
    end
    
    % 按照夹角从大到小排序（更接近180度的排在前面）
    if size(roadPairs, 1) > 1
        angles = zeros(size(roadPairs, 1), 1);
        for k = 1:size(roadPairs, 1)
            idx1 = find(connectedRoads == roadPairs(k, 1));
            idx2 = find(connectedRoads == roadPairs(k, 2));
            v1 = dirs(idx1, :);
            v2 = dirs(idx2, :);
            dot_product = dot(v1, v2);
            angles(k) = acosd(min(max(dot_product, -1), 1));
        end
        [~, sortIdx] = sort(angles, 'descend');
        roadPairs = roadPairs(sortIdx, :);
    end
end

%% 函数: 计算方向向量
function dirs = calculateDirectionVectors(model, nodeIdx, connectedRoads)
    % 计算从节点到连接道路的方向向量
    nodes = model.nodes;
    roads = model.roads;
    
    nodeX = nodes.x(nodeIdx);
    nodeY = nodes.y(nodeIdx);
    
    numRoads = numel(connectedRoads);
    dirs = zeros(numRoads, 2);
    
    for i = 1:numRoads
        roadIdx = connectedRoads(i);
        startNodeId = roads.startNode{roadIdx};
        endNodeId = roads.endNode{roadIdx};
        
        startIdx = find(strcmp(nodes.id, startNodeId));
        endIdx = find(strcmp(nodes.id, endNodeId));
        
        % 选择另一端的节点计算方向
        if startIdx == nodeIdx
            otherX = nodes.x(endIdx);
            otherY = nodes.y(endIdx);
        else
            otherX = nodes.x(startIdx);
            otherY = nodes.y(startIdx);
        end
        
        % 计算方向向量
        dx = otherX - nodeX;
        dy = otherY - nodeY;
        len = sqrt(dx^2 + dy^2);
        
        if len > 0
            dirs(i, :) = [dx/len, dy/len];
        end
    end
end

%% 函数: 获取夹角对应的道路索引
function [road1, road2] = getPairRoads(pairIdx)
    % 根据夹角索引获取对应的两条道路索引
    pairs = [
        1, 2;
        1, 3;
        1, 4;
        2, 3;
        2, 4;
        3, 4
    ];
    
    road1 = pairs(pairIdx, 1);
    road2 = pairs(pairIdx, 2);
end

%% 函数:✨路口类型/连接情况/道路分组 调试信息✨
% 前置信息
function printInfoHeader(model, nodeIdx)
    type = model.nodes.type{nodeIdx};
    connectedRoads = model.intersections.connectedRoads{nodeIdx};

    % 节点头部
    fprintf('==== 分析节点 ID %s（类型: %s）====\n', model.nodes.id{nodeIdx}, type);

    % 连接道路
    fprintf('连接道路: ');
    for j = 1:numel(connectedRoads)
        fprintf('%s  ', model.roads.id{connectedRoads(j)});
    end
    fprintf('\n');

    % 夹角
    dirs = calculateDirectionVectors(model, nodeIdx, connectedRoads);
    n = size(dirs, 1);
    for r = 1:n-1
        for c = r+1:n
            angle = acosd(dot(dirs(r,:), dirs(c,:)));
            fprintf('道路 %s vs %s → 夹角 = %.2f°\n', ...
                model.roads.id{connectedRoads(r)}, ...
                model.roads.id{connectedRoads(c)}, angle);
        end
    end
end

% 分组信息
function printInfoGroups(model, nodeIdx)
    type = model.nodes.type{nodeIdx};

    switch type
        case 'X'
            if ~isempty(model.xaGroup{nodeIdx})
                fprintf('  Xa组（主路一）: ');
                for k = 1:numel(model.xaGroup{nodeIdx})
                    fprintf('%s  ', model.roads.id{model.xaGroup{nodeIdx}(k)});
                end
                fprintf('\n');
            end
            if ~isempty(model.xbGroup{nodeIdx})
                fprintf('  Xb组（主路二）: ');
                for k = 1:numel(model.xbGroup{nodeIdx})
                    fprintf('%s  ', model.roads.id{model.xbGroup{nodeIdx}(k)});
                end
                fprintf('\n');
            end
        case 'T'
            if ~isempty(model.mainGroup{nodeIdx})
                fprintf('  主干道组: ');
                for k = 1:numel(model.mainGroup{nodeIdx})
                    fprintf('%s  ', model.roads.id{model.mainGroup{nodeIdx}(k)});
                end
                fprintf('\n');
            end
            if ~isempty(model.branchGroup{nodeIdx})
                fprintf('  支路组: ');
                for k = 1:numel(model.branchGroup{nodeIdx})
                    fprintf('%s  ', model.roads.id{model.branchGroup{nodeIdx}(k)});
                end
                fprintf('\n');
            end
        case {'A', 'B'}
            fprintf('  A/B型路口 - 默认连接，无方向分组。\n');
    end
    fprintf('\n');
end




%% 函数: 初始化信号灯
function trafficLights = initializeTrafficLights(model)
    global CONFIG
    % 初始化各路口的信号灯
    nodes = model.nodes;
    numNodes = numel(nodes.id);
    
    trafficLights = struct();
    trafficLights.nodeIndex = find(model.intersections.hasSignal);
    trafficLights.type = cell(numNodes, 1);
    trafficLights.phase = zeros(numNodes, 1);
    trafficLights.timer = rand(numNodes, 1) * CONFIG.SIGNAL_CYCLE; % 初始随机相位
    trafficLights.pedPhase = false(numNodes, 1);
    trafficLights.pedPhaseTimer = zeros(numNodes, 1);
    
    % 设置信号灯类型
    for i = 1:numNodes
        if model.intersections.hasSignal(i)
            trafficLights.type{i} = nodes.type{i};
        else
            trafficLights.type{i} = '';
        end
    end
end

%% 函数: 更新信号灯状态
function trafficLights = updateTrafficLights(trafficLights, time)
    global CONFIG
    % 更新各路口信号灯的状态
    numNodes = numel(trafficLights.nodeIndex);
    
    for i = 1:numNodes
        nodeIdx = trafficLights.nodeIndex(i);
        type = trafficLights.type{nodeIdx};
        
        % 如果正在执行行人通行相位
        if trafficLights.pedPhase(nodeIdx)
            trafficLights.pedPhaseTimer(nodeIdx) = trafficLights.pedPhaseTimer(nodeIdx) + CONFIG.TIME_STEP;
            
            % 根据路口类型设置行人相位持续时间
            if strcmp(type, 'X')
                pedDuration = 30;
            elseif strcmp(type, 'T')
                pedDuration = 20;
            else  % A型路口
                pedDuration = 10;
            end
            
            % 行人相位结束
            if trafficLights.pedPhaseTimer(nodeIdx) >= pedDuration
                trafficLights.pedPhase(nodeIdx) = false;
                trafficLights.pedPhaseTimer(nodeIdx) = 0;
                trafficLights.timer(nodeIdx) = 0;  % 重置常规相位计时器
            end
            
            continue;  % 跳过常规相位更新
        end
        
        % 更新常规相位计时器
        trafficLights.timer(nodeIdx) = trafficLights.timer(nodeIdx) + CONFIG.TIME_STEP;
        
        % 相位切换逻辑
        if trafficLights.timer(nodeIdx) >= CONFIG.SIGNAL_CYCLE
            trafficLights.timer(nodeIdx) = 0;
            
            % 根据路口类型更新相位
            if strcmp(type, 'X')
                trafficLights.phase(nodeIdx) = mod(trafficLights.phase(nodeIdx) + 1, 5);
            elseif strcmp(type, 'T')
                trafficLights.phase(nodeIdx) = mod(trafficLights.phase(nodeIdx) + 1, 3);
            else  % A型路口
                trafficLights.phase(nodeIdx) = 0;  % 只有一个相位
            end
            
            % 检查是否触发行人通行相位
            if rand < CONFIG.PED_PHASE_PROB
                trafficLights.pedPhase(nodeIdx) = true;
            end
        end
    end
end

%% 函数: 生成新车辆
function vehicles = spawnVehicles(vehicles, model, roads, time)

    global CONFIG

    lanes = model.lanes;

    % 初始化生成统计
    if ~isfield(CONFIG, 'actualCarCount')
        CONFIG.actualCarCount = 0;
        CONFIG.actualBikeCount = 0;
    end

    % 每个时间步仅尝试生成一次车辆
    if rand < CONFIG.SPAWN_RATE * CONFIG.TIME_STEP
    
        % 计算当前实际车辆比例
        currentRatio = CONFIG.actualCarCount / max(CONFIG.actualBikeCount,1);
    
        % 动态调整汽车生成概率
        if currentRatio < CONFIG.CAR_TO_BIKE_RATIO
            % 当前汽车比例不足，汽车概率更高，但仍然有机会生成自行车
            carProb = 0.8; 
        else
            % 当前汽车比例达到目标，降低汽车生成概率
            carProb = 0.2;
        end
    
        % 根据概率决定本次生成的车辆类型
        if rand < carProb
            desiredType = 'car';
        else
            desiredType = 'bike';
        end

        generated = false;

        % 先尝试生成所选车辆类型（汽车或自行车）
        attempts = 0;
        maxAttempts = numel(lanes.roadIndex);

        while ~generated && attempts < maxAttempts
            if strcmp(desiredType, 'car')
                carGenLaneIndices = find(lanes.isCarGenLane);
                if isempty(carGenLaneIndices)
                    break; % 无法生成汽车
                end
                laneIdx = carGenLaneIndices(randi(numel(carGenLaneIndices)));
                startX = lanes.startX(laneIdx);
                startY = lanes.startY(laneIdx);
            else % bike
                laneIdx = randi(numel(lanes.roadIndex));
                startX = lanes.startX(laneIdx);
                startY = lanes.startY(laneIdx);
            end

            % 检查是否可生成
            if isLaneClear(vehicles, laneIdx, startX, startY, desiredType)
                newVehicle = struct('id', numel(vehicles) + 1, ...
                                    'type', desiredType, ...
                                    'laneIdx', laneIdx, ...
                                    'x', startX, ...
                                    'y', startY, ...
                                    'direction', lanes.direction(laneIdx), ...
                                    'speed', strcmp(desiredType,'car')*5 + strcmp(desiredType,'bike')*3, ...
                                    'acceleration', 0, ...
                                    'distance', 0, ...
                                    'nextNodeIdx', [], ...
                                    'status', 'moving', ...
                                    'waitingTime', 0, ...
                                    'stopCount', 0, ...
                                    'totalTime', 0, ...
                                    'lowSpeedTime', 0, ...
                                    'length', strcmp(desiredType,'car')*CONFIG.CAR_LENGTH + strcmp(desiredType,'bike')*CONFIG.BIKE_LENGTH, ...
                                    'width', strcmp(desiredType,'car')*CONFIG.CAR_WIDTH + strcmp(desiredType,'bike')*CONFIG.BIKE_WIDTH, ...
                                    'maxSpeed', strcmp(desiredType,'car')*CONFIG.CAR_MAX_SPEED + strcmp(desiredType,'bike')*CONFIG.BIKE_MAX_SPEED, ...
                                    'maxAccel', strcmp(desiredType,'car')*CONFIG.CAR_MAX_ACCEL + strcmp(desiredType,'bike')*CONFIG.BIKE_MAX_ACCEL, ...
                                    'maxDecel', strcmp(desiredType,'car')*CONFIG.CAR_MAX_DECEL + strcmp(desiredType,'bike')*CONFIG.BIKE_MAX_DECEL);
                vehicles = [vehicles, newVehicle];

                if strcmp(desiredType, 'car')
                    CONFIG.actualCarCount = CONFIG.actualCarCount + 1;
                else
                    CONFIG.actualBikeCount = CONFIG.actualBikeCount + 1;
                end
                generated = true;
            end
            attempts = attempts + 1;
        end

        % 如果尝试失败，再尝试另一类型
        if ~generated
            otherType = 'bike';
            if strcmp(desiredType, 'bike')
                otherType = 'car';
            end

            attempts = 0;
            while ~generated && attempts < maxAttempts
                if strcmp(otherType, 'car')
                    carGenLaneIndices = find(lanes.isCarGenLane);
                    if isempty(carGenLaneIndices)
                        break; % 无法生成汽车
                    end
                    laneIdx = carGenLaneIndices(randi(numel(carGenLaneIndices)));
                else
                    laneIdx = randi(numel(lanes.roadIndex));
                end
                startX = lanes.startX(laneIdx);
                startY = lanes.startY(laneIdx);

                if isLaneClear(vehicles, laneIdx, startX, startY, otherType)
                    newVehicle = struct('id', numel(vehicles) + 1, ...
                                        'type', otherType, ...
                                        'laneIdx', laneIdx, ...
                                        'x', startX, ...
                                        'y', startY, ...
                                        'direction', lanes.direction(laneIdx), ...
                                        'speed', strcmp(otherType,'car')*3 + strcmp(otherType,'bike')*2, ...
                                        'acceleration', 0, ...
                                        'distance', 0, ...
                                        'nextNodeIdx', [], ...
                                        'status', 'moving', ...
                                        'waitingTime', 0, ...
                                        'stopCount', 0, ...
                                        'totalTime', 0, ...
                                        'lowSpeedTime', 0, ...
                                        'length', strcmp(otherType,'car')*CONFIG.CAR_LENGTH + strcmp(otherType,'bike')*CONFIG.BIKE_LENGTH, ...
                                        'width', strcmp(otherType,'car')*CONFIG.CAR_WIDTH + strcmp(otherType,'bike')*CONFIG.BIKE_WIDTH, ...
                                        'maxSpeed', strcmp(otherType,'car')*CONFIG.CAR_MAX_SPEED + strcmp(otherType,'bike')*CONFIG.BIKE_MAX_SPEED, ...
                                        'maxAccel', strcmp(otherType,'car')*CONFIG.CAR_MAX_ACCEL + strcmp(otherType,'bike')*CONFIG.BIKE_MAX_ACCEL, ...
                                        'maxDecel', strcmp(otherType,'car')*CONFIG.CAR_MAX_DECEL + strcmp(otherType,'bike')*CONFIG.BIKE_MAX_DECEL);
                    vehicles = [vehicles, newVehicle];

                    if strcmp(otherType, 'car')
                        CONFIG.actualCarCount = CONFIG.actualCarCount + 1;
                    else
                        CONFIG.actualBikeCount = CONFIG.actualBikeCount + 1;
                    end
                    generated = true;
                end
                attempts = attempts + 1;
            end
        end
    end
end



%% 函数: 检查车道是否有足够空间生成车辆 - 改进版
function clear = isLaneClear(vehicles, laneIdx, x, y, type)
    global CONFIG
    % 检查在指定位置是否有足够空间生成新车辆
    
    % 根据车辆类型确定长度和安全距离
    if strcmp(type, 'car')
        vehicleLen = CONFIG.CAR_LENGTH;
        safeDistance = CONFIG.SAFE_DISTANCE_MIN + 1; % 增加一点额外安全距离
    else
        vehicleLen = CONFIG.BIKE_LENGTH;
        safeDistance = CONFIG.SAFE_DISTANCE_MIN; % 对自行车可以使用标准安全距离
    end
    
    % 检查该车道上的所有车辆
    for i = 1:numel(vehicles)
        if vehicles(i).laneIdx == laneIdx
            % 计算到起点的距离
            dx = vehicles(i).x - x;
            dy = vehicles(i).y - y;
            distance = sqrt(dx^2 + dy^2);
            
            % 如果距离小于安全距离，则无法生成
            requiredDistance = CONFIG.SAFE_DISTANCE_MIN + vehicleLen/2 + vehicles(i).length/2;
            if distance < requiredDistance
                clear = false;
                return;
            end
        end
    end
    
    % 如果车道上没有现有车辆，或所有车辆都保持了足够距离，则允许生成
    clear = true;
end

%% 函数: 更新车辆位置 - 修复版，确保车辆不会离开道路
function [vehicles, stats] = updateVehicles(vehicles, model, trafficLights, stats, time, deltaTime)
    global CONFIG
    % 更新所有车辆的位置和状态
    lanes = model.lanes;
    nodes = model.nodes;
    roads = model.roads;
    
    % 如果没有车辆，直接返回
    if isempty(vehicles)
        return;
    end
    
    % 创建一个标记数组，用于记录哪些车辆需要被移除
    removeFlag = false(size(vehicles));
    
    % 更新每个车辆
    for i = 1:numel(vehicles)
        % 更新车辆总时间
        vehicles(i).totalTime = vehicles(i).totalTime + deltaTime;
        
        % ✅ 添加这行防止 canPassIntersection 未定义报错
        canPassIntersection = false;

        % 获取当前车道信息
        laneIdx = vehicles(i).laneIdx;
        roadIdx = lanes.roadIndex(laneIdx);
        laneDirection = lanes.direction(laneIdx);
        
        % 获取车道的起点和终点坐标
        laneStartX = lanes.startX(laneIdx);
        laneStartY = lanes.startY(laneIdx);
        laneEndX = lanes.endX(laneIdx);
        laneEndY = lanes.endY(laneIdx);
        
        % 计算车道总长度
        laneTotalLength = sqrt((laneEndX - laneStartX)^2 + (laneEndY - laneStartY)^2);
        
        % 计算车道方向向量（归一化）
        if laneTotalLength > 0
            laneDirX = (laneEndX - laneStartX) / laneTotalLength;
            laneDirY = (laneEndY - laneStartY) / laneTotalLength;
        else
            % 如果车道长度为0，使用默认值
            laneDirX = 1;
            laneDirY = 0;
        end
        
        % 计算车辆到车道终点的距离
        dx = laneEndX - vehicles(i).x;
        dy = laneEndY - vehicles(i).y;
        distanceToEnd = sqrt(dx^2 + dy^2);
        
        % 计算车辆到车道起点的距离
        dxStart = vehicles(i).x - laneStartX;
        dyStart = vehicles(i).y - laneStartY;
        distanceFromStart = sqrt(dxStart^2 + dyStart^2);
        
        % 计算车辆在车道上的投影长度
        projectionLength = dxStart * laneDirX + dyStart * laneDirY;
        
        % 计算垂直于车道的分量 - 用于检测车辆是否偏离车道
        perpDistance = abs(dxStart * (-laneDirY) + dyStart * laneDirX);
        
        % 如果车辆偏离车道太远，校正其位置
        if perpDistance > CONFIG.LANE_WIDTH / 2 - vehicles(i).width / 2
            % 计算校正后的位置
            correctionFactor = (CONFIG.LANE_WIDTH / 2 - vehicles(i).width / 2) / perpDistance;
            perpX = -laneDirY; % 垂直于车道方向的单位向量
            perpY = laneDirX;
            
            %计算车辆到车道中心线的垂直偏移方向
            side = sign(dxStart * perpX + dyStart * perpY);
            
            %校正车辆位置
            vehicles(i).x = laneStartX + projectionLength * laneDirX + side * correctionFactor * perpDistance * perpX;
            vehicles(i).y = laneStartY + projectionLength * laneDirY + side * correctionFactor * perpDistance * perpY;
            
            %重新计算到终点的距离
            dx = laneEndX - vehicles(i).x;
            dy = laneEndY - vehicles(i).y;
            distanceToEnd = sqrt(dx^2 + dy^2);
        end

        %% 跟车
        % 确定车辆是否接近路口
        approachingIntersection = distanceToEnd < 10;  % 10米内视为接近路口
        
        % 获取前方车辆
        leadVehicle = findLeadVehicle(vehicles, i, laneIdx);
              
        % ✅ 安全距离计算 + 提前反应距离
        reactionBuffer = 2;  % 反应缓冲距离（可调）
        
        if ~isempty(leadVehicle)
            centerDist = sqrt((leadVehicle.x - vehicles(i).x)^2 + (leadVehicle.y - vehicles(i).y)^2);
            leadDistance = centerDist - vehicles(i).length/2 - leadVehicle.length/2;
            safeDistance = CONFIG.SAFE_DISTANCE_MIN + vehicles(i).length/2 + leadVehicle.length/2 + vehicles(i).speed * CONFIG.HEADWAY_TIME;
        else
            leadDistance = inf;
            safeDistance = CONFIG.SAFE_DISTANCE_MIN + vehicles(i).length/2 + vehicles(i).speed * CONFIG.HEADWAY_TIME;
        end

        % 限制最大安全距离，防止过度保守
        safeDistance = min(safeDistance, 20);
        
        % 确定终点节点
        if laneDirection == 1
            endNodeId = roads.endNode{roadIdx};
        else
            endNodeId = roads.startNode{roadIdx};
        end
        endNodeIdx = find(strcmp(nodes.id, endNodeId));
        
        % 决定车辆加速度
        targetAccel = 0;

        
        if approachingIntersection
            % 检查是否可以通过路口
            canPassIntersection = isAllowedToGo(vehicles(i), model, trafficLights, endNodeIdx);
            
            if canPassIntersection
                % 可以通过路口
                if leadDistance < safeDistance
                    % 前方车辆距离过近，需要减速
                    targetAccel = computeSafeDecel(leadDistance, safeDistance, vehicles(i).maxDecel, reactionBuffer);
                    
                    if vehicles(i).speed < 0.1 && leadDistance < CONFIG.SAFE_DISTANCE_MIN
                        % 检查上一时刻是否仍在移动状态且速度非零
                        if ~strcmp(vehicles(i).status, 'stopped') && vehicles(i).speed > 0.01
                            vehicles(i).stopCount = vehicles(i).stopCount + 1;
                        end
                        vehicles(i).status = 'stopped';
                        vehicles(i).speed = 0;
                        vehicles(i).acceleration = 0;
                      
                         % 【新增调试信息】
                       fprintf('车辆 %d 在位置(x=%.2f, y=%.2f)因安全距离不足而停车\n', vehicles(i).id, vehicles(i).x, vehicles(i).y);

                    end
                else
                    % 可以通过路口且前方没有车辆阻挡
                    targetAccel = vehicles(i).maxAccel * (1 - vehicles(i).speed / vehicles(i).maxSpeed);
                    vehicles(i).status = 'moving';
                end
            else
                % 不能通过路口
               if distanceToEnd < 2  % 接近停止线
                    if vehicles(i).speed < 0.1
                        % 只有首次停车时增加统计次数
                        if ~strcmp(vehicles(i).status, 'stopped')
                            vehicles(i).stopCount = vehicles(i).stopCount + 1;
                            if strcmp(vehicles(i).type, 'car')
                                stats.carStopCount = stats.carStopCount + 1;
                            elseif strcmp(vehicles(i).type, 'bike')
                                stats.bikeStopCount = stats.bikeStopCount + 1;
                            end
                        end
                        vehicles(i).status = 'stopped';
                        vehicles(i).speed = 0;
                        vehicles(i).acceleration = 0;
                    else
                        vehicles(i).status = 'waiting';
                        targetAccel = -vehicles(i).maxDecel * 0.8;  % 适当减速
                    end
                    vehicles(i).waitingTime = vehicles(i).waitingTime + deltaTime;
                
                    % 【新增调试信息】
                    fprintf('车辆 %d 在节点%d处等待信号灯 (当前相位=%d)\n', vehicles(i).id, endNodeIdx, trafficLights.phase(endNodeIdx));
                else
                    % 接近路口但还未到停止线
                    if leadDistance < safeDistance
                        % 前方车辆距离过近
                        targetAccel = computeSafeDecel(leadDistance, safeDistance, vehicles(i).maxDecel, reactionBuffer);
                    else
                        % 减速接近路口
                        targetAccel = -vehicles(i).maxDecel * min(1, distanceToEnd / 10);
                    end
                    vehicles(i).status = 'approaching';
                end
            end
            
            % 到达终点，需要选择下一个节点
            if distanceToEnd < 0.5
                % 获取当前节点类型
                nodeType = nodes.type{endNodeIdx};
                
                % 补充定义当前节点ID 
                if laneDirection == 1
                    currentNodeId = roads.endNode{roadIdx};
                else
                    currentNodeId = roads.startNode{roadIdx};
                end
                
                % 获取可能的下一个节点
                possibleNextNodes = lanes.nextNodes{laneIdx};
                
                if isempty(possibleNextNodes)
                    % 若没有明确指定，则移除
                    fprintf('车辆 %d 到达终点，没有指定下一个节点，将被移除\n', vehicles(i).id);
                    vehicles(i).status = 'removed';
                    removeFlag(i) = true;
                else
                    % 对于X和T型路口，根据当前行驶方向选择合适的下一个节点
                    if strcmp(nodeType, 'X') || strcmp(nodeType, 'T')
                        % 过滤合法的下一个节点
                        validNextNodes = {};
                        for j = 1:numel(possibleNextNodes)
                            nextNodeId = possibleNextNodes{j};
                            if findNextLane(model, currentNodeId, nextNodeId, vehicles(i).type)
                                validNextNodes{end+1} = nextNodeId;
                            end
                        end
                        
                        if isempty(validNextNodes)
                            fprintf('车辆 %d 没有找到有效的下一个节点，将被移除\n', vehicles(i).id);
                            vehicles(i).status = 'removed';
                            removeFlag(i) = true;
                            continue;
                        end
                        
                        % 随机选择一个有效的下一个节点
                        nextNodeId = validNextNodes{randi(numel(validNextNodes))};
                    else
                        % A和B型路口直接随机选择
                        nextNodeId = possibleNextNodes{randi(numel(possibleNextNodes))};
                    end
                    
                    % 找到连接该节点的下一车道
                    nextLaneIdx = findNextLane(model, currentNodeId, nextNodeId, vehicles(i).type);
                    
                    if isempty(nextLaneIdx)
                        % 若未找到连接车道，移除车辆
                        fprintf('车辆 %d 无法从节点 %s 到节点 %s，将被移除\n', vehicles(i).id, currentNodeId, nextNodeId);
                        vehicles(i).status = 'removed';
                        removeFlag(i) = true;
                    else
                        % 严格设置在新车道起点
                        vehicles(i).laneIdx = nextLaneIdx;
                        
                        % 确保车辆精确放置在新车道的起点
                        vehicles(i).x = lanes.startX(nextLaneIdx);
                        vehicles(i).y = lanes.startY(nextLaneIdx);
                        vehicles(i).direction = lanes.direction(nextLaneIdx);
                        vehicles(i).distance = 0; % 重置距离
                        vehicles(i).nextNodeIdx = [];
                        
                        % 输出车辆转向信息便于调试
                        if CONFIG.TIME_STEP > 0.09
                            fprintf('车辆 %d 从节点 %s 转向节点 %s，使用车道 %d\n', vehicles(i).id, currentNodeId, nextNodeId, nextLaneIdx);
                        end
                    end
                end
            
                % 对于B型节点，车辆离开路网
                if strcmp(nodeType, 'B')
                    % 更新统计数据
                    if strcmp(vehicles(i).type, 'car')
                        stats.carCount = stats.carCount + 1;
                        stats.carLowSpeedTime = stats.carLowSpeedTime + vehicles(i).lowSpeedTime;
                        stats.carTotalTime = stats.carTotalTime + vehicles(i).totalTime;
                        stats.carWaitingTime = stats.carWaitingTime + vehicles(i).waitingTime;
                        stats.carStopCount = stats.carStopCount + vehicles(i).stopCount;
                    else
                        stats.bikeCount = stats.bikeCount + 1;
                        stats.bikeLowSpeedTime = stats.bikeLowSpeedTime + vehicles(i).lowSpeedTime;
                        stats.bikeTotalTime = stats.bikeTotalTime + vehicles(i).totalTime;
                        stats.bikeWaitingTime = stats.bikeWaitingTime + vehicles(i).waitingTime;
                        stats.bikeStopCount = stats.bikeStopCount + vehicles(i).stopCount;
                    end
                    
                    % 标记车辆移除
                    vehicles(i).status = 'removed';
                    removeFlag(i) = true;
                    continue;
                end
                
                % 对于Bike，有概率离开路网
                if strcmp(vehicles(i).type, 'bike') && rand < 0.05
                    % 更新统计数据
                    stats.bikeCount = stats.bikeCount + 1;
                    stats.bikeLowSpeedTime = stats.bikeLowSpeedTime + vehicles(i).lowSpeedTime;
                    stats.bikeTotalTime = stats.bikeTotalTime + vehicles(i).totalTime;
                    stats.bikeWaitingTime = stats.bikeWaitingTime + vehicles(i).waitingTime;
                    stats.bikeStopCount = stats.bikeStopCount + vehicles(i).stopCount;
                    
                    % 标记车辆移除
                    vehicles(i).status = 'removed';
                    removeFlag(i) = true;
                    continue;
                end
            end
        else
            % 在道路中段
            if leadDistance < safeDistance
                % 前方车辆距离过近，需要减速
                targetAccel = computeSafeDecel(leadDistance, safeDistance, vehicles(i).maxDecel, reactionBuffer);
                if vehicles(i).speed < 0.1 && leadDistance < CONFIG.SAFE_DISTANCE_MIN
                    vehicles(i).status = 'stopped';
                    if vehicles(i).speed > 0
                        vehicles(i).stopCount = vehicles(i).stopCount + 1;
                    end
                    targetAccel = 0;
                    vehicles(i).speed = 0;
                end
            else
                % 正常行驶，加速到最大速度
                targetAccel = vehicles(i).maxAccel * (1 - vehicles(i).speed / vehicles(i).maxSpeed);
                vehicles(i).status = 'moving';
            end
        end
        
        % 应用加速度限制
        vehicles(i).acceleration = max(-vehicles(i).maxDecel, min(vehicles(i).maxAccel, targetAccel));
        
        % 更新速度
        vehicles(i).speed = max(0, vehicles(i).speed + vehicles(i).acceleration * deltaTime);

        % ✅ 仅在“允许通行 + 正常行驶状态”时，设置最低速度（避免滑行重叠）
        if canPassIntersection && strcmp(vehicles(i).status, 'moving')
            vehicles(i).speed = max(vehicles(i).speed, 0.2);  % 最低速度阈值（可调）
        end
        
        % 精确定义“低速”状态：仅在速度 > 0 且 < maxSpeed × 比例时计入
        lowSpeedThreshold = vehicles(i).maxSpeed * CONFIG.LOW_SPEED_RATIO;
        if vehicles(i).speed > 0 && vehicles(i).speed < lowSpeedThreshold
            vehicles(i).lowSpeedTime = vehicles(i).lowSpeedTime + deltaTime;
        end
                
        % 更新位置 - 改进版，确保车辆沿着车道中心线移动
        if vehicles(i).speed > 0
            % 计算当前位置到起点的向量在车道方向上的投影
            currentProjLength = projectionLength + vehicles(i).speed * deltaTime;
            
            % 确保投影长度不超过车道总长度
            currentProjLength = min(currentProjLength, laneTotalLength);
            
            % 计算新位置 - 使用精确的车道方向
            vehicles(i).x = laneStartX + currentProjLength * laneDirX;
            vehicles(i).y = laneStartY + currentProjLength * laneDirY;
            vehicles(i).distance = currentProjLength;  % 更新距离
        end
    end
    
    % 移除标记为removed的车辆
    if ~isempty(vehicles)
        vehicles = vehicles(~removeFlag);
    end
end

%% 函数: 安全减速计算
function accel = computeSafeDecel(leadDistance, safeDistance, maxDecel, buffer)
    totalRequired = safeDistance + buffer;
    gap = totalRequired - leadDistance;

    if gap <= 0
        accel = 0;
    else
        ratio = min(1, gap / totalRequired);
        accel = -maxDecel * ratio;
    end
end


%% 函数: 查找前方车辆
function leadVehicle = findLeadVehicle(vehicles, vehicleIdx, laneIdx)
    % 找出当前车道上的前方车辆
    vehicle = vehicles(vehicleIdx);
    minDistance = inf;
    leadVehicle = [];
    
    for i = 1:numel(vehicles)
        if i ~= vehicleIdx && vehicles(i).laneIdx == laneIdx
            % 计算两车之间的距离
            dx = vehicles(i).x - vehicle.x;
            dy = vehicles(i).y - vehicle.y;
            distance = sqrt(dx^2 + dy^2);
            
            % 检查是否在前方
            dirX = (dx > 0 && vehicle.direction > 0) || (dx < 0 && vehicle.direction < 0);
            dirY = (dy > 0 && vehicle.direction > 0) || (dy < 0 && vehicle.direction < 0);
            
            if (dirX || dirY) && distance < minDistance
                minDistance = distance;
                leadVehicle = vehicles(i);
            end
        end
    end
end

%% 函数: 查找下一个车道
function nextLaneIdx = findNextLane(model, currentNodeId, nextNodeId, vehicleType)
    % 找出通往下一个节点的可用车道
    lanes = model.lanes;
    roads = model.roads;
    nodes = model.nodes;
    
    % 检查节点是否存在
    currentNodeExists = any(strcmp(nodes.id, currentNodeId));
    nextNodeExists = any(strcmp(nodes.id, nextNodeId));
    
    if ~currentNodeExists || ~nextNodeExists
        % 如果节点不存在，无法找到下一条车道
        fprintf('警告: 节点不存在! 当前节点: %s, 下一节点: %s\n', currentNodeId, nextNodeId);
        nextLaneIdx = [];
        return;
    end
    
    % 查找连接当前节点和下一个节点的道路
    roadIdx = [];
    for i = 1:numel(roads.id)
        if (strcmp(roads.startNode{i}, currentNodeId) && strcmp(roads.endNode{i}, nextNodeId)) || ...
           (strcmp(roads.startNode{i}, nextNodeId) && strcmp(roads.endNode{i}, currentNodeId))
            roadIdx = i;
            break;
        end
    end
    
    if isempty(roadIdx)
        fprintf('警告: 没有找到从节点 %s 到节点 %s 的道路\n', currentNodeId, nextNodeId);
        nextLaneIdx = [];
        return;
    end
    
    % 确定车道方向
    if strcmp(roads.startNode{roadIdx}, currentNodeId)
        direction = 1;  % 小到大方向
        possibleLanes = roads.smallToBigLanes{roadIdx};
    else
        direction = -1; % 大到小方向
        possibleLanes = roads.bigToSmallLanes{roadIdx};
    end
    
    if isempty(possibleLanes)
        fprintf('警告: 没有找到从节点 %s 到节点 %s 的车道\n', currentNodeId, nextNodeId);
        nextLaneIdx = [];
        return;
    end
    
    % 查找所有符合条件的车道
    validLanes = [];
    for i = 1:numel(lanes.roadIndex)
        if lanes.roadIndex(i) == roadIdx && lanes.direction(i) == direction
            % 检查车道是否适合当前车辆类型
            % 自行车可以走任何车道，汽车只能走汽车车道
            if strcmp(vehicleType, 'bike') || ~strcmp(vehicleType, 'bike')
                validLanes = [validLanes, i];
            end
        end
    end
    
    if isempty(validLanes)
        fprintf('警告: 没有找到从节点 %s 到节点 %s 的有效车道\n', currentNodeId, nextNodeId);
        nextLaneIdx = [];
    else
        % 随机选择一条有效车道
        nextLaneIdx = validLanes(randi(numel(validLanes)));
    end
end


%% 函数: 检查是否可以通过路口
function allowed = isAllowedToGo(vehicle, model, trafficLights, nodeIdx)
    % 检查车辆是否可以通过路口（信号灯和让行规则）
    
    % 首先检查该节点是否有信号灯
    if ~model.intersections.hasSignal(nodeIdx)
        % 如果没有信号灯，使用默认的让行规则
        allowed = true;
        return;
    end
    
    % 检查行人相位
    if trafficLights.pedPhase(nodeIdx)
        allowed = false;
        return;
    end
    
    % 获取路口类型和相位
    type = trafficLights.type{nodeIdx};
    phase = trafficLights.phase(nodeIdx);
    
    % 获取车辆所在的车道和道路
    laneIdx = vehicle.laneIdx;
    roadIdx = model.lanes.roadIndex(laneIdx);
    
    % 根据路口类型和当前相位判断
    if strcmp(type, 'X')
        % X型路口相位逻辑
        % 相位说明:
        % 0: 全红
        % 1: xa组直行和左转
        % 2: xb组直行和左转
        % 3: xa组右转
        % 4: xb组右转
        
        % 找出车辆所在道路是否属于xa组或xb组
        isXaGroup = ismember(roadIdx, model.xaGroup{nodeIdx});
        isXbGroup = ismember(roadIdx, model.xbGroup{nodeIdx});
        
        % 判断是直行、左转还是右转
        % 简化处理：随机分配动作，实际应基于下一个节点位置计算
        action = randi(3);  % 1=直行, 2=左转, 3=右转
        
        if phase == 0
            % 全红相位，所有车辆停止
            allowed = false;
        elseif isXaGroup
            if (phase == 1 && (action == 1 || action == 2)) || (phase == 3 && action == 3)
                allowed = true;
            else
                allowed = false;
            end
        elseif isXbGroup
            if (phase == 2 && (action == 1 || action == 2)) || (phase == 4 && action == 3)
                allowed = true;
            else
                allowed = false;
            end
        else
            % 如果不属于任何组，默认不允许通行
            allowed = false;
        end
        
    elseif strcmp(type, 'T')
        % T型路口相位逻辑
        % 相位说明:
        % 0: 支路转向
        % 1: 主干道直行和左转
        % 2: 主干道直行和右转
        
        % 找出车辆所在道路是否属于main组或branch组
        isMainGroup = ismember(roadIdx, model.mainGroup{nodeIdx});
        isBranchGroup = ismember(roadIdx, model.branchGroup{nodeIdx});
        
        % 简化：随机分配直行/左转/右转
        action = randi(3);  % 1=直行, 2=左转, 3=右转
        
        if isMainGroup
            if (phase == 1 && (action == 1 || action == 2)) || ...
               (phase == 2 && (action == 1 || action == 3))
                allowed = true;
            else
                allowed = false;
            end
        elseif isBranchGroup
            if phase == 0
                allowed = true;
            else
                allowed = false;
            end
        else
            % 如果不属于任何组，默认不允许通行
            allowed = false;
        end
        
    else  % A型路口
        % A型路口只有一个相位，总是允许通行
        allowed = true;
    end
end

%% 函数: 更新统计数据
function stats = updateStatistics(vehicles, model, roads, stats, time)
    % 更新各项统计数据
    
    % 计算当前时间点的统计索引
    timeIdx = ceil(time / 2);
    if timeIdx < 1
        timeIdx = 1;
    end
    
    % 计算各道路的承载量指数
    numRoads = numel(roads.id);
    roadCapacity = zeros(numRoads, 1);
    
    for i = 1:numRoads
        % 获取该道路的所有车道
        roadLanes = find([model.lanes.roadIndex] == i);
        
        if isempty(roadLanes)
            continue;
        end
        
        % 计算总车道长度
        totalLaneLength = model.roadLength(i) * roads.laneCount(i);
        
        % 计算该道路上的车辆总长度
        vehicleLength = 0;
        for j = 1:numel(vehicles)
            if ismember(vehicles(j).laneIdx, roadLanes)
                vehicleLength = vehicleLength + vehicles(j).length;
            end
        end
        
        % 计算承载量指数
        if totalLaneLength > 0
            roadCapacity(i) = vehicleLength / totalLaneLength;
        end
    end
    
    % 保存道路承载量数据
    if timeIdx <= size(stats.roadCapacity, 2)
        stats.roadCapacity(:, timeIdx) = roadCapacity;
    end
    
    % 计算平均承载量和方差
    avgCapacity = mean(roadCapacity);
    varCapacity = var(roadCapacity);
    
    % 打印当前统计数据
    fprintf('时间 %.1f秒:\n', time);
    fprintf('  道路平均承载量: %.2f\n', avgCapacity);
    fprintf('  道路承载量方差: %.2f\n', varCapacity);
    
    % 计算车辆统计数据
    if stats.carCount > 0
        carActiveTime = stats.carTotalTime - stats.carWaitingTime;
        carLowSpeedRatio = stats.carLowSpeedTime / max(carActiveTime, 1e-6);  % 防止除0

        carAvgWaitTime = stats.carWaitingTime / stats.carCount;

        carAvgStopCount = stats.carStopCount / stats.carCount;
        
        fprintf('  汽车低速时间占比: %.2f%%\n', carLowSpeedRatio * 100);
        fprintf('  汽车平均等待时间: %.2f秒\n', carAvgWaitTime);
        fprintf('  汽车平均停车次数: %.2f次\n', carAvgStopCount);
    end
    
    if stats.bikeCount > 0
        bikeActiveTime = stats.bikeTotalTime - stats.bikeWaitingTime;
        bikeLowSpeedRatio = stats.bikeLowSpeedTime / max(bikeActiveTime, 1e-6);

        bikeAvgWaitTime = stats.bikeWaitingTime / stats.bikeCount;
        bikeAvgStopCount = stats.bikeStopCount / stats.bikeCount;
        
        fprintf('  自行车低速时间占比: %.2f%%\n', bikeLowSpeedRatio * 100);
        fprintf('  自行车平均等待时间: %.2f秒\n', bikeAvgWaitTime);
        fprintf('  自行车平均停车次数: %.2f次\n', bikeAvgStopCount);
    end
    
    if ~isempty(vehicles)
        fprintf('  当前车辆数: %d (汽车: %d, 自行车: %d)\n', ...
            numel(vehicles), ...
            sum(strcmp({vehicles.type}, 'car')), ...
            sum(strcmp({vehicles.type}, 'bike')));
    else
        fprintf('  当前车辆数: 0 (汽车: 0, 自行车: 0)\n');
    end
end

%% 函数: 初始化可视化
function figHandle = initializeVisualization(model)
    global CONFIG
    % 创建图形窗口和初始化可视化
    figHandle = figure('Name', '交通仿真', 'Position', [100, 100, 1200, 800]);
    
    % 创建子图
    subplot(1, 1, 1);
    hold on;
    axis equal;
    title('交通网络仿真');
    
    % 绘制节点
    nodes = model.nodes;
    plot(nodes.x, nodes.y, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    % 显示节点编号
    for i = 1:numel(nodes.id)
        text(nodes.x(i) - 25, nodes.y(i) + 5, ...
             sprintf('N%s', nodes.id{i}), ...
             'FontSize', 8, 'Color', [0.4 0.4 0.4], ...
             'FontWeight', 'bold', ...
             'HorizontalAlignment', 'left', ...
             'VerticalAlignment', 'bottom');
    end
    % 绘制道路和车道
    roads = model.roads;
    for i = 1:numel(roads.id)
        % 获取道路起点和终点节点
        startNodeId = roads.startNode{i};
        endNodeId = roads.endNode{i};
        
        startIdx = find(strcmp(nodes.id, startNodeId));
        endIdx = find(strcmp(nodes.id, endNodeId));
        
        if isempty(startIdx) || isempty(endIdx)
            continue;
        end
        
        % 道路起点和终点
        startX = nodes.x(startIdx);
        startY = nodes.y(startIdx);
        endX = nodes.x(endIdx);
        endY = nodes.y(endIdx);
        
        % 道路方向向量
        dx = endX - startX;
        dy = endY - startY;
        roadLen = sqrt(dx^2 + dy^2);
        dirX = dx / roadLen;
        dirY = dy / roadLen;
        
        % 垂直于道路方向的向量
        perpX = -dirY;
        perpY = dirX;
        
        % 道路宽度（基于车道数）
        roadWidth = roads.laneCount(i) * CONFIG.LANE_WIDTH;
        
        % 绘制道路边界
        leftX = [startX + perpX * roadWidth/2, endX + perpX * roadWidth/2];
        leftY = [startY + perpY * roadWidth/2, endY + perpY * roadWidth/2];
        rightX = [startX - perpX * roadWidth/2, endX - perpX * roadWidth/2];
        rightY = [startY - perpY * roadWidth/2, endY - perpY * roadWidth/2];
        
        plot(leftX, leftY, 'k-', 'LineWidth', 1.5);
        plot(rightX, rightY, 'k-', 'LineWidth', 1.5);
        
        % 绘制车道分割线
        for j = 1:(roads.laneCount(i) - 1)
            offset = (j - roads.laneCount(i)/2) * CONFIG.LANE_WIDTH;
            laneX = [startX + perpX * offset, endX + perpX * offset];
            laneY = [startY + perpY * offset, endY + perpY * offset];
            
            plot(laneX, laneY, 'k--', 'LineWidth', 0.5);
        end
    end
    
    % 绘制路口线
    for i = 1:numel(model.intersections.nodeIndex)
        nodeIdx = model.intersections.nodeIndex(i);
        nodeType = model.nodes.type{nodeIdx};
        connectedRoads = model.intersections.connectedRoads{i};
        stopLines = model.intersections.stopLines{i};
        
        for j = 1:numel(connectedRoads)
            roadIdx = connectedRoads(j);
            stopDist = stopLines(j);
            
            % 获取道路信息
            startNodeId = roads.startNode{roadIdx};
            endNodeId = roads.endNode{roadIdx};
            
            startIdx = find(strcmp(nodes.id, startNodeId));
            endIdx = find(strcmp(nodes.id, endNodeId));
            
            if isempty(startIdx) || isempty(endIdx)
                continue;
            end
            
            % 确定节点是路的起点还是终点
            if nodeIdx == startIdx
                otherIdx = endIdx;
                isStart = true;
            else
                otherIdx = startIdx;
                isStart = false;
            end
            
            % 计算方向向量
            dirX = nodes.x(otherIdx) - nodes.x(nodeIdx);
            dirY = nodes.y(otherIdx) - nodes.y(nodeIdx);
            dist = sqrt(dirX^2 + dirY^2);
            
            if dist > 0
                dirX = dirX / dist;
                dirY = dirY / dist;
                
                % 计算停车线的位置
                stopX = nodes.x(nodeIdx) + dirX * stopDist;
                stopY = nodes.y(nodeIdx) + dirY * stopDist;
                
                % 垂直于道路方向的向量
                perpX = -dirY;
                perpY = dirX;
                
                % 道路宽度
                roadWidth = roads.laneCount(roadIdx) * CONFIG.LANE_WIDTH;
                
                % 绘制停车线
                line1X = stopX + perpX * roadWidth/2;
                line1Y = stopY + perpY * roadWidth/2;
                line2X = stopX - perpX * roadWidth/2;
                line2Y = stopY - perpY * roadWidth/2;
                
                plot([line1X, line2X], [line1Y, line2Y], 'r-', 'LineWidth', 2);
            end
        end
    end
    
    % 设置图形范围
    minX = min(nodes.x) - 20;
    maxX = max(nodes.x) + 20;
    minY = min(nodes.y) - 20;
    maxY = max(nodes.y) + 20;
    
    axis([minX, maxX, minY, maxY]);
    
    % 创建图例
    % ✅ 替换图例绑定（真实图例绑定 + 英文图例）
    hNodeLegend      = plot(nan, nan, 'ko', 'MarkerFaceColor', 'k');
    hBorderLegend    = plot(nan, nan, 'k-', 'LineWidth', 1.5);
    hDividerLegend   = plot(nan, nan, 'k--', 'LineWidth', 0.5);
    hStopLegend      = plot(nan, nan, 'r-', 'LineWidth', 2);
    
    hLegend = legend([hNodeLegend, hBorderLegend, hDividerLegend, hStopLegend, ], ...
           {'Node', 'Road Border', 'Lane Divider', 'Stop Line', }, ...
           'Location', 'northeast');
    hLegend.AutoUpdate = 'off';

    
    % 添加统计数据文本
    annotation('textbox', [0.01, 0.01, 0.3, 0.15], 'String', '', 'EdgeColor', 'none', 'Tag', 'statsText');
    
    drawnow;
end

%% 函数: 绘制车辆 - 独立函数，负责精确绘制车辆
function drawVehicles(vehicles, lanes)
    % 绘制所有车辆，确保精确显示在车道上
    
    % 清除之前的车辆绘图
    vehicleHandles = findobj(gca, 'Tag', 'vehicle');
    if ~isempty(vehicleHandles)
        delete(vehicleHandles);
    end
    
    % 遍历所有车辆
    for i = 1:numel(vehicles)
        vehicle = vehicles(i);
        laneIdx = vehicle.laneIdx;
        
        % 根据车辆类型设置颜色和大小
        if strcmp(vehicle.type, 'car')
            color = 'b';  % 蓝色代表汽车
            width = vehicle.width;
            length = vehicle.length;
            alpha = 0.7;  % 透明度
        else  % bike
            color = 'g';  % 绿色代表自行车
            width = vehicle.width;
            length = vehicle.length;
            alpha = 0.8;  % 透明度
        end
        
        % 获取车辆当前位置
        centerX = vehicle.x;
        centerY = vehicle.y;
        
        % 计算车道方向向量
        startX = lanes.startX(laneIdx);
        startY = lanes.startY(laneIdx);
        endX = lanes.endX(laneIdx);
        endY = lanes.endY(laneIdx);
        
        dx = endX - startX;
        dy = endY - startY;
        dist = sqrt(dx^2 + dy^2);
        
        if dist > 0
            dirX = dx / dist;
            dirY = dy / dist;
            
            % 计算方向角度（以弧度表示）
            angle = atan2(dirY, dirX);
            
            % 计算四个角点的坐标（根据车辆方向旋转）
            % 计算未旋转状态下的角点偏移（车辆前后左右）
            frontOffset = length / 2;
            rearOffset = -length / 2;
            leftOffset = width / 2;
            rightOffset = -width / 2;
            
            % 计算旋转后的角点坐标
            cos_angle = cos(angle);
            sin_angle = sin(angle);
            
            % 前左
            x1 = centerX + frontOffset * cos_angle - leftOffset * sin_angle;
            y1 = centerY + frontOffset * sin_angle + leftOffset * cos_angle;
            
            % 前右
            x2 = centerX + frontOffset * cos_angle - rightOffset * sin_angle;
            y2 = centerY + frontOffset * sin_angle + rightOffset * cos_angle;
            
            % 后右
            x3 = centerX + rearOffset * cos_angle - rightOffset * sin_angle;
            y3 = centerY + rearOffset * sin_angle + rightOffset * cos_angle;
            
            % 后左
            x4 = centerX + rearOffset * cos_angle - leftOffset * sin_angle;
            y4 = centerY + rearOffset * sin_angle + leftOffset * cos_angle;
            
            % 使用填充多边形绘制车辆
            h = patch([x1, x2, x3, x4], [y1, y2, y3, y4], color, 'Tag', 'vehicle');
            
            % 设置透明度
            set(h, 'FaceAlpha', alpha);
            
            % 为汽车添加方向指示器
            if strcmp(vehicle.type, 'car')
                % 添加一个简单的箭头表示行驶方向，使其更加明显
                arrowLength = length * 0.6;
                arrowWidth = width * 0.4;
                
                % 计算箭头位置
                arrowStartX = centerX;
                arrowStartY = centerY;
                arrowEndX = centerX + arrowLength * 0.4 * cos_angle;
                arrowEndY = centerY + arrowLength * 0.4 * sin_angle;
                
                % 绘制箭头
                quiver(arrowStartX, arrowStartY, cos_angle, sin_angle, 0.2, 'w', ...
                    'LineWidth', 1.5, 'MaxHeadSize', 2, 'Tag', 'vehicle');
            end
            
            % 为自行车添加一个简单标记
            if strcmp(vehicle.type, 'bike')
                % 在自行车中心添加一个点
                plot(centerX, centerY, '.w', 'MarkerSize', 4, 'Tag', 'vehicle');
            end
        else
            % 如果无法确定方向，回退到默认的矩形显示
            rectangle('Position', [centerX - length/2, centerY - width/2, length, width], ...
                     'Curvature', [0.2, 0.2], 'FaceColor', color, 'Tag', 'vehicle');
        end
        
        % 可选：显示车辆状态
        if false  % 设置为true可启用状态显示
            switch vehicle.status
                case 'waiting'
                    statusColor = 'y';  % 黄色表示等待
                case 'stopped'
                    statusColor = 'r';  % 红色表示停止
                case 'approaching'
                    statusColor = 'm';  % 品红表示接近路口
                otherwise  % moving
                    statusColor = 'g';  % 绿色表示正常移动
            end
            
            % 在车辆旁边绘制一个小点表示状态
            plot(centerX + width/2 * cos(angle + pi/2), centerY + width/2 * sin(angle + pi/2), ...
                 '.', 'Color', statusColor, 'MarkerSize', 8, 'Tag', 'vehicle');
        end
    end
end

%% 函数: 更新可视化 
% 辅助函数 信号灯
function drawSignalLine(model, nodeIdx, roadIdx, nodeX, nodeY, color, tSize)
    startNode = model.roads.startNode{roadIdx};
    endNode = model.roads.endNode{roadIdx};

    if strcmp(model.nodes.id{nodeIdx}, startNode)
        dir = model.roadDirection(roadIdx, :);  % 正方向
    else
        dir = -model.roadDirection(roadIdx, :);  % 反方向
    end

    endX = nodeX + dir(1) * tSize;
    endY = nodeY + dir(2) * tSize;

    line([nodeX, endX], [nodeY, endY], 'Color', color, 'LineWidth', 2, 'Tag', 'signal');
end

function updateVisualization(figHandle, model, vehicles, trafficLights, stats, time)
    global CONFIG
    % 更新仿真可视化
    
    % 切换到图形窗口
    figure(figHandle);
    
    % 使用改进的车辆绘制函数
    drawVehicles(vehicles, model.lanes);
    
    % 清除之前的信号灯绘图
    signalHandles = findobj(gca, 'Tag', 'signal');
    if ~isempty(signalHandles)
        delete(signalHandles);
    end
    
    % 绘制路口信号灯
    nodes = model.nodes;
    roads = model.roads;

    for i = 1:numel(trafficLights.nodeIndex)
        nodeIdx = trafficLights.nodeIndex(i);
        phase = trafficLights.phase(nodeIdx);
        type = trafficLights.type{nodeIdx};
        
        % 获取节点位置
        nodeX = nodes.x(nodeIdx);
        nodeY = nodes.y(nodeIdx);

        % 👈 定义信号灯指示长度
        global CONFIG
        tSize = CONFIG.SIGNAL_ARROW_LENGTH;

        % 计算倒计时
        countdown = ceil(CONFIG.SIGNAL_CYCLE - trafficLights.timer(nodeIdx));
        
        % 显示倒计时（字体小一点，放在路口旁边）
        text(nodeX + 3, nodeY + 3, num2str(countdown), 'Color', 'k', 'FontWeight', 'bold', ...
             'FontSize', 8, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
             'BackgroundColor', [1 1 1 0.6], 'Tag', 'signal');
        
        % 行人相位显示（小巧的行人图标）
        if trafficLights.pedPhase(nodeIdx)
            % 行人相位用一个小的橙色三角形表示
            plot(nodeX, nodeY, '^', 'MarkerSize', 6, 'MarkerFaceColor', [1, 0.5, 0], ...
                 'MarkerEdgeColor', 'k', 'LineWidth', 0.5, 'Tag', 'signal');
            continue;
        end
        
        % 根据路口类型设置信号灯显示
        if strcmp(type, 'X')
            % 获取Xa和Xb组的道路
            xaRoads = model.xaGroup{nodeIdx};
            xbRoads = model.xbGroup{nodeIdx};
            
            % 根据相位设置颜色
            switch phase
                case 0 % 全红
                    xaColor = [0.8, 0, 0]; % 红色
                    xbColor = [0.8, 0, 0]; % 红色
                case 1 % xa组直行和左转
                    xaColor = [0, 0.8, 0]; % 绿色
                    xbColor = [0.8, 0, 0]; % 红色
                case 2 % xb组直行和左转
                    xaColor = [0.8, 0, 0]; % 红色
                    xbColor = [0, 0.8, 0]; % 绿色
                case 3 % xa组右转
                    xaColor = [0, 0.6, 0.8]; % 青色
                    xbColor = [0.8, 0, 0]; % 红色
                case 4 % xb组右转
                    xaColor = [0.8, 0, 0]; % 红色
                    xbColor = [0, 0.6, 0.8]; % 青色
            end
            
            % 绘制小型十字交叉路口符号
            crossSize = 4; % 缩小十字大小
            
            % 绘制Xa组指示器（短线条）
            for m = 1:numel(xaRoads)
                drawSignalLine(model, nodeIdx, xaRoads(m), nodeX, nodeY, xaColor, tSize);
            end
            
            % 绘制Xb组指示器（短线条）
            for m = 1:numel(xbRoads)
                 drawSignalLine(model, nodeIdx, xbRoads(m), nodeX, nodeY, xbColor, tSize);
            end
            
            % 绘制路口外围轮廓（轻量级黑框，增加辨识度）
            rectangle('Position', [nodeX-crossSize-1, nodeY-crossSize-1, 2*crossSize+2, 2*crossSize+2], ...
                     'Curvature', [0.3, 0.3], 'EdgeColor', [0.3, 0.3, 0.3], 'LineWidth', 0.5, ...
                     'LineStyle', '--', 'Tag', 'signal');
            
        elseif strcmp(type, 'T')
            % 获取主干道和支路的道路
            mainRoads = model.mainGroup{nodeIdx};
            branchRoads = model.branchGroup{nodeIdx};
            
            % 根据相位设置颜色
            switch phase
                case 0 % a支路转向
                    mainColor = [0.8, 0, 0]; % 红色
                    branchColor = [0.8, 0.8, 0]; % 黄色
                case 1 % 主干道直行和左转
                    mainColor = [0, 0.8, 0]; % 绿色
                    branchColor = [0.8, 0, 0]; % 红色
                case 2 % 主干道直行和右转
                    mainColor = [0, 0.6, 0.8]; % 青色
                    branchColor = [0.8, 0, 0]; % 红色
            end
            
            % 绘制小型T型路口符号
            tSize = 4; % 缩小T型符号大小
            
            % 用方向向量绘制主干道组
            for m = 1:numel(mainRoads)
                drawSignalLine(model, nodeIdx, mainRoads(m), nodeX, nodeY, mainColor, tSize);
            end
            
            % 用方向向量绘制支路组
            for m = 1:numel(branchRoads)
                drawSignalLine(model, nodeIdx, branchRoads(m), nodeX, nodeY, branchColor, tSize);
            end
            
            % 绘制路口外围轮廓（半圆形，增加辨识度）
            th = linspace(0, pi, 20);
            xunit = tSize * cos(th) + nodeX;
            yunit = tSize * sin(th) + nodeY;
            plot(xunit, yunit, 'Color', [0.3, 0.3, 0.3], 'LineWidth', 0.5, 'LineStyle', '--', 'Tag', 'signal');
            
        else  % A型或B型路口，极简显示
            % 设置颜色
            if strcmp(type, 'A')
                signalColor = [0, 0.7, 0]; % A型用绿色
            else
                signalColor = [0.8, 0, 0]; % B型用红色
            end
            
            % 绘制小点代表简单路口
            plot(nodeX, nodeY, '.', 'MarkerSize', 8, 'Color', signalColor, 'Tag', 'signal');
            
            % 标注路口类型
            text(nodeX, nodeY+2, type, 'Color', signalColor, 'FontSize', 6, ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Tag', 'signal');
        end
    end
    
    % 更新统计数据显示
    statsText = sprintf('仿真时间: %.1f秒\n', time);
    
    % 计算平均承载量
    timeIdx = ceil(time / 2);
    if timeIdx >= 1 && timeIdx <= size(stats.roadCapacity, 2)
        avgCapacity = mean(stats.roadCapacity(:, timeIdx));
        varCapacity = var(stats.roadCapacity(:, timeIdx));
        statsText = [statsText, sprintf('道路平均承载量: %.2f\n', avgCapacity)];
        statsText = [statsText, sprintf('道路承载量方差: %.2f\n', varCapacity)];
    end
    
    % 车辆统计
    if stats.carCount > 0
        carActiveTime = stats.carTotalTime - stats.carWaitingTime;
        carLowSpeedRatio = stats.carLowSpeedTime / max(carActiveTime, 1e-6);  % 防止除0

        carAvgWaitTime = stats.carWaitingTime / stats.carCount;
        carAvgStopCount = stats.carStopCount / stats.carCount;
        
        statsText = [statsText, sprintf('汽车低速时间占比: %.2f%%\n', carLowSpeedRatio * 100)];
        statsText = [statsText, sprintf('汽车平均等待时间: %.2f秒\n', carAvgWaitTime)];
        statsText = [statsText, sprintf('汽车平均停车次数: %.2f次\n', carAvgStopCount)];
    end
    
    if stats.bikeCount > 0
        bikeActiveTime = stats.bikeTotalTime - stats.bikeWaitingTime;
        bikeLowSpeedRatio = stats.bikeLowSpeedTime / max(bikeActiveTime, 1e-6);

        bikeAvgWaitTime = stats.bikeWaitingTime / stats.bikeCount;
        bikeAvgStopCount = stats.bikeStopCount / stats.bikeCount;
        
        statsText = [statsText, sprintf('自行车低速时间占比: %.2f%%\n', bikeLowSpeedRatio * 100)];
        statsText = [statsText, sprintf('自行车平均等待时间: %.2f秒\n', bikeAvgWaitTime)];
        statsText = [statsText, sprintf('自行车平均停车次数: %.2f次\n', bikeAvgStopCount)];
    end
    
    if ~isempty(vehicles)
        statsText = [statsText, sprintf('当前车辆数: %d (汽车: %d, 自行车: %d)', ...
            numel(vehicles), ...
            sum(strcmp({vehicles.type}, 'car')), ...
            sum(strcmp({vehicles.type}, 'bike')))];
    else
        statsText = [statsText, sprintf('当前车辆数: 0 (汽车: 0, 自行车: 0)')];
    end
    
    textHandle = findobj(figHandle, 'Tag', 'statsText');
    set(textHandle, 'String', statsText);
    
    drawnow;
end


%% 函数: 显示最终统计结果
function displayFinalStatistics(stats)
    figure('Name', 'Simulation Statistics', 'Position', [200, 200, 1000, 600]);
    %% subplot 1: 平均承载量曲线
    subplot(2, 2, 1);
    timeSteps = 1:size(stats.roadCapacity, 2);
    avgCapacity = mean(stats.roadCapacity, 1);
    plot(timeSteps * 2, avgCapacity, 'b-', 'LineWidth', 2);
    title('Average Road Capacity Over Time');
    xlabel('Time (s)');
    ylabel('Average Capacity');
    grid on;

    %% subplot 2: 方差曲线
    subplot(2, 2, 2);
    varCapacity = var(stats.roadCapacity, 0, 1);
    plot(timeSteps * 2, varCapacity, 'r-', 'LineWidth', 2);
    title('Road Capacity Variance Over Time');
    xlabel('Time (s)');
    ylabel('Capacity Variance');
    grid on;

    %% subplot 3: 柱状图统计
    subplot(2, 2, 3);
    carStats = [0, 0, 0];
    bikeStats = [0, 0, 0];
    if stats.carCount > 0
        carActiveTime = stats.carTotalTime - stats.carWaitingTime;
        carStats = [stats.carLowSpeedTime / max(carActiveTime, 1e-6) * 100, ...
                    stats.carWaitingTime / stats.carCount, ...
                    stats.carStopCount / stats.carCount];
    end
    if stats.bikeCount > 0
        bikeActiveTime = stats.bikeTotalTime - stats.bikeWaitingTime;
        bikeStats = [stats.bikeLowSpeedTime / max(bikeActiveTime, 1e-6) * 100, ...
                     stats.bikeWaitingTime / stats.bikeCount, ...
                     stats.bikeStopCount / stats.bikeCount];
    end


    statsMatrix = [carStats; bikeStats];
    b = bar(statsMatrix', 'grouped'); hold on;
    b(1).FaceColor = [0.2, 0.4, 0.9];
    b(2).FaceColor = [0.2, 0.8, 0.2];

    for i = 1:3
        for j = 1:2
            x = b(j).XEndPoints(i);
            y = b(j).YEndPoints(i);
            text(x, y + 0.5, sprintf('%.1f', statsMatrix(j,i)), ...
                'HorizontalAlignment', 'center', 'FontSize', 9);
        end
    end
    set(gca, 'XTickLabel', {'Low Speed %', 'Average Wait Time', 'Average Stop Times'});
    title('Car vs Bike Statistics');
    legend({'Car', 'Bike'});
    ylabel('Value');
    grid on;

    %% subplot 4: 扇形图 + 比例标签
    subplot(2, 2, 4);
    totalCars = stats.carCount;
    totalBikes = stats.bikeCount;
    total = totalCars + totalBikes;

    if total > 0
        carPct = totalCars / total * 100;
        bikePct = totalBikes / total * 100;
        labels = {
            sprintf('Car: %d (%.1f%%)', totalCars, carPct), ...
            sprintf('Bike: %d (%.1f%%)', totalBikes, bikePct)
        };
        pie([totalCars, totalBikes], labels);
    else
        pie([1, 1], {'Car: 0', 'Bike: 0'});
    end
    title(sprintf('Vehicle Distribution (Total: %d)', total));

    %% 命令行输出
    fprintf('\nFinal Statistics:\n');
    fprintf('Number of Cars: %d\n', stats.carCount);
    fprintf('Number of Bikes: %d\n', stats.bikeCount);
    if stats.carCount > 0
        fprintf('Car Low Speed Time Ratio: %.2f%%\n', carStats(1));
        fprintf('Car Average Waiting Time: %.2f s\n', carStats(2));
        fprintf('Car Average Stops: %.2f\n', carStats(3));
    end
    if stats.bikeCount > 0
        fprintf('Bike Low Speed Time Ratio: %.2f%%\n', bikeStats(1));
        fprintf('Bike Average Waiting Time: %.2f s\n', bikeStats(2));
        fprintf('Bike Average Stops: %.2f\n', bikeStats(3));
    end

    %% 自动保存
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    filenameBase = ['simulation_statistics_' timestamp];
    exportgraphics(gcf, [filenameBase '.png'], 'Resolution', 300);
    savefig([filenameBase '.fig']);
end




