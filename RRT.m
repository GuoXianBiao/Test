clearvars
clear;
clc;
close all;
orgin_map=im2bw(imread('map2.bmp'));           % 加载地图
x_start.coord = [100 100];
x_goal.coord = [450 400];
[x_max, y_max]=size(orgin_map);                                 % 获取地图尺寸大小
map = gridmapextend(5,orgin_map);                               % 障碍扩展的层数为5，规划路径时，使用map地图

map_op = setOccupationProbabilityValuesOriginMap(orgin_map);    % 栅格地图（orgin_map，map等）转为概率地图
newmap_op = OccupationProbabilityValuesExtend(map_op,6,-0.25);  % 概率地图扩展 
                                                                % 参数1：预扩展的概率地图
                                                                % 参数2：扩展的层数
                                                                % 参数3：1/n(层数的平方)
imshow(orgin_map);                                              % 显示原始地图
% 初始化参数
iteration = 20000;                  % 最大迭代次数
step_length = 20;                   % 随机树扩展步长
disTh=30;                           % 目标区域范围
x_start.cost = 0;                   % 路径起始点代价
x_start.parent = 0;                 % 路径起始点索引值
x_goal.cost = 0;                    % 路径目标点代价
pathFound=0;                        % 如果找到路径，则pathFound为1
nodes(1) = x_start;                 % 随机树中第一个点

figure(1)                           % 建立图形
axis([0 x_max 0 y_max])             % 设置坐标轴的范围
hold on;                            % 保持当前绘图的颜色属性不被刷新
plot(x_start.coord(2), x_start.coord(1),'.','Color', 'b', 'MarkerSize', 30); % 绘制起点
plot(x_goal.coord(2), x_goal.coord(1), '.','Color','r', 'MarkerSize', 30);   % 绘制终点
if ~feasiblePoint(x_start.coord,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(x_goal.coord,map), error('goal lies on an obstacle or outside map'); end
tic; %开始计时
for i = 1:1:iteration
    % 1.采样
    q_rand = [rand(1)*x_max, rand(1)*y_max];  %  产生随机采样点
    x_rand = [max([1, floor(q_rand(1))]), max([1, floor(q_rand(2))])];
    if  ~map(x_rand(1),x_rand(2))==1;continue;end
    plot(x_rand(2), x_rand(1), 'x', 'Color',[0.4660 0.6740 0.1880]);%显示采样点更新过程
    % 2.遍历随机树，找出距离采样点最近的节点
    [x_nearest, mindistance_value] = FindNearestNode(nodes, x_rand);
    % 3.扩展新节点
    x_new.coord = steer(x_rand, x_nearest.coord, mindistance_value, step_length);
   if ~CollisionFree(x_nearest.coord, x_new.coord, map),continue;end
    x_new.cost = distanceCost(x_new.coord, x_nearest.coord) + x_nearest.cost; %获取新节点的cost和parent
    for j = 1:1:length(nodes)
        if nodes(j).coord == x_nearest.coord
            x_new.parent = j;
        end
    end    
    nodes = [nodes x_new];
    plot(x_new.coord(2),x_new.coord(1),'*r');
    line([nodes(x_new.parent).coord(2), x_new.coord(2)], [nodes(x_new.parent).coord(1), x_new.coord(1)], 'Color', 'b', 'LineWidth', 0.01);
    if distanceCost(x_new.coord,x_goal.coord) < disTh && CollisionFree(x_goal.coord, x_new.coord, map)
        pathFound=1;
        break;
    end
    pause(0.01);
    fprintf('第 %d 次循环\n',i);
end
path = Find_Path(nodes, x_start, x_goal);%原始路径点集%未加入路径修剪！！
for i=1:1:length(path)-1
  line([path(i,2), path(i+1,2)], [path(i,1), path(i+1,1)], 'Color', 'r', 'LineWidth', 3);
end
pathTime = toc;%初始路径计算时间
pathLength = calcuatepathlength(path);%初始路径长度
pathSmooth =  calcuatepathsmoothness(path);%初始路径平滑
pathSafety = calcuatepathsafety(newmap_op,path);
fprintf( '(1)随机树节点个数： %d\n', length(nodes));
fprintf( '(2)初始路径计算时间： %d \n', pathTime);
fprintf( '(3)初始路径长度：%d\n', pathLength);
fprintf( '(4)初始路径平滑：%d\n', pathSmooth);
fprintf( '(5)初始路径安全：%d\n', pathSafety);