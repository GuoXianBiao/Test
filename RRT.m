clearvars
clear;
clc;
close all;
orgin_map=im2bw(imread('map2.bmp'));           % ���ص�ͼ
x_start.coord = [100 100];
x_goal.coord = [450 400];
[x_max, y_max]=size(orgin_map);                                 % ��ȡ��ͼ�ߴ��С
map = gridmapextend(5,orgin_map);                               % �ϰ���չ�Ĳ���Ϊ5���滮·��ʱ��ʹ��map��ͼ

map_op = setOccupationProbabilityValuesOriginMap(orgin_map);    % դ���ͼ��orgin_map��map�ȣ�תΪ���ʵ�ͼ
newmap_op = OccupationProbabilityValuesExtend(map_op,6,-0.25);  % ���ʵ�ͼ��չ 
                                                                % ����1��Ԥ��չ�ĸ��ʵ�ͼ
                                                                % ����2����չ�Ĳ���
                                                                % ����3��1/n(������ƽ��)
imshow(orgin_map);                                              % ��ʾԭʼ��ͼ
% ��ʼ������
iteration = 20000;                  % ����������
step_length = 20;                   % �������չ����
disTh=30;                           % Ŀ������Χ
x_start.cost = 0;                   % ·����ʼ�����
x_start.parent = 0;                 % ·����ʼ������ֵ
x_goal.cost = 0;                    % ·��Ŀ������
pathFound=0;                        % ����ҵ�·������pathFoundΪ1
nodes(1) = x_start;                 % ������е�һ����

figure(1)                           % ����ͼ��
axis([0 x_max 0 y_max])             % ����������ķ�Χ
hold on;                            % ���ֵ�ǰ��ͼ����ɫ���Բ���ˢ��
plot(x_start.coord(2), x_start.coord(1),'.','Color', 'b', 'MarkerSize', 30); % �������
plot(x_goal.coord(2), x_goal.coord(1), '.','Color','r', 'MarkerSize', 30);   % �����յ�
if ~feasiblePoint(x_start.coord,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(x_goal.coord,map), error('goal lies on an obstacle or outside map'); end
tic; %��ʼ��ʱ
for i = 1:1:iteration
    % 1.����
    q_rand = [rand(1)*x_max, rand(1)*y_max];  %  �������������
    x_rand = [max([1, floor(q_rand(1))]), max([1, floor(q_rand(2))])];
    if  ~map(x_rand(1),x_rand(2))==1;continue;end
    plot(x_rand(2), x_rand(1), 'x', 'Color',[0.4660 0.6740 0.1880]);%��ʾ��������¹���
    % 2.������������ҳ��������������Ľڵ�
    [x_nearest, mindistance_value] = FindNearestNode(nodes, x_rand);
    % 3.��չ�½ڵ�
    x_new.coord = steer(x_rand, x_nearest.coord, mindistance_value, step_length);
   if ~CollisionFree(x_nearest.coord, x_new.coord, map),continue;end
    x_new.cost = distanceCost(x_new.coord, x_nearest.coord) + x_nearest.cost; %��ȡ�½ڵ��cost��parent
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
    fprintf('�� %d ��ѭ��\n',i);
end
path = Find_Path(nodes, x_start, x_goal);%ԭʼ·���㼯%δ����·���޼�����
for i=1:1:length(path)-1
  line([path(i,2), path(i+1,2)], [path(i,1), path(i+1,1)], 'Color', 'r', 'LineWidth', 3);
end
pathTime = toc;%��ʼ·������ʱ��
pathLength = calcuatepathlength(path);%��ʼ·������
pathSmooth =  calcuatepathsmoothness(path);%��ʼ·��ƽ��
pathSafety = calcuatepathsafety(newmap_op,path);
fprintf( '(1)������ڵ������ %d\n', length(nodes));
fprintf( '(2)��ʼ·������ʱ�䣺 %d \n', pathTime);
fprintf( '(3)��ʼ·�����ȣ�%d\n', pathLength);
fprintf( '(4)��ʼ·��ƽ����%d\n', pathSmooth);
fprintf( '(5)��ʼ·����ȫ��%d\n', pathSafety);