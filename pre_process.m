load('practice.mat')
%% Goal
%posePF = particleLocalization(ranges[:,0:num-1], scanAngles, M, param)

%% Parameter Desciption
% init_pose: 一开始的位置和heading角度
% M: 地图
% pose: measurement
% t: 时间
% ranges: 各个角度*每一时刻各个角度的距离
% scan Angles：各个角度

%% Testing
% Test Map
imshow(M);
%map(x,y) is a log odds ratio of occupancy probability at (x,y)

% Testing Angle
i=1;
(scanAngles(1081)-scanAngles(1))/pi
% It shows that the Lidar can only scan 270degrees 

% Test Pose
% ...

% Test ranges of the first Time Step
t=100;
lidar_local_x=ranges(:,t).*cos(scanAngles);
lidar_local_y=-ranges(:,t).*sin(scanAngles);
plot(lidar_local_x,lidar_local_y);

% Map Matching in Demo
resol=25;
origin=[685,572];
imshow(M);
hold on;
lidar_gobal_x=pose(1,:)*resol+origin(1);
lidar_gobal_y=pose(2,:)*resol+origin(2);
plot(lidar_gobal_x,lidar_gobal_y, 'r.-');
hold off;
