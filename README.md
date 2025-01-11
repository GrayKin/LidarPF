# LidarPF
State Space Model
State
$$\mathbf{x}=[x,y,\theta]^T$$
State Transit function
$$\mathbf{x}_{k+1}=f(\mathbf{x}_k)+w_{k}=\mathbf{x}_k+w_{k}$$
Due to the limitations of the data, we can't get any useful prediction, so we just use the previous state as the prediction of the next state.
Measurement
$$\mathbf{z}_k=h(\mathbf{x}_k)+v_k$$
Where the function $$h(\cdot)$$ is how the lidar measures the distance from the car to the wall from -135 to 135 degrees, which is a nonlinear function. $$\mathbf{z}_k$$ is the measurement.
Map Matching
resol=25;
origin=[685,572];
imshow(M);
hold on;
lidar_gobal_x=pose(1,:)*resol+origin(1);
lidar_gobal_y=pose(2,:)*resol+origin(2);
scatter(lidar_gobal_x,lidar_gobal_y,2, 'r*');
hold off;
[图片]
Given the example.py program, we can easily get the method.
Particle Filter
Initialization
Generate particles
function particles=generate_Particles(Num,q1,q2)
particles=zeros(Num,3);
particles(:,1)=particles(:,1)+randn(Num,1)*q1;
particles(:,2)=particles(:,2)+randn(Num,1)*q2;
particles(:,3)=rand(Num,1)*3.14*2-3.14;
end
Weights
Equal.
weights=ones(Num,1)/Num;
Predict
This part is simple because we do not make any prediction other than adding some randomness.
[图片]
function particles=predict(particles,Q)
[m,~]=size(particles);
particles(:,1)=particles(:,1)+randn(m,1)*Q(1);
particles(:,2)=particles(:,2)+randn(m,1)*Q(2);
particles(:,3)=particles(:,3)+randn(m,1)*Q(3);
end
Update
This is the most complicate part in PF. We can not get a theoretical weight for each particle in Bayes. But we can calculate a score to show the correctness of each particle, representing the weight.
In each direction, we can get a distance from the measurement and check if it is a wall. If true, add the score of this particle (increasing the weights).
The following figure shows the predicted states and lidar measurements in the first time step. The blue line indicates the measurement of lidar, and the red points are the particles after adding randomness in the prediction process. Our goal is to find the best particle which can exactly fit the blue lidar measurement by giving a high score.
[图片]
For example, the score of the left particle should be much lower than the right one.
[图片]
[图片]
We design a scoring algorithm which checks the end point of the lidar. If the end point is the wall, we add points, otherwise remain the same. And if the end point is out of the map, the score will be minus 0.2.

Map Occupied
Map Free
Out of the Map
Lidar Occupied
+2
-0.1
-0.2
In order to make the suitable particle outstanding, we use the cube of the score after assigning the small values to zero.(Reduce the computational burden) Of course, the weights should be normalized at the last step.
$$\begin{aligned}
weight_{new} \propto weight_{old}\times(score)^{3.5}
\end{aligned}$$
function weights=update(particles,weights,M,ranges,scanAngles)
[m,~]=size(particles);
angles=length(ranges);
scores=zeros(m,1);
resol=25;
origin=[685,572];
for i=1:m
    % Map matching for each particle
    lidar_local_x=ranges.*cos(scanAngles+particles(i,3));
    lidar_local_y=-ranges.*sin(scanAngles+particles(i,3));
    lidar_gobal_x=(particles(i,1)+lidar_local_x)*resol+origin(1);
    lidar_gobal_y=(particles(i,2)+lidar_local_y)*resol+origin(2);
    % calculate score for each particle
    tmp_score=0;
    for j=1:angles
        % detect if there is a wall here for each direction
        Map_loc=[round(lidar_gobal_y(j)),round(lidar_gobal_x(j))];
        if Map_loc(1)<=824&&Map_loc(2)<=870&&Map_loc(1)>0&&Map_loc(2)>0
            if M(Map_loc(1),Map_loc(2))>1
                tmp_score=tmp_score+1;
            elseif M(Map_loc(1),Map_loc(2))>0
                tmp_score=tmp_score-0.1;
            end
        else
            tmp_score=tmp_score-0.2;
        end
    end
    % result
    scores(i)=tmp_score;
end
scores=scores-50;
scores(scores<0)=0;
scores=scores.^3.5;
weights =weights.*scores;
weights =weights/ sum(weights);
end
There will be mistakes. For example, the score of the left following figure is lower than the right one, while obviously the left one is better. It is unavoidable because I want to simplify our algorithm. Otherwise, we need to check all the area in the measurements of lidar. There will be both difficulties in coding and calculation.
[图片]
[图片]
Resampling
[图片]
function [particles,weights]=resample_systematic(particles,weights)
    N = length(weights);
    %make N subdivisions, choose positions with a random offset
    positions = (unifrnd(-1,0,[N,1])+(1:N)')/N;
    cumulative_sum = cumsum(weights);
    j=1;
    for i=1:N
        while(positions(i)>cumulative_sum(j))
            j=j+1;
        end
        particles(i,:)=particles(j,:);
    end
    %weights are equal after resampling
    weights=ones([N,1])/N;
end
function val=neff(weights)
    %when val<N/2, particles need resample
    val=1. /sum(weights.^2);
end
Counting the efficient particles to determine when to do resampling.
function val=neff(weights)
    %when val<N/2, particles need resample
    val=1. /sum(weights.^2);
end
Estimate
Calculate a weighted average.
function [mu,var]=estimate(particles,weights)
    %input:
    %particles: N*k
    %weight: N*1
    %output:
    %mu:1*k
    %var:1*k
    mu=sum(particles.*weights)/sum(weights);
    var=(particles-mu).^2;
    var=sum(var.*weights)/sum(weights);
end
Whole Algorithm
function posePF = particleLocalization(ranges, scanAngles, M,Num)
%% Implement of a particle filter for pose tracking in 2D space with lidar measurement
% Input:
%   M: 地图
%   ranges: 各个角度*每一时刻各个角度的距离
%   scan Angles：lidar各个角度
%   Num: 粒子数目
% Output:
%   posePF: 汽车轨迹
T=3701;
%% Initialize
particles=generate_Particles(Num,0.1,0.1);% 3 states: x,y, theta
weights=ones(Num,1)/Num;
xs=zeros(T,3);
Q=[0.4,0.4,1];
%% Main Algorithm
for t=1:T
    % predict
    particles=predict(particles,Q);
    % update
    weights=update(particles,weights,M,ranges(:,t),scanAngles);
    % resample
    if(neff(weights)<Num/2)
        [particles,weights]=resample_systematic(particles,weights);
    end
    % estimate
    [mu,var]=estimate(particles,weights);
    xs(t,:)=mu;
end
posePF=xs';
end
