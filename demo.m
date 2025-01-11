Num=10000;
T=3701;
% initialize
particles=generate_Particles(Num,0.1,0.1);% 3 states: x,y, theta
weights=ones(Num,1)/Num;
xs=zeros(T,3);
Q=[0.4,0.4,1];
for t=1:T
    % predict
    particles=predict(particles,Q);
    % update
    %tag=mod(t-1,2);
    weights=update(particles,weights,M,ranges(:,t),scanAngles);%,pose(:,t),tag);
    % resample
    if(neff(weights)<Num/2)
        [particles,weights]=resample_systematic(particles,weights);
        t
    end
    % estimate
    [mu,var]=estimate(particles,weights);
    xs(t,:)=mu;
end

function particles=generate_Particles(Num,q1,q2)
particles=zeros(Num,3);
particles(:,1)=particles(:,1)+randn(Num,1)*q1;
particles(:,2)=particles(:,2)+randn(Num,1)*q2;
particles(:,3)=rand(Num,1)*3.14*2-3.14;
end

function weights=update(particles,weights,M,ranges,scanAngles)
[m,~]=size(particles);
angles=length(ranges);
scores=zeros(m,1);
resol=25;
origin=[685,572];
for i=1:m
    % calculate score for each particle
    
    %hold off
    % Map matching for each particle
   
    
    % if tag==0
    %     imshow(M);
    %     hold on;
    %     lidar_gobal_x=particles(:,1)*resol+origin(1);
    %     lidar_gobal_y=particles(:,2)*resol+origin(2);
    %     plot(lidar_gobal_x,lidar_gobal_y, 'r.-');
    %     hold off;
    % 
    % 
    %     imshow(M);
    %     hold on;
    %     lidar_local_x=ranges.*cos(scanAngles+pose(3));
    %     lidar_local_y=-ranges.*sin(scanAngles+pose(3));
    %     lidar_gobal_x=(lidar_local_x+pose(1))*resol+origin(1);
    %     lidar_gobal_y=(lidar_local_y+pose(2))*resol+origin(2);
    %     plot(lidar_gobal_x,lidar_gobal_y);
    %     scatter((particles(:,1))*resol+origin(1),(particles(:,2))*resol+origin(2),'r*');
    %     hold off
    %     tag=1;
    % end 
    lidar_local_x=ranges.*cos(scanAngles+particles(i,3));
    lidar_local_y=-ranges.*sin(scanAngles+particles(i,3));
    lidar_gobal_x=(particles(i,1)+lidar_local_x)*resol+origin(1);
    lidar_gobal_y=(particles(i,2)+lidar_local_y)*resol+origin(2);
    % imshow(M);
    % hold on;
    % plot(lidar_gobal_x,lidar_gobal_y, 'r.-');
    % hold off;
    %result
    tmp_score=0;
    for j=1:angles
        % detect if there is a wall here for each direction
        %Map_loc=[round(lidar_gobal_x(j)),round(lidar_gobal_y(j))];
        Map_loc=[round(lidar_gobal_y(j)),round(lidar_gobal_x(j))];
        if Map_loc(1)<=824&&Map_loc(2)<=870&&Map_loc(1)>0&&Map_loc(2)>0
            if M(Map_loc(1),Map_loc(2))>1
                tmp_score=tmp_score+2;
            elseif M(Map_loc(1),Map_loc(2))>0
                tmp_score=tmp_score-0.1;
            end
        else
            tmp_score=tmp_score-0.2;
        end
    end
    % imshow(M);
    % hold on;
    % plot(lidar_gobal_x,lidar_gobal_y, 'r.-');
    % hold off;
    % result
    scores(i)=tmp_score;
end
scores=scores-50;
scores(scores<0)=0;
scores=scores.^3.5;
weights =weights.*scores;
weights =weights/ sum(weights);
end


function particles=predict(particles,Q)
[m,~]=size(particles);
particles(:,1)=particles(:,1)+randn(m,1)*Q(1);
particles(:,2)=particles(:,2)+randn(m,1)*Q(2);
particles(:,3)=particles(:,3)+randn(m,1)*Q(3);
end

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