%% Relative motion dynamics

p_I = [1,2,0];
p_B=[];
j=1;
p_B(1,1:3) =[1;2;0];
dt=0.1;
w=0;
v=0;
p_goal =[1;1;0];
p=[0;0;0];

%controller
K=0.1*[1 1 1;
    1 1 1];
kp=3;
kalpha = 8;
kbeta = -1.5;

for t=0:dt:10
    j=j+1;
    % error computation
    e(:,j) = p_goal-p(:,j-1);
    
    %angles
    alpha = -p(3,j-1)+atan2(e(2,j),e(1,j));
    beta = -p(3,j-1)-alpha;
    
    %compute control input
    u= [kp*norm(e(1:2,j));kalpha*alpha+kbeta*beta];
    
    p(:,j)=p(:,j-1)+dt*[cos(p(3,j-1)) 0;sin(p(3,j-1)) 0;0 1]*u;
    
    %p_B(j,:)=p_B(j-1,:)+[w*p_B(j-1,2)-v*cos];
    
end

figure
hold on;

plot(0:dt:10+dt,p);