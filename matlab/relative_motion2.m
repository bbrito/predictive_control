%% Relative motion dynamics
clc;
clear;
close all;

p_I = [1,2,0];
p_B=[];
j=1;
p_B(1,1:3) =[1;2;0];
dt=0.1;
w=0;
v=0;
p_goal =[1;1;atan2(1,1)];
p=p_goal;

%controller
K=0.1*[1 1 1;
    1 1 1];
kp=0.1;
kalpha = 0.01;
kbeta = 0;
p_robot = [0;0;0];
fun = @(x,u)norm(x(1))+norm(x(2));
for t=0:dt:50
    j=j+1;
    % error computation
    e(:,j) = p(:,j-1);
    
    %angles
    alpha = -p(3,j-1)+atan2(e(2,j),e(1,j));
    beta = -p(3,j-1)-alpha;
    
    %compute control input
    u(:,j)= fmincon(fun,p(:,j-1),A,b);

    p(:,j)=p(:,j-1)+dt*[-u(2,j)*p(2,j-1)-u(1,j)*cos(p(3,j-1));u(2,j)*p(1,j-1)-u(1,j)*sin(p(3,j-1));-u(2,j)];
    p(3,j) = atan2(sin(p(3,j)),cos(p(3,j)));
    p_robot(:,j) = p_robot(:,j-1) +dt*[cos(p_robot(3)) 0;sin(p_robot(3)) 0;0 1]*u(:,j);
    %p_B(j,:)=p_B(j-1,:)+[w*p_B(j-1,2)-v*cos];
    
end

figure
hold on;

plot(0:dt:50+dt,p_robot);

figure
hold on;

plot(0:dt:50+dt,p);

figure
hold on;
title('Control input');
plot(0:dt:50+dt,u);