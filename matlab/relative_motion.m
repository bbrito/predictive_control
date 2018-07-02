%% Relative motion dynamics

p_I = [1,2,0];
p_B=[];
j=1;
p_B(1,1:3) =[1;2;0];
dt=0.1;
w=0;
v=0;
for t=0:dt:10
    j=j+1;
    p_B(j,:)=p_B(j-1,:)+[w*p_B(j-1,2)-v*cos];
    
end