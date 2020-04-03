close all;

filename = 'cyberzoo.png';
fig=figure()
y = imread(filename, 'BackgroundColor', [1 1 1]);

v = VideoWriter('newfile.avi','Motion JPEG AVI');
open(v);
robot_radius = 0.320156;

% plot(reference_traj_sim(1,:)-1.5,reference_traj_sim(2,:)-2.5,'+')
% grid on
% hold on
% h = image([-5.3 5.3],[-5.3 5.3],y);
%     uistack(h,'bottom');
%     axis equal
%     axis([-5 5 -5 5])
%     title('Traveled path by the robot','Interpreter','Latex')
%     plot(robot_state_ekf(1,1:end)-1.5,robot_state_ekf(2,1:end)-2.5)
% plot(robot_state_mocap(1,:),robot_state_mocap(2,:))

for i=1:1:length(robot_state_ekf)
    plot(reference_traj_sim(1,:)-1.5,reference_traj_sim(2,:)-2.75,'+')
    grid on
    hold on

    
    h = image([-5.3 5.3],[-5.3 5.3],y);
    uistack(h,'bottom');
    axis equal
    axis([-5 5 -5 5])
    title('Traveled path by the robot','Interpreter','Latex')
    if(i>35192-202)
        plot(robot_state_ekf(1,i:end),robot_state_ekf(2,i:end))
        drawEllipse(person1(1,i),person1(2,i),obsth_0(1,i),obsta_0,obstb_0);
        drawEllipse(person2(1,i),person2(2,i),obsth_1(1,i),obsta_1,obstb_1);
        drawCircle(robot_state_ekf(1,i),robot_state_ekf(2,i),robot_radius)
    else
        %plot(robot_state_mocap(1,i:i+1200),robot_state_mocap(2,i:i+1200))
        plot(robot_state_ekf(1,i:i+200)-1.5,robot_state_ekf(2,i:i+200)-2.5,'*')
        drawEllipse(person1(1,i),person1(2,i),obsth_0(1,i),obsta_0,obstb_0);
        drawEllipse(person2(1,i),person2(2,i),obsth_1(1,i),obsta_1,obstb_1);
        %drawCircle(robot_state_mocap(1,i),robot_state_mocap(2,i),robot_radius)
        drawCircle(robot_state_ekf(1,i)-1.5,robot_state_ekf(2,i)-2.5,robot_radius)
    end
    %reply = input('Do you want more? Y/N [Y]: ', 's');
    frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
    writeVideo(v,frame)
    unplot;
end
close(v);