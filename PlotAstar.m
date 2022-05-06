close all
figure()
plot(ppose(:,1),ppose(:,2))
hold on
plot(Oppose(:,1),Oppose(:,2))
hold on
xlabel('x(m)')
ylabel('y(m)')
legend('Truepos','Odematry','FontSize',12);
title ('Robot Trajectory')
drawnow
figure()
plot(ppose(:,1)-Oppose(:,1))
hold on
xlabel('Time step')
ylabel('Error x (m)')
title('Error in X')
ylim([-0.1 0.1]);
drawnow

figure()
plot(ppose(:,2)-Oppose(:,2))
hold on
xlabel('Time step')
ylabel('Error y (m)')
ylim([-0.1 0.1]);
title('Error in Y')
drawnow

% figure()
% plot(ppose(:,1))
% hold on
% plot(Oppose(:,1))
% hold on
% title('x')
% drawnow
% 
% figure()
% plot(ppose(:,2))
% hold on
% plot(Oppose(:,2))
% hold on
% title('y')
% drawnow