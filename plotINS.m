close all
figure()
plot(pposetrue(:,1),pposetrue(:,2))
hold on
plot(ppose(:,1),ppose(:,2))
hold on
plot(GGPS(:,1),GGPS(:,2))
hold on
xlabel('x(m)')
ylabel('y(m)')
legend('Truepos','Kalman','GPS','FontSize',12);
title ('Robot Trajectory')
drawnow
figure()
plot(ppose(:,1)-pposetrue(:,1))
hold on
xlabel('Time step')
ylabel('Error x (m)')
title('Error in X')
%ylim([-0.1 0.1]);
drawnow

figure()
plot(pposetrue(:,2));
hold on
plot(ppose(:,2))
hold on
plot(GGPS(:,2))
hold on
legend('truepos','Kaman','GPS','FontSize',12)
title('Y')
xlabel('Time step')
ylabel('y (m)')
grid on

figure()
plot(pposetrue(:,1));
hold on
plot(ppose(:,1))
hold on
plot(GGPS(:,1))
hold on
legend('truepos','Kaman','GPS','FontSize',12)
title('x')
xlabel('Time step')
ylabel('x (m)')
grid on

figure()
plot(ppose(:,2)-pposetrue(:,2))
hold on
xlabel('Time step')
ylabel('Error y (m)')
%ylim([-0.1 0.1]);
title('Error in Y')
drawnow