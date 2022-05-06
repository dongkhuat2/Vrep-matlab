close all
%all
figure()
plot(pposetrue(:,1),pposetrue(:,2))
hold on
plot(ppose(:,1),ppose(:,2))
hold on
plot(Oppose(:,1),Oppose(:,2))
hold on 
plot(GGPS(:,1),GGPS(:,2))
hold on
plot(initPos(1),initPos(2),'s','color','green','markers',10)
hold on
plot(goalPos(1),goalPos(2),'s','color','red','markers',10)
txt1 = '\leftarrow Init';
txt2 = '\leftarrow Goal';
text(initPos(1),initPos(2),txt1)
text(goalPos(1),goalPos(2),txt2)
xlabel('x (m)')
ylabel('y (m)')
legend('truepos','Kaman','Odematry','GPS','init','goal','FontSize',12);
title('Robot Trajectory')

% true kaman gps
figure()
plot(pposetrue(:,1),pposetrue(:,2))
hold on
plot(ppose(:,1),ppose(:,2))
hold on
plot(GGPS(:,1),GGPS(:,2))
hold on
plot(initPos(1),initPos(2),'s','color','green','markers',10)
hold on
plot(goalPos(1),goalPos(2),'s','color','red','markers',10)
txt1 = '\leftarrow Init';
txt2 = '\leftarrow Goal';
text(initPos(1),initPos(2),txt1)
text(goalPos(1),goalPos(2),txt2)
xlabel('x (m)')
ylabel('y (m)')
legend('truepos','Kaman','GPS','init','goal','FontSize',12);
title('Robot Trajectory')

% true odemetry
figure()
plot(pposetrue(:,1),pposetrue(:,2))
hold on
plot(Oppose(:,1),Oppose(:,2))
hold on 
plot(initPos(1),initPos(2),'s','color','green','markers',10)
hold on
plot(goalPos(1),goalPos(2),'s','color','red','markers',10)
txt1 = '\leftarrow Init';
txt2 = '\leftarrow Goal';
text(initPos(1),initPos(2),txt1)
text(goalPos(1),goalPos(2),txt2)
xlabel('x (m)')
ylabel('y (m)')
legend('truepos','Odemetry','init','goal','FontSize',12);
title('Robot Trajectory')

figure()
plot(pposetrue(:,1));
hold on
plot(ppose(:,1))
hold on
plot(GGPS(:,1))
hold on
legend('truepos','Kaman','GPS','FontSize',12)
drawnow
xlabel('Time step')
ylabel('x (m)')
title('X')
grid on

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
plot(Oppose(:,1))
hold on
legend('truepos','Odemetry')
drawnow
xlabel('Time step')
ylabel('x (m)')
title('X')
grid on

figure()
plot(pposetrue(:,2));
hold on
plot(Oppose(:,2))
hold on
legend('truepos','Odemetry')
drawnow
xlabel('Time step')
ylabel('y (m)')
title('Y')
grid on

%true kaman in X
figure()
plot(pposetrue(:,1)-ppose(:,1))
hold on
%legend('truepos','Kaman')
title('Error in X True and Kaman')
xlabel('Time step')
ylabel('Error x (m)')

%true kaman in Y
figure()
plot(pposetrue(:,2)-ppose(:,2))
hold on
%legend('truepos','Kaman')

title('Error in Y True and Kaman')
xlabel('Time step')
ylabel('Error y (m)')

% True Odemetry in X
figure()
plot(pposetrue(:,1)-Oppose(:,1))
hold on
%legend('truepos','Odemetry')
ylim([-0.1 0.1]);
title('Error in X True and Odemetry')
xlabel('Time step')
ylabel('Error x (m)')

% True Odemetry in Y
figure()
%grid on;
plot(pposetrue(:,2)-Oppose(:,2))
hold on
%legend('truepos','Odemetry')
xlabel('Time step')
ylabel('Error y (m)')
ylim([-0.1 0.1]);
title('Error in Y True and Odemetry')

% %true kaman in X
% figure()
% plot(pposetrue(:,1)-GGPS(:,1))
% hold on
% %legend('truepos','Kaman')
% title('Error in X True and GPS')
% xlabel('Time step')
% ylabel('Error x (m)')
% %true kaman in Y
% figure()
% plot(pposetrue(:,2)-GGPS(:,2))
% hold on
% %legend('truepos','Kaman')
% xlabel('Time step')
% ylabel('Error y (m)')
% title('Error in Y True and GPS')

