close all  % close all opening figure windows
clear % Clear all variables in workspace
clc
global robPose ppose
% Sampling time
dt = 0.1;
wheel_radifront=0.03;
vrep=remApi('remoteApi');
vrep.simxFinish(-1); 
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
ppose=[0 0 0];
position=[0 0 0];
% Start pose
xk=0; %for caculating kinematic
yk=0; %for caculating kinematic
thetak=0; %for caculating kinematic

% Robot parameter: Distance between two wheels
d=0.0823;
% End pose
% target.time=0;
% target.signals.values=[1 1 pi/2];
goal=[1 1 pi/2];
% Control parameters
gamma=0.3;
lamda=0.6;
h=0.1;
endgoal=false

robPose=[0 0 0]
 if (clientID>-1)
    disp('Connected');
    [returnCode,Robot]=vrep.simxGetObjectHandle(clientID,'Robot',vrep.simx_opmode_blocking );
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking );
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking );
    [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_streaming)
    [returnCode,Orobot]=vrep.simxGetObjectHandle(clientID,'Robotpose',vrep.simx_opmode_blocking)
    [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_streaming);
    

    while endgoal==false
    [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_buffer)
    [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
    x=position(1);
    y=position(2);
    theta1= orientation(3);  
    % Calculate polar variables
    rho = sqrt((goal(1)-x)^2+(goal(2)-y)^2);
    phi = atan2(goal(2)-y,goal(1)-x)-goal(3);
    alpha = phi+goal(3)-theta1;
    % Calculate control laws
    v = gamma*cos(alpha)*rho;
    w = lamda*alpha + gamma*cos(alpha)*sin(alpha)/alpha*(alpha+h*phi);
    % Calculate Vl and Vr from v and w
    vl = (2*v-d*w)/2;
    vr = (2*v+d*w)/2;
    xk= xk+(vr+vl)*cos(thetak)*dt/2 ;
    yk= yk+(vr+vl)*sin(thetak)*dt/2 ;
    thetak=thetak +(vr-vl)*dt/(2*d);
   
    robPose=[robPose;[xk,yk,thetak]];
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,vl/wheel_radifront,vrep.simx_opmode_blocking);    
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,right_Motor,vr/wheel_radifront,vrep.simx_opmode_blocking);
    ppose=[ppose;position];
    if sqrt((goal(1)-x)^2+(goal(2)-y)^2)<0.15&& theta1-goal(3)<0.01
        endgoal =true;
    end
    end
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking);    
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,right_Motor,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
    % Run the simulation
    %sim('TestinvKinematic');  
    %Plot results
    figure()
    plot(ppose(:,1),ppose(:,2));
    hold on
    plot(robPose(:,1),robPose(:,2));
    hold on
    legend('Truepos','Kinematic','FontSize',12);
    title('Robot trajectory')
    xlabel('x (m)')
    ylabel('y (m)')
    grid on
    
    figure()
    plot(ppose(:,1)-robPose(:,1))
    title('Error in X')
    xlabel('Time step')
    ylabel('Error x(m)')
    grid on
    figure()
    plot(ppose(:,2)-robPose(:,2))
    title('Error in Y')
    xlabel('Time step')
    ylabel('Error y(m)')
    grid on
    
else
    % Connection Failed
    disp('Failed connecting to remote API server')
end

