close all  % close all opening figure windows
clear % Clear all variables in workspace
clc
global robPose ppose
% Sampling time
Tstep = 0.1;
wheel_radifront=0.03;
vrep=remApi('remoteApi');
vrep.simxFinish(-1); 
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
ppose=[0 0 0];
position=[0 0 0];
t=5;
% Start pose
x0 = 0;
y0 = 0;
theta0 = 0;
x0_sim=[0,x0];
y0_sim=[0,y0];
theta0_sim=[0,theta0];
 if (clientID>-1)
    disp('Connected');
    [returnCode,Robot]=vrep.simxGetObjectHandle(clientID,'Robot',vrep.simx_opmode_blocking );
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking );
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking );
    [returnCode,time]=vrep.simxGetFloatSignal(clientID,'SimulationTime',vrep.simx_opmode_streaming);
    [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_streaming)
    run = true;
    [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_buffer)
    %pause(0.05);
    while run
    [returnCode,time]=vrep.simxGetFloatSignal(clientID,'SimulationTime',vrep.simx_opmode_buffer);
    if time>t &&time <=2+t
    [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_buffer)
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0.1/wheel_radifront,vrep.simx_opmode_blocking);    
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,right_Motor,0.1/wheel_radifront,vrep.simx_opmode_blocking);
    ppose=[ppose;position];
    
    %pause(0.05)
    elseif time>2+t && time<=5+t
     [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_buffer)
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0.05/wheel_radifront,vrep.simx_opmode_blocking);    
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,right_Motor,0.1/wheel_radifront,vrep.simx_opmode_blocking);
    ppose=[ppose;position];
    
   % pause(0.05)
 
    elseif time >5+t&& time <9+t
     [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_buffer)
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0.1/wheel_radifront,vrep.simx_opmode_blocking);    
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,right_Motor,0.05/wheel_radifront,vrep.simx_opmode_blocking);
    ppose=[ppose;position];
    
   % pause(0.05)
    elseif time>=9+t
    run= false;
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, 0 ,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, 0 ,vrep.simx_opmode_blocking);
    end
    end
    %[returnCode]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
    % Run the simulation
    sim('Testkinematic');   
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
    
     % Stop the simulation
    vrep.simxFinish(-1);
 
else
    % Connection Failed
    disp('Failed connecting to remote API server')
end

% 