close all  % close all opening figure windows
clear % Clear all variables in workspace
clc
global pposetrue ppose GGPS
wheel_radifront=0.03;
vrep=remApi('remoteApi');
vrep.simxFinish(-1); 
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
ppose=[];
pposetrue=[]
position=[0 0 0];
t=10;
initPos=[0,0,0];
sample = 100;
Accel_sample = zeros(sample,3);
Pos_sample =zeros(sample,3);
getSample = 1;
Pos_sample(getSample,:)=initPos
checkFull = false;
delta_t=0.05;
run = true;
GGPS=[];
GPSs=[];
 if (clientID>-1)
    disp('Connected');
    [returnCode,Orobot]=vrep.simxGetObjectHandle(clientID,'Robotpose',vrep.simx_opmode_blocking)
    [returnCode,Robot]=vrep.simxGetObjectHandle(clientID,'Robot',vrep.simx_opmode_blocking );
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking );
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking );
    [returnCode,time]=vrep.simxGetFloatSignal(clientID,'SimulationTime',vrep.simx_opmode_streaming);
    [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_streaming)
    [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_streaming);
    [returnCode,GPS(1)]=vrep.simxGetFloatSignal(clientID,'GPS1',vrep.simx_opmode_streaming);
    [returnCode,GPS(2)]=vrep.simxGetFloatSignal(clientID,'GPS2',vrep.simx_opmode_streaming);
    [returnCode,GPS(3)]=vrep.simxGetFloatSignal(clientID,'GPS3',vrep.simx_opmode_streaming);
    [returnCode,Accel(1)]=vrep.simxGetFloatSignal(clientID,'Accel1',vrep.simx_opmode_streaming);
    [returnCode,Accel(2)]=vrep.simxGetFloatSignal(clientID,'Accel2',vrep.simx_opmode_streaming);
    [returnCode,Accel(3)]=vrep.simxGetFloatSignal(clientID,'Accel3',vrep.simx_opmode_streaming);
    [returnCode,Vel(1)]=vrep.simxGetFloatSignal(clientID,'Velocity1',vrep.simx_opmode_streaming);
    [returnCode,Vel(2)]=vrep.simxGetFloatSignal(clientID,'Velocity2',vrep.simx_opmode_streaming);
    [returnCode,Vel(3)]=vrep.simxGetFloatSignal(clientID,'Velocity3',vrep.simx_opmode_streaming);
    [returnCode,observed_Vel,]=vrep.simxGetObjectVelocity(clientID,Orobot,vrep.simx_opmode_streaming);
%    [returnCode,observed_Vel,]=vrep.simxGetObjectVelocity(clientID,robot,vrep.simx_opmode_streaming);
%   
    pause(0.05);
    while run== true
    [returnCode,time]=vrep.simxGetFloatSignal(clientID,'SimulationTime',vrep.simx_opmode_buffer);
    [returnCode,GPS(1)]=vrep.simxGetFloatSignal(clientID,'GPS1',vrep.simx_opmode_buffer);
    [returnCode,GPS(2)]=vrep.simxGetFloatSignal(clientID,'GPS2',vrep.simx_opmode_buffer);
    [returnCode,GPS(3)]=vrep.simxGetFloatSignal(clientID,'GPS3',vrep.simx_opmode_buffer);
    [returnCode,Accel(1)]=vrep.simxGetFloatSignal(clientID,'Accel1',vrep.simx_opmode_buffer);
    [returnCode,Accel(2)]=vrep.simxGetFloatSignal(clientID,'Accel2',vrep.simx_opmode_buffer);
    [returnCode,Accel(3)]=vrep.simxGetFloatSignal(clientID,'Accel3',vrep.simx_opmode_buffer);
    [returnCode,Vel(1)]=vrep.simxGetFloatSignal(clientID,'Velocity1',vrep.simx_opmode_buffer);
    [returnCode,Vel(2)]=vrep.simxGetFloatSignal(clientID,'Velocity2',vrep.simx_opmode_buffer);
    [returnCode,Vel(3)]=vrep.simxGetFloatSignal(clientID,'Velocity3',vrep.simx_opmode_buffer);
    %[returnCode,position]=vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_streaming);
    [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
    if checkFull ==false
        pause(0.05);
        Pos_sample(getSample,:)=GPS;
        Accel_sample(getSample,:)=Accel;
        getSample = getSample + 1
        if getSample > sample 
        checkFull = true
        [R,Accel_cov_mat] = getCovMat(Pos_sample,Accel_sample)
        init_state = GPS;
        init_state(4) = 0;
        init_state(5) = 0;
        init_state(6) = 0;
        prev_state = transpose(init_state);
        prev_P =[1,0,0,1,0,0,
                0,1,0,0,1,0,
                0,0,1,0,0,1,
                1,0,0,1,0,0,
                0,1,0,0,1,0,
                0,0,1,0,0,1];
        end
    elseif checkFull == true
        %Kaman predict
        A = [1,0,0,delta_t,0,0,
            0,1,0,0,delta_t,0,
            0,0,1,0,0,delta_t,
            0,0,0,1,0,0,
            0,0,0,0,1,0,
            0,0,0,0,0,1]
        B = [(1/2)*(delta_t^2),0,0,
            0,(1/2)*(delta_t^2),0,
            0,0,(1/2)*(delta_t^2),
            delta_t,0,0,
            0,delta_t,0,
            0,0,delta_t]
        Q = [(1/4)*(delta_t^4),0,0,(1/2)*(delta_t^3),0,0,
            0,(1/4)*(delta_t^4),0,0,(1/2)*(delta_t^3),0,
            0,0,(1/4)*(delta_t^4),0,0,(1/2)*(delta_t^3),
            (1/2)*(delta_t^3),0,0,delta_t,0,0,
            0,(1/2)*(delta_t^3),0,0,delta_t,0,
            0,0,(1/2)*(delta_t^3),0,0,delta_t]*(Accel_cov_mat)
        [returnCode,Accel(1)]=vrep.simxGetFloatSignal(clientID,'Accel1',vrep.simx_opmode_buffer);
        [returnCode,Accel(2)]=vrep.simxGetFloatSignal(clientID,'Accel2',vrep.simx_opmode_buffer);
        [returnCode,Accel(3)]=vrep.simxGetFloatSignal(clientID,'Accel3',vrep.simx_opmode_buffer);   
        control_vec = transpose(Accel);
        xk = A*(prev_state)+B*control_vec;
        [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
       % pose =[position(1,1),position(1,2),orientation(1,3)];
        pose =[xk(1),xk(2),orientation(1,3)];
        P = A*(prev_P)*transpose(A)+(Q)
        P(1,2)=0;
        P(1,3)=0;
        P(1,5)=0;
        P(1,6)=0;
        P(2,1)=0;
        P(2,3)=0;
        P(2,4)=0;
        P(2,6)=0;
        P(3,1)=0;
        P(3,2)=0;
        P(3,4)=0;
        P(3,5)=0;
        P(4,2)=0;
        P(4,3)=0;
        P(4,5)=0;
        P(4,6)=0;
        P(5,1)=0;
        P(5,3)=0;
        P(5,4)=0;
        P(5,6)=0;
        P(6,1)=0;
        P(6,2)=0;
        P(6,4)=0;
        P(6,5)=0;
    
    if time>t &&time <=5+t
%    [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_buffer)
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0.2/wheel_radifront,vrep.simx_opmode_blocking);    
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,right_Motor,0.1/wheel_radifront,vrep.simx_opmode_blocking);
    elseif time>t+5 &&time <=10+t
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0.1/wheel_radifront,vrep.simx_opmode_blocking);    
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,right_Motor,0.1/wheel_radifront,vrep.simx_opmode_blocking);   
    elseif time >10+t   
        run= false
    end
    [returnCode,datapose]=vrep.simxGetStringSignal(clientID,'Robotpose',vrep.simx_opmode_buffer);
    [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_buffer)
    truepose =[position(1,1),position(1,2),orientation(1,3)];
    ppose=[ppose;pose];
    pposetrue=[pposetrue;truepose];
    GPSs=[GPS(1),GPS(2)];
    GGPS=[GGPS;GPSs];
     H = eye(6);
    [returnCode,GPS(1)]=vrep.simxGetFloatSignal(clientID,'GPS1',vrep.simx_opmode_buffer);
    [returnCode,GPS(2)]=vrep.simxGetFloatSignal(clientID,'GPS2',vrep.simx_opmode_buffer);
    [returnCode,GPS(3)]=vrep.simxGetFloatSignal(clientID,'GPS3',vrep.simx_opmode_buffer);
    observed = [GPS(1),GPS(2),GPS(3)];
    [returnCode,Vel(1)]=vrep.simxGetFloatSignal(clientID,'Velocity1',vrep.simx_opmode_buffer);
    [returnCode,Vel(2)]=vrep.simxGetFloatSignal(clientID,'Velocity2',vrep.simx_opmode_buffer);
    [returnCode,Vel(3)]=vrep.simxGetFloatSignal(clientID,'Velocity3',vrep.simx_opmode_buffer);    
    observed(4) = Vel(1);
    observed(5) = Vel(2);
    observed(6) = Vel(3);
    z = transpose(observed);
    y = z-H*xk;
    K = P*transpose(H)/(H*P*transpose(H)+R);         
    prev_state = xk+K*y
    prev_P = (eye(6)-K*H)*P;

    end
    end
    
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking);    
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,right_Motor,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)

     % Stop the simulation
    vrep.simxFinish(-1);
else
    % Connection Failed
    disp('Failed connecting to remote API server')
 end
function [R,Accel_cov_mat]= getCovMat(Pos_sample,Accel_sample)
     Pos_deviation = Pos_sample - ones(length(Pos_sample),length(Pos_sample))*(Pos_sample)*(1/length(Pos_sample))
     Pos_cov = transpose(Pos_deviation)*(Pos_deviation)
     Accel_deviation = Accel_sample - ones(length(Accel_sample),length(Accel_sample))*(Accel_sample)*(1/length(Accel_sample))
     Accel_cov = transpose(Accel_deviation)*(Accel_deviation)
     R = diag([Pos_cov(1,1),Pos_cov(2,2),Pos_cov(3,3),0.1,0.1,0.1])
     Accel_cov_mat = diag([Accel_cov(1,1),Accel_cov(2,2),Accel_cov(3,3),Accel_cov(1,1),Accel_cov(2,2),Accel_cov(3,3)]) 
end

% 