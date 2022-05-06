close all;
clear;
clc;
global  wL wR pose
global pposetrue ppose Oppose GGPS initPos goalPos
GPS=[];
Accel=[];
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    % If connection successful
    disp('Connected')
    % Create handles for required V-Rep objects
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking);
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking);
    [returnCode,front_Sensor]=vrep.simxGetObjectHandle(clientID,'front_prox',vrep.simx_opmode_blocking);
    [returnCode,front_right]=vrep.simxGetObjectHandle(clientID,'front_right',vrep.simx_opmode_blocking);
    [returnCode,rear_right]=vrep.simxGetObjectHandle(clientID,'rear_right',vrep.simx_opmode_blocking);
    [returnCode,front_left]=vrep.simxGetObjectHandle(clientID,'front_left',vrep.simx_opmode_blocking);
    [returnCode,rear_left]=vrep.simxGetObjectHandle(clientID,'rear_left',vrep.simx_opmode_blocking);
    [returnCode,init]=vrep.simxGetObjectHandle(clientID,'init',vrep.simx_opmode_blocking)
    [returnCode,goal]=vrep.simxGetObjectHandle(clientID,'goal',vrep.simx_opmode_blocking)
    [returnCode,Orobot]=vrep.simxGetObjectHandle(clientID,'Robotpose',vrep.simx_opmode_blocking)
    [returnCode,robot]=vrep.simxGetObjectHandle(clientID,'.',vrep.simx_opmode_blocking)
    % Initialise sensors
    [returnCode,detectionState,distanceFront,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionState,dFR,~,~]=vrep.simxReadProximitySensor(clientID,front_right,vrep.simx_opmode_streaming);
    [returnCode,detectionState,dRR,~,~]=vrep.simxReadProximitySensor(clientID,rear_right,vrep.simx_opmode_streaming);
    [returnCode,detectionState,dFL,~,~]=vrep.simxReadProximitySensor(clientID,front_left,vrep.simx_opmode_streaming);
    [returnCode,detectionState,dRL,~,~]=vrep.simxReadProximitySensor(clientID,rear_left,vrep.simx_opmode_streaming);
    [returnCode,Orierobot]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_streaming)
    [returnCode,initPos]=vrep.simxGetObjectPosition(clientID,init,-1,vrep.simx_opmode_streaming)
    [returnCode,goalPos]=vrep.simxGetObjectPosition(clientID,goal,-1,vrep.simx_opmode_streaming)
    [returnCode,time]=vrep.simxGetFloatSignal(clientID,'SimulationTime',vrep.simx_opmode_streaming);
    [returnCode,GPS(1)]=vrep.simxGetFloatSignal(clientID,'GPS1',vrep.simx_opmode_streaming);
    [returnCode,GPS(2)]=vrep.simxGetFloatSignal(clientID,'GPS2',vrep.simx_opmode_streaming);
    [returnCode,GPS(3)]=vrep.simxGetFloatSignal(clientID,'GPS3',vrep.simx_opmode_streaming);
    [returnCode,Gyro]=vrep.simxGetFloatSignal(clientID,'Gyrodata',vrep.simx_opmode_streaming);
    [returnCode,Accel(1)]=vrep.simxGetFloatSignal(clientID,'Accel1',vrep.simx_opmode_streaming);
    [returnCode,Accel(2)]=vrep.simxGetFloatSignal(clientID,'Accel2',vrep.simx_opmode_streaming);
    [returnCode,Accel(3)]=vrep.simxGetFloatSignal(clientID,'Accel3',vrep.simx_opmode_streaming);
    [returnCode,Vel(1)]=vrep.simxGetFloatSignal(clientID,'Velocity1',vrep.simx_opmode_streaming);
    [returnCode,Vel(2)]=vrep.simxGetFloatSignal(clientID,'Velocity2',vrep.simx_opmode_streaming);
    [returnCode,Vel(3)]=vrep.simxGetFloatSignal(clientID,'Velocity3',vrep.simx_opmode_streaming);
    [returnCode,observed_Vel,]=vrep.simxGetObjectVelocity(clientID,Orobot,vrep.simx_opmode_streaming);
    [returnCode,observed_Vel,]=vrep.simxGetObjectVelocity(clientID,robot,vrep.simx_opmode_streaming);
    %Constructiveparameter
    orientation=[0,0,0];
    initPos=[1.025,1.925,0];
    goalPos=[-1.3,-2.025,0];
    position=initPos;
    wheel_radifront=0.03  %wheel radifront
    b=0.0823  %wheel base(wheel separation distance)
    a=0.03  %Separation distance between front and rear prox sensors
    vref=0.08  %velocity when following the wall moving forward
    vTurn=0.01 %velocity when turning
    v=0.1;
    objectDist=0.2  %distance to stop the robot when detecting an object
    min_distfront= objectDist  %Distance to stop when there's a wall in front of the robot
    kWall=100%Wall following gain
    e=0.5 %Distance of an off-center point toper form the kinematic control for wallfollowing
    wallDist=objectDist%-sepprox  %The expected distance to detect a wall with prox sensors
    dWallSide=0.3;
    goalDetectedTol=10*pi/180;  %Tolerance to point to the goal
    wallDetectedTol=0.5  %Tolerance to determine that there's a wall when rotating
    goalReachedTol=0.1   %Tolerance to determine the robot has reached the goal
    States={"Pointtogoal","Moveforward","Turn","Followwalllinenotcrossed","Followwall","Stop"}  %Namesforeachdefinedstateofthealgorithm
    state=1  %Initial state
    direction=1  %Follow right 1,follow left -1
    max_distfront=0.5  %Maximum distance returned by the front sensor
    d_front=max_distfront  %Initial front distance
    max_distprox=1  %Maximum distance returned by the prox sensors
    dFR=max_distprox  %Initial prox distance
    dRR=max_distprox  %Initial prox distance
    dFL=max_distprox  %Initial prox distance
    dRL=max_distprox  %Initial prox distance
    turnTime=0;
    checkGoal=false;
    i=1;
    Oppose=[]; %updatetoplot
    ppose=[];%updatetoplot
    pposetrue=[];%updatetoplot
    GGPS=[];%updatetoplot
    GPSs=[];%updatetoplot
    %
    sample = 100
    Accel_sample = zeros(sample,3);
    Pos_sample =zeros(sample,3);
    getSample = 1;
    Pos_sample(getSample,:)=initPos
    checkFull = false;
    delta_t = 0.05;
    counter=1;
    i=1;
    %Bug algorithm
    while ~checkGoal
    %Read proximity sensors
    [returnCode,detectionState,distanceFront,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer);
    [returnCode,detectionState,dFR,~,~]=vrep.simxReadProximitySensor(clientID,front_right,vrep.simx_opmode_buffer);
    [returnCode,detectionState,dRR,~,~]=vrep.simxReadProximitySensor(clientID,rear_right,vrep.simx_opmode_buffer);
    [returnCode,detectionState,dFL,~,~]=vrep.simxReadProximitySensor(clientID,front_left,vrep.simx_opmode_buffer);
    [returnCode,detectionState,dRL,~,~]=vrep.simxReadProximitySensor(clientID,rear_left,vrep.simx_opmode_buffer);
     distanceFront=distanceFront(1,3);
     dFR=dFR(1,3);
     dRR=dRR(1,3);
     dFL=dFL(1,3);
     dRL=dRL(1,3);
     [returnCode,time]=vrep.simxGetFloatSignal(clientID,'SimulationTime',vrep.simx_opmode_buffer);
     [returnCode,GPS(1)]=vrep.simxGetFloatSignal(clientID,'GPS1',vrep.simx_opmode_buffer);
     [returnCode,GPS(2)]=vrep.simxGetFloatSignal(clientID,'GPS2',vrep.simx_opmode_buffer);
     [returnCode,GPS(3)]=vrep.simxGetFloatSignal(clientID,'GPS3',vrep.simx_opmode_buffer);
     [returnCode,Gyro]=vrep.simxGetFloatSignal(clientID,'Gyrodata',vrep.simx_opmode_buffer);
     [returnCode,Accel(1)]=vrep.simxGetFloatSignal(clientID,'Accel1',vrep.simx_opmode_buffer);
     [returnCode,Accel(2)]=vrep.simxGetFloatSignal(clientID,'Accel2',vrep.simx_opmode_buffer);
     [returnCode,Accel(3)]=vrep.simxGetFloatSignal(clientID,'Accel3',vrep.simx_opmode_buffer);
     [returnCode,Vel(1)]=vrep.simxGetFloatSignal(clientID,'Velocity1',vrep.simx_opmode_buffer);
     [returnCode,Vel(2)]=vrep.simxGetFloatSignal(clientID,'Velocity2',vrep.simx_opmode_buffer);
     [returnCode,Vel(3)]=vrep.simxGetFloatSignal(clientID,'Velocity3',vrep.simx_opmode_buffer);
     %Get Robotpose
    vrep.simxGetStringSignal(clientID,'Robotpose',vrep.simx_opmode_streaming);
    vrep.simxGetStringSignal(clientID,'init',vrep.simx_opmode_streaming);
    vrep.simxGetStringSignal(clientID,'goal',vrep.simx_opmode_streaming);
    [returnCode,datapose]=vrep.simxGetStringSignal(clientID,'Robotpose',vrep.simx_opmode_buffer);
    [returnCode,datainit]=vrep.simxGetStringSignal(clientID,'init',vrep.simx_opmode_buffer);
    [returnCode,datagoal]=vrep.simxGetStringSignal(clientID,'goal',vrep.simx_opmode_buffer);
    if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        position=vrep.simxUnpackFloats(datapose)
    end
    %[returnCode,position]=vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_streaming);
    [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
    truepose =[position(1,1),position(1,2),orientation(1,3)];
    %init state
    if (checkFull == false)
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
    %State machine
    if(state==1)
    %TODO:Check if it is pointing to goal
    [pointing,direction] = PointingToGoal(pose,initPos,goalPos,goalDetectedTol);
        if pointing == true
        state=2
        end
    elseif (state==2)
    %TODO:Check if it is pointing to goal
    %TODO:Check if goal has been reached
    %TODO:Check if there's an object in front
    [pointing,direction] = PointingToGoal(pose,initPos,goalPos,goalDetectedTol);
    checkGoal=GoalReached(pose,goalPos,goalReachedTol);
    [returnCode,detectionState,distanceFront,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer);
    distanceFront=distanceFront(1,3);    
        if checkGoal==true
            state=6;
        elseif pointing==false
            state=1;
        elseif  (distanceFront > 0.01) && (distanceFront <= min_distfront)
            direction=selectRandomDirection()
            %direction=1;
            state=3
        end
    elseif(state==3)
    %TODO:Check wall on the side
    [pointing,direction] = PointingToGoal(pose,initPos,goalPos,goalDetectedTol);
    [front,rear] = wallDetected(direction,dFL,dRL,dFR,dRR,dWallSide,wallDetectedTol);
        if (front ==true) || (rear == true)
            state=4
        end
    elseif(state==4)
    %TODO:Check side of theline
    [pointing,direction] = PointingToGoal(pose,initPos,goalPos,goalDetectedTol);    
    lineSide= LineSide(pose,initPos,goalPos);
        if lineSide==direction
            state=5
        end
    elseif(state==5)
    %TODO:Check side of theline
    lineSide= LineSide(pose,initPos,goalPos);
        if lineSide~=direction
            state=1
        end
    end
    [wL,wR,turnTime]=checkstate(state,a,kWall,vTurn,vref,wheel_radifront,direction,b,dFL,dRL,dFR,dRR,wallDist,e);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, wL ,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, wR ,vrep.simx_opmode_blocking);
    %pause(turnTime);
    %Odometry
    SL= wL* delta_t*wheel_radifront;
    SR= wR*delta_t*wheel_radifront;
    deltaS=(SL+SR)/2;
    deltatheta=(SR-SL)/(2*b)
    x=truepose(1)+deltaS*cos(truepose(3));
    y=truepose(2)+deltaS*sin(truepose(3));
    theta=truepose(3)+ deltatheta;
    Odometrypose=[x,y,theta];
    %update to plot
    Oppose=[Odometrypose;Oppose]; 
    ppose=[pose ;ppose];
    pposetrue=[truepose;pposetrue];
    GPSs=[GPS(1),GPS(2)];
    GGPS=[GPSs;GGPS];

 %Kaman update
    H = eye(6);
    [returnCode,GPS(1)]=vrep.simxGetFloatSignal(clientID,'GPS1',vrep.simx_opmode_streaming);
    [returnCode,GPS(2)]=vrep.simxGetFloatSignal(clientID,'GPS2',vrep.simx_opmode_streaming);
    [returnCode,GPS(3)]=vrep.simxGetFloatSignal(clientID,'GPS3',vrep.simx_opmode_streaming);
    observed = [GPS(1),GPS(2),GPS(3)];
%    [returnCode,observed_Vel,~]=vrep.simxGetObjectVelocity(clientID,Orobot,vrep.simx_opmode_buffer);
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
    %[returnCode]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)

     % Stop the simulation
    vrep.simxFinish(-1);
 
else
    % Connection Failed
    disp('Failed connecting to remote API server')
end
 
function [wL,wR,t]=checkstate(state,a,kWall,vTurn,vref,wheel_radifront,direction,b,dFL,dRL,dFR,dRR,wallDist,e)
    if(state==1)
        [wTurnL,wTurnR,turnTime]=precomputeTurn(vTurn,direction,wheel_radifront,b)
        wL=wTurnL;
        wR=wTurnR;
        t=turnTime;
    elseif(state==2)
        wL=vref/wheel_radifront;
        wR=vref/wheel_radifront;
        t=0;
    elseif(state==3)
        [wTurnL,wTurnR,turnTime]=precomputeTurn(vTurn,direction,wheel_radifront,b);
        wL=wTurnL;
        wR=wTurnR;
        t=turnTime;
    elseif(state==4)
        [wL,wR]=followWall(vref,a,direction,dFL,dRL,dFR,dRR,wallDist,kWall,wheel_radifront,b,e);
        t=0;
    elseif(state==5)
        [wL,wR]=followWall(vref,a,direction,dFL,dRL,dFR,dRR,wallDist,kWall,wheel_radifront,b,e);
        t=0;
    elseif(state==6)
        wL=0
        wR=0
        t=0;
    end
end

function [pointing,direction] = PointingToGoal(pose,initPos,goalPos,tolerance)
    %pose:list with x,y,theta values of therobot
    %initPose:list with x,y of the initial position
    %goalPose:list with x,y of the goal position
    %tolerance:admissible angle(in radians)
    %Returns true or false and the turning direction(1 or -1)
    angle_diff=(atan2(goalPos(1,2)-initPos(1,2),goalPos(1,1)-initPos(1,1))-pose(1,3));
    if(abs(angle_diff)<tolerance)
        pointing=true
    else
        pointing=false
    end
    if(angle_diff>0)
        direction= +1
    else
        direction=-1
    end
end

function [wL,wR,t]=precomputeTurn(vturn,direction,wheel_radifront,b)
    %vturn:turning speed
    %direction:1->Follow right,-1->Follow left
    %wheel_radifront,b:constructive parameters
    %Returns wheels velocities and turningtime
    wL=-direction*vturn/wheel_radifront
    wR=direction*vturn/wheel_radifront;
    t=b*(pi/2)/vturn
end

function [front,rear] = wallDetected(direction,dFL,dRL,dFR,dRR,dWallSide,tolerance)
    %direction:1 wall on the right,-1 wall on the left
    %dFL,dRL:distance of prox sensors on the left
    %dFR,dRR:distance of prox sensors on the right
    %dWallSide:expected separation distance of the wall
    %tolerance:admissible distance to consider that there's a wall
    %Returns true or false
    if(direction==-1)
    dF=dFR
    dR=dRR
    else
    dF=dFL
    dR=dRL
    end
    dfront=abs(dF-dWallSide)
    drear=abs(dR-dWallSide)
    if dfront <tolerance
        front = true
    else front = false
    end
    if drear < tolerance
        rear = true;
    else rear =false
    end
end

function [wL,wR]= followWall(v,a,direction,dFL,dRL,dFR,dRR,dWallSide,kWall,wheel_radifront,b,e)
    %dFL,dRL:distance of prox sensors on the left
    %dFR,dRR:distance of prox sensors on the right
    %dWallSide:expected separation distance of the wall
    %wheel_radifront,a,b:Constructuve parameters
    %v,kWall,e:Parameters for wallfollowing algorithm
    %Returns wheels velocities
    if(direction==1)  %follow right wall
    phi=atan((dFR-dRR)/a)
    d=(0.5*(dFR+dRR)-dWallSide)
    else  %follow left wall
    phi=atan((dFL-dRL)/a)
    d=(dWallSide-0.5*(dFL+dRL))
    end
    gamma=kWall*d
    alpha=phi+gamma
    wL=(v/wheel_radifront)*(cos(alpha)+(b/e)*sin(alpha));
    wR=(v/wheel_radifront)*(cos(alpha)-(b/e)*sin(alpha))
end

function  [lineSide]= LineSide(pose,initPos,goalPos)
    %pose:list with x,y,theta values of the robot
    %initPose:list with x,y of the initial position
    %goalPose:listwith x,y of the goal position
    %Returns 1 on the left side and -1 on the right side
    xDiff1=initPos(1,1)-goalPos(1,1)
    yDiff1=goalPos(1,2)-initPos(1,2)
    xDiff2=initPos(1,1)-pose(1,1)
    yDiff2=initPos(1,2)-pose(1,2)
    dGoalIni=sqrt(xDiff1^2+yDiff1^2)
    d=(xDiff2*yDiff1+yDiff2*xDiff1)/dGoalIni
    if d>0
        lineSide = 1
    else
        lineSide = -1
    end
end

function checkGoal=GoalReached(pose,goalPos,tolerance)
    %pose:list with x,y,theta values of the robot
    %goalPose:list with x,y of the goal position
    %tolerance:admissible distance to consider that the goal has been reached
    %Returns true or false
    dGoal=sqrt((goalPos(1,1)-pose(1,1))^2+(goalPos(1,2)-pose(1,2))^2);
    if dGoal<tolerance
        checkGoal =true;
    else checkGoal =false
    end
end
function [direction] = selectRandomDirection()
    n=random('Normal',0,1);
    if(n<0.5)
    direction=1;
    else
    direction=-1;
    end
end
function [R,Accel_cov_mat]= getCovMat(Pos_sample,Accel_sample)
     Pos_deviation = Pos_sample - ones(length(Pos_sample),length(Pos_sample))*(Pos_sample)*(1/length(Pos_sample))
     Pos_cov = transpose(Pos_deviation)*(Pos_deviation)
     Accel_deviation = Accel_sample - ones(length(Accel_sample),length(Accel_sample))*(Accel_sample)*(1/length(Accel_sample))
     Accel_cov = transpose(Accel_deviation)*(Accel_deviation)
     R = diag([Pos_cov(1,1),Pos_cov(2,2),Pos_cov(3,3),0.1,0.1,0.1])
     Accel_cov_mat = diag([Accel_cov(1,1),Accel_cov(2,2),Accel_cov(3,3),Accel_cov(1,1),Accel_cov(2,2),Accel_cov(3,3)]) 
end

 
