    vrep=remApi('remoteApi');
    vrep.simxFinish(-1);
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    pose = [1,1,0];
    tireDiameter_m = 0.03;
    d = 0.0823;
    v = 0.1;
    pathx=path(:,1)/100;
    pathy=path(:,2)/100;
    goalPoints=[pathx pathy]
    ii = 1;
    lastGoal = pose(1:2);
    currentGoal = goalPoints(ii, :);
    atFinalGoal = 0;
    robPose(1,:)= pose;
    position=[1.000,1.000];
    state =1;
    wheel_radifront=0.035
    b=0.0823
    vTurn=0.03
    goalDetectedTol=1*pi/180;
    vref=0.1
    ppose=[]
    Oppose=[]
    delta_t=0.05
if (clientID>-1)
    % If connection successful
    disp('Connected')
    % Create handles for required V-Rep objects
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking);
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking);
    [returnCode,Orobot]=vrep.simxGetObjectHandle(clientID,'Robotpose',vrep.simx_opmode_blocking)
    [returnCode,Orierobot]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_streaming)
    [returnCode,Posirobot]=vrep.simxGetObjectPosition(clientID,Orobot,-1,vrep.simx_opmode_streaming)


    
    while(~atFinalGoal)
        [returnCode,datapose]=vrep.simxGetStringSignal(clientID,'Robotpose',vrep.simx_opmode_buffer)
        if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        position=vrep.simxUnpackFloats(datapose)
        end
       % [returnCode,position]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
        [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,Orobot,-1,vrep.simx_opmode_buffer)
        pose=[position(1,1),position(1,2),orientation(1,3)];
        if(state==1)
            %TODO:Check if it is pointing to goal
            [pointing,direction] = PointingToGoal(pose,lastGoal,currentGoal,goalDetectedTol);
                if pointing == true
                state=2
                end
            elseif (state==2)
            %TODO:Check if it is pointing to goal
            %TODO:Check if goal has been reached
            %TODO:Check if there's an object in front
            [pointing,direction] = PointingToGoal(pose,lastGoal,currentGoal,goalDetectedTol);
            atGoal=atGoalPoint(pose, currentGoal)    
                if pointing==false
                    state=1;
                elseif  atGoal ==true
                    state=3;
                end
            elseif (state==3)
                if(~atFinalGoal)
                    state =1;
                end
        end
        if(state==1)
            [wTurnL,wTurnR,turnTime]=precomputeTurn(vTurn,direction,wheel_radifront,b)
            wL=wTurnL;
            wR=wTurnR;
        elseif(state==2)
            wL=vref/wheel_radifront;
            wR=vref/wheel_radifront;
            t=0;
        elseif(state==3)
            wL=0
            wR=0
        end
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, wL,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, wR ,vrep.simx_opmode_blocking);
        % Update goal points if target goal reached
        atGoal=atGoalPoint(pose, currentGoal) 
        if atGoal ==true
            if(ii < length(goalPoints))  
                % increment goal point
                lastGoal = currentGoal;
                ii = ii + 1;
                currentGoal = goalPoints(ii, :);         
            else
                % No more goal points in list; at final goal
                atFinalGoal = 1;
            end
        end
        [returnCode,datapose]=vrep.simxGetStringSignal(clientID,'Robotpose',vrep.simx_opmode_buffer)
        if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        position=vrep.simxUnpackFloats(datapose)
        end
        [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,Orobot,-1,vrep.simx_opmode_buffer)
        pose=[position(1,1),position(1,2),orientation(1,3)];
        %state=1;
       
        SL= wL* delta_t*wheel_radifront;
        SR= wR*delta_t*wheel_radifront;
        deltaS=(SL+SR)/2;
        deltatheta=(SR-SL)/(2*b)
        x=pose(1)+deltaS*cos(pose(3));
        y=pose(2)+deltaS*sin(pose(3));
        theta=pose(3)+ deltatheta;
        Odometrypose=[x,y,theta];
        %update to plot
        Oppose=[Oppose;Odometrypose];
        ppose=[ppose;position];
    end
     vrep.simxFinish(-1);
     %[returnCode]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)

else
    % Connection Failed
    disp('Failed connecting to remote API server')
end

function [atGoal] = atGoalPoint(robotPose, goalPoint)
    goalRadius_m = 0.1;
    dist = sqrt((robotPose(1) - goalPoint(1))^2 + (robotPose(2) - goalPoint(2))^2);
    if abs(dist) < goalRadius_m
         atGoal = true;
    else atGoal=false;
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
function [pointing,direction] = PointingToGoal(pose,lastGoal,currentGoal,tolerance)
    %pose:list with x,y,theta values of therobot
    %initPose:list with x,y of the initial position
    %goalPose:list with x,y of the goal position
    %tolerance:admissible angle(in radians)
    %Returns true or false and the turning direction(1 or -1)
    angle_diff=(atan2(currentGoal(1,2)-lastGoal(1,2),currentGoal(1,1)-lastGoal(1,1))-pose(1,3));
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