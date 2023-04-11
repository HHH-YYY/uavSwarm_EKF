
classdef distributedFollowerControl < control
    %CONTROLLER Abstract class of robot controllers
    properties
        dcm %direction cosine matrix
        horVel %Horizontal velocity
        droneNo %Drone No.
        numOfFollower %Number of follower per drone according to graph
        currentTime
        initialPos
        tempTruePos
    end
    
    methods
        function obj = distributedFollowerControl(drones, ...
                                                  leader, ...
                                                  formation, ...
                                                  droneNo, ...
                                                  accuracy, ...
                                                  currentTime,...
                                                  tempTruePos )
            obj.currentTime = currentTime;
            obj.formation = formation;
            obj.drones = drones;
            obj.leader = leader;
            obj.numOfDrones = formation.numOfDrones;
            obj.tempTruePos = tempTruePos;
            [motion,~] = read(leader);

            %Acquisition of noisy data from GNSS.
            motionForNoisy = struct( ...
                                    'Position',motion(:,1:3), ...
                                    'Velocity', motion(:,4:6), ...
                                    'Acceleration', motion(:,7:9), ...
                                    'Orientation', motion(:,4:6), ...
                                    'AngularVelocity', motion(:,14:16));
            motionWithNoisy = generatingNoiseFromGNSS(motionForNoisy,accuracy);
            obj.position = motionWithNoisy(1,:);

            %print GPS location of leader
            obj.velocity = motionWithNoisy(2,:);
            obj.acceleration = motionWithNoisy(3,:);
            obj.angularVelocity = motionWithNoisy(4,:);
            obj.quaternion = motion(:,10:13);
            
%             %为了产生GPS的生成的位置和速度
%             [pos, vel]=generatingNoseFromGPS(motion(:,1:3),motion(:,4:6));
% 
%             %为了产生IMU的加速度
%             [accel,mag] = generatingNoseFromIMU(motion(:,7:9),motion(:,14:16),quat2rotm(motion(:,10:13)));
%             accel
%             
%             obj.position = pos;
%             obj.velocity = vel;
%             obj.acceleration = accel;
%             obj.quaternion = motion(:,10:13);
%             obj.angularVelocity = motion(:,14:16);


            %记录leader的数据，记录follower的数据在下面的方法followerMove()里
            if droneNo ==1
                obj.tempTruePos = [obj.tempTruePos [obj.position obj.velocity obj.acceleration obj.quaternion obj.angularVelocity]];
            end 

            obj.horVel = [motion(4) motion(5) 0];
            v1 = obj.horVel;
            v2 = [1 0 0];
            obj.dcm = getDcmFrom2Vectors(v1,v2);
            obj.droneNo = droneNo;
            [obj.numOfFollower,~] = size(obj.formation.graph.successors(droneNo));
        end
        
        function folTruePos = followerMove(obj)
            for i = 1:obj.numOfFollower
                paths = obj.formation.graph.successors(obj.droneNo);
                pos = calculatAbsPos(obj.position, ...
                                     obj.formation.relPos(paths(i),:), ...
                                     obj.dcm);
%                 [pos, vel]=generatingNoseFromGPS(pos,obj.velocity);
%                 [acc,mag] = generatingNoseFromIMU(obj.acceleration,obj.angularVelocity,quat2rotm(obj.quaternion));
                move(obj.drones(paths(i)),[pos, ...
                                           obj.velocity, ...
                                           obj.acceleration, ...
                                           obj.quaternion, ...
                                           obj.angularVelocity])
                %记录follower的数据
               obj.tempTruePos = [obj.tempTruePos [pos obj.velocity obj.acceleration obj.quaternion obj.angularVelocity]];
            end
            %为了返回记录
            folTruePos = obj.tempTruePos;
        end

    end
end