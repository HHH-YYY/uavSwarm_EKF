classdef scenario
    
    properties
        scene
        formation
        updateRate
        wayPoint
        traj
        stopTime
        timeOfArrival
        baseOfInertialFrame            %Base of new inertial frame
        uavPlats
    end
    
    methods
        function obj = scenario(wayPoint,formation)
            obj.updateRate = 240;
            obj.wayPoint = wayPoint;
            obj.stopTime = length(wayPoint)-1;
            obj.timeOfArrival = 0:obj.stopTime;
            obj.timeOfArrival
            obj.baseOfInertialFrame = "NED";
            obj.scene = uavScenario("UpdateRate",obj.updateRate,"StopTime",obj.stopTime,"ReferenceLocation",[46, 42, 0]); 
            obj.traj = waypointTrajectory("Waypoints", obj.wayPoint,"TimeOfArrival",obj.timeOfArrival);
           
            obj.formation = formation;
            for n = 1:formation.numOfDrones
                if n == 1
                    obj.uavPlats = [uavPlatform(['UAV',mat2str(n)],obj.scene,"Trajectory",obj.traj)];
                    updateMesh(obj.uavPlats,"quadrotor", {4}, [1 0 0],eul2tform([0 0 pi])); 
                    continue
                end
                obj.uavPlats = [uavPlatform(['UAV',mat2str(n)],obj.scene,"ReferenceFrame","NED", ...
                    "InitialPosition",formation.initialPos(n-1,:),"InitialOrientation",obj.traj.Orientation(1))];
                updateMesh(obj.uavPlats,"quadrotor", {4}, [1 0 0],eul2tform([0 0 pi])); 
            end
        end
        
        function ax = showScenario(obj)
            obj.scene.addInertialFrame(obj.baseOfInertialFrame,"MAP",trvec2tform([1 0 0])); 
            color.Gray = 0.651*ones(1,3);
            addMesh(obj.scene,"polygon",{[-250 -150; 200 -150; 200 180; -250 180],[-4 0]},color.Gray);
            ax = show3D(obj.scene); 
            axis(ax,"equal");
            
        end

        function drawTraj(obj)
            plot3(obj.wayPoint(:,2),obj.wayPoint(:,1),-obj.wayPoint(:,3),"Color",[1 1 1],'Linewidth',0.4);
            hold on
        end
       
    end    
      
end

