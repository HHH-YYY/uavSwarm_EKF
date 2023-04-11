classdef leaderFollowerSim < simulation
    %LEADERFOLLOWERSIMULATION 
    % A Leader-Follower formation control simulation
    
    properties
        accuracy
        initialPos
        allVel
        allGpsPos
        allTruePos
    end
    
    methods
        function obj = leaderFollowerSim(scene,accuracy)
            obj.sampleTime = 0.05;
            obj.formation = scene.formation;
            obj.numOfDrones = scene.formation.numOfDrones;
            obj.scene = scene;
            obj.graph = obj.formation.graph;
            obj.accuracy = accuracy;
        end

        function [allTruePos] = run(obj, ax)
            setup(obj.scene.scene);
            while advance(obj.scene.scene)
                time = obj.scene.scene.CurrentTime;
                updateSensors(obj.scene.scene);
                if mod(time,obj.sampleTime) == 0
                    tempTruePos=[];
                    for i = 1:obj.numOfDrones
                        % If there is no path
                        if isempty(obj.graph.successors(i))
                           continue
                        end
                        control = distributedFollowerControl(obj.scene.scene.Platforms, ...
                                                             obj.scene.scene.Platforms(i), ...
                                                             obj.formation, ...
                                                             i, ...
                                                             obj.accuracy,...
                                                             time,...
                                                             tempTruePos);
                        tempTruePos = control.followerMove();

                        if i == 1
                            %for get pos data of GPS
                            obj.allGpsPos = [obj.allGpsPos;control.position];
                            %for get pos data of DR
                            if time == 0.05
                                obj.initialPos = control.position;
                                obj.allVel = control.velocity;
                            elseif time > 0.05
                                obj.allVel = [obj.allVel;control.velocity];
                            end
                        end

                    end
                    show3D(obj.scene.scene,"Parent",ax,"FastUpdate",true); 
                    zlim([0 100])
                    drawnow limitrate
                    obj.allTruePos = [obj.allTruePos;tempTruePos];
                end
            end
%             dr = deadReckoning(15,obj.sampleTime,obj.initialPos,obj.allVel);
%             drpos = dr.generatePos;
%             gpspos = obj.allGpsPos;
            allTruePos = obj.allTruePos;
            hold off
        end

    end
end

