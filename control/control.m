classdef (Abstract)control
    %CONTROLLER Abstract class of robot controllers
    
    properties
        formation
        leader
        drones
        numOfDrones
        position
        velocity
        acceleration
        quaternion 
        angularVelocity
        
    end
    
    methods (Abstract)
        followerMove(obj)
    end
end