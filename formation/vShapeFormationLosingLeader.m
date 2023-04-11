classdef vShapeFormationLosingLeader < formation
    %VSHAPEFORMATION 
    % A five-robot formation in V shape
    %           1 Losing
    %         /   \
    %        2     3
    %       /       \
    %      4         5
    
    properties
    end
    
    methods
        function obj = vShapeFormationLosingLeader()
            %Formation of 4 drones
            obj.numOfDrones = 4;
            %The default initial position of UAV1 is [0 0 -50], UAV2-5 is set as follows
            obj.initialPos = [10  -200   -50;
                              -5  -200-5 -50;
                              -5  -200+5 -50];
            %Store formation in graph form
            s = [1 1 2];
            t = [2 3 4]; 
            obj.graph = digraph(s,t);
            obj.relPos = [ 0   -5   0;
                           0    10  0;
                          -5   -5   0; 
                          -5    5   0];
        end
        
    end
end