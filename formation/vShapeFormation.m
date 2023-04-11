classdef vShapeFormation < formation
    %VSHAPEFORMATION 
    % A five-robot formation in V shape
    %           1
    %         /   \
    %        2     3
    %       /       \
    %      4         5
    
    properties
    end
    
    methods
        function obj = vShapeFormation()
            
            obj.numOfDrones = 5;
            obj.initialPos = [0-5 -200-5 -50;
                             0+5 -200-5 -50;
                             0-5 -5 -50;
                             0+5 -5 -50];
            s = [1 1 2 3];
            t = [2 3 4 5]; 
            obj.graph = digraph(s,t);
            obj.relPos = [0 0 0
                          -5   5 0;
                          -5   -5 0; 
                          -5   5 0
                          -5   -5 0];
        end
        
    end
end