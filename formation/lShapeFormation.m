classdef lShapeFormation < formation
    %VSHAPEFORMATION 
    % A five-robot formation in V shape
    %
    %   4 - 2 - 1 - 3 - 5
    %
    
    properties
    end
    
    methods
        function obj = lShapeFormation()
            
            obj.numOfDrones = 5;
            obj.initialPos = [-5 -200 -50;
                             5 -200 -50;
                             -10 -200 -50;
                             10 -200 -50];
            s = [1 1 2 3];
            t = [2 3 4 5]; 
            obj.graph = digraph(s,t);
            obj.relPos = [0 0 0
                          -5 0 0;
                          5 0 0; 
                          -5 0 0
                          5 0 0];
        end
        
    end
end