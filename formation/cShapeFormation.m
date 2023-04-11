classdef cShapeFormation < formation
    %cSHAPEFORMATION 
    % A six-robot formation in circle shape
    %           1
    %       /       \
    %      2         3
    %      |         | 
    %      4         5
    %       \ 
    %           6       
    properties
    end
    
    methods
        function obj = cShapeFormation()
            
            obj.numOfDrones = 6;
            obj.initialPos = [0-4*sqrt(2) -200-4 -50;
                             0+4*sqrt(2) -200-4 -50;
                             0-4*sqrt(2) -200-8 -50;
                             0+4*sqrt(2) -200-8 -50;
                             0 -200-12 -50];
            s = [1 1 2 3 4];
            t = [2 3 4 5 6]; 
            obj.graph = digraph(s,t);
            obj.relPos = [0 0 0
                          -4*sqrt(2)   -4 0;
                          4*sqrt(2)   -4 0; 
                          0   -4 0;
                          0   -4 0;
                          4*sqrt(2) -4 0];
        end
        
    end
end