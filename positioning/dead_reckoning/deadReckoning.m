classdef deadReckoning
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        sampleTime
        prePos
        preVel
        preAcc
    end
    
    methods
        function obj = deadReckoning(sampleTime,prePos,preVel,preAcc)
            obj.sampleTime = sampleTime;
            obj.prePos = prePos;
            obj.preVel = preVel;
            obj.preAcc = preAcc;
        end
        
        function [pos,vel] = generateNextPos(obj)
            x = normrnd(obj.prePos(1) + obj.preVel(1)*obj.sampleTime,0.2);
            y = normrnd(obj.prePos(2) + obj.preVel(2)*obj.sampleTime,0.2);
            z = normrnd(obj.prePos(3) + obj.preVel(3)*obj.sampleTime,0.2);
            pos = [x,y,z];
            vx = obj.preVel(1) + obj.preAcc(1)*obj.sampleTime;
            vy = obj.preVel(2) + obj.preAcc(2)*obj.sampleTime;
            vz = obj.preVel(3) + obj.preAcc(3)*obj.sampleTime;
            vel = [vx,vy,vz];
        end
    end
end

