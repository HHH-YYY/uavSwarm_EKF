function [pos, vel] = generatingNoseFromGPS(truePos,trueVel)
%GENERATINGNOSEFROMIMU 此处显示有关此函数的摘要
%   此处显示详细说明
    gps = gpsSensor('HorizontalPositionAccuracy',2 ,'VerticalPositionAccuracy',2 ,'VelocityAccuracy',0.4);
    [posLLA, vel]= gps(truePos,trueVel);
    [y,x,z] = latlon2local(posLLA(1),posLLA(2),-posLLA(3),[0,0,0]);
    pos = [x,y,z];
end


