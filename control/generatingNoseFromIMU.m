function [accel,mag] = generatingNoseFromIMU(acc,angVel,orientation)
%GENERATINGNOSEFROMIMU 此处显示有关此函数的摘要
%   此处显示详细说明
%全都是使用默认参数
    IMU = imuSensor("Temperature",20);
    [acc,mag] = IMU(acc,angVel,orientation);
    accel = [acc(1),acc(2)+0.08,acc(3)-9.79];
end

