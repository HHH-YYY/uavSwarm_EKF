load insPos.mat

[r,c] = size(allPos);
IMUData = allPos(1,:);
rresult = [];


for i=2:r
    rresult = [];
    cresult = [];
    for j=1:c
        if mod(j,16)==0 && j <= c
            gap = j-16;
            prePos = IMUData(i-1,1+gap:3+gap);
            preVel = IMUData(i-1,4+gap:6+gap);
            preAcc = IMUData(i-1,7+gap:9+gap);
            dr = deadReckoning(0.05,prePos,preVel,preAcc);
            [currentPos, currentVel] = dr.generateNextPos();
            [currentAcc,currentMag] = generatingNoseFromIMU(allPos(i,7+gap:9+gap),allPos(i,14+gap:16+gap),quat2rotm(allPos(i,10+gap:13+gap)));
            
            rresult = [currentPos,currentVel,currentAcc,allPos(i,10+gap:13+gap),allPos(i,14+gap:16+gap)];
            cresult = [cresult rresult];
        end   
    end
    IMUData = [IMUData;cresult];
end