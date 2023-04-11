load insPos.mat

[r,c] = size(allPos);
gpsPos = [allPos(1,:)];


for i=2:r
    rresult = [];
    cresult = [];
    for j=1:c
        if mod(j,16)==0 && j <= c
            gap = j-16;
            [currentPos,currentVel]= generatingNoseFromGPS(allPos(i,1+gap:3+gap),allPos(i,4+gap:6+gap));
            a = (currentPos-gpsPos(i-1,1+gap:3+gap));
            b = (currentVel-gpsPos(i-1,4+gap:6+gap));
            gpsAcc = [a(1)/b(1),a(1)/b(1),a(1)/b(1)] ;
            rresult = [currentPos,currentVel,gpsAcc,allPos(i,10+gap:13+gap),allPos(i,14+gap:16+gap)];
            cresult = [cresult rresult];
        end   
    end
    gpsPos = [gpsPos;cresult];
end