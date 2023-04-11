%Calculate the relative position of follower based
%on the absolute position of leader and the relative 
%position of follower

function pos = calculatAbsPos(absPosLeader, relPosFollower, dcm) 
    pos = absPosLeader + (dcm*relPosFollower')';
end

