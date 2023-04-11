% Get direction cosine matrix from two vectors
function dcm = getDcmFrom2Vectors(v1,v2)
    nv1 = v1/norm(v1);
    nv2 = v2/norm(v2);
    
    if norm(nv1+nv2) == 0
        q = [0 0 0 0];
    else
        u = cross(nv1,nv2);
        u = u/norm(u);
    
        theta = acos(sum(nv1.*nv2))/2;
        q = [cos(theta) sin(theta)*u];
    end
    
    dcm = quat2dcm(q);
end

