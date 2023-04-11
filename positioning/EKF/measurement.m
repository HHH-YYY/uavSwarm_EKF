function f = measurement(x,xp)
%借助GPS距离作为距离量测
    v_mu=0;
    q=0.001;
    Qk=q*eye(1);% cov. of process noise
    v=mvnrnd(v_mu',Qk);  
f=sqrt((x(1)-xp(1))^2+(x(2)-xp(2))^2+(x(3)-xp(3))^2)+v;
end

