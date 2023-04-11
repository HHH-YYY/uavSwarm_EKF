% 根据初始编队形状，设计无人机之间的几何推算关系
function pos=LocationEstimation(Formation,Dnolost,Posavai)
% input：无人机编队形状(初始时所有无人机的坐标)
          %丢失GPS信号的无人机编号Dnolost,
          %可使用GPS信号的无人机位置Posavai
% output：推算出的丢失GPS信号的无人机的位置

    N=size(Formation,1);
    if Dnolost==1
        Diff=repmat(Formation(Dnolost,:),[N-1,1])-Formation(2:N,:);
    elseif Dnolost==N
        Diff=repmat(Formation(Dnolost,:),[N-1,1])-Formation(1:N-1,:);
    else
        Diff=repmat(Formation(Dnolost,:),[N-1,1])-[Formation(1:Dnolost-1,:);Formation(Dnolost+1:N,:)];
    end
    pos=mean(Posavai+Diff);

