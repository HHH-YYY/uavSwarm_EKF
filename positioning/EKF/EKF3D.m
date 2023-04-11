%% Read in real data, GPS position, INS data and IMU data;读入真实数据,GPS定位,INS数据以及IMU数据;
clear;
clc;
load('TrueData.mat')
load('IMUData.mat');
load('INSData.mat')
load('GPSData.mat')
N=5;% 无人机个数
TrueTraj=cell(N,1);
GPSTraj=cell(N,1);
IMUTraj=cell(N,1);
INSTraj=cell(N,1);
for i=1:N
    TrueTraj{i}=DataExtraction(TrueData,i);
    GPSTraj{i}=DataExtraction(GPSData,i);
    IMUTraj{i}=DataExtraction(IMUData,i);
    INSTraj{i}=DataExtraction(INSData,i);
end

%% Drone flight trajectory display 无人机飞行轨迹展示

i=1;
plot3(TrueTraj{i}.TrajPos(:,2),TrueTraj{i}.TrajPos(:,3),TrueTraj{i}.TrajPos(:,1),'ro','Linewidth',1,'MarkerSize',3);
xlim([-200,200]);
ylim([-60,-40]);
zlim([-100,100]);
hold on;
plot3(INSTraj{i}.TrajPos(:,2),INSTraj{i}.TrajPos(:,3),INSTraj{i}.TrajPos(:,1),'go','Linewidth',1,'MarkerSize',3);
plot3(GPSTraj{i}.TrajPos(:,2),GPSTraj{i}.TrajPos(:,3),GPSTraj{i}.TrajPos(:,1),'bo','Linewidth',1,'MarkerSize',3);
plot3(IMUTraj{i}.TrajPos(:,2),IMUTraj{i}.TrajPos(:,3),IMUTraj{i}.TrajPos(:,1),'yo','Linewidth',1,'MarkerSize',3);
grid on;
legend('True','INS','GPS','IMU');
hold off;

%% Flight scene setting飞行场景设定
% Assuming that the nth UAV loses its GPS signal at time tc, the GPS position of this UAV is deduced from the positions of 
% other UAVs with GPS signals and fused with the IMU data of this UAV to achieve position prediction.
% 假设第n个无人机在时刻tc丢失GPS信号，借助存在GPS信号的其他无人机位置推算该无人机的GPS定位,并与该无人机的IMU数据进行融合,实现位置预测

%% Drone position projection无人机位置推算
figure;
T=length(GPSTraj{1}.TrajPos);
Formation=[];
Dnolost=round(rand()*4)+1;% Randomly generate a drone that loses GPS signal随机产生一个丢失GPS信号的无人机
tc=80;
for i =1:N
    Formation=[Formation;GPSTraj{i}.TrajPos(1,:)];%Initial Drone Formation无人机初始阵型
end
Pos_pred=zeros(T-tc+1,3);%Store the predicted location存放预测出的位置

for t=tc:T
    Posavai=[];
    for i =1:N
        if i~=Dnolost
            Posavai=[Posavai;GPSTraj{i}.TrajPos(t,:)];%Location of drones with GPS signal有GPS信号的无人机位置
        end
    end
    Pos_pred(t-tc+1,:)=LocationEstimation(Formation,Dnolost,Posavai);%Projected position at the moment of tc 推算出的在tc时刻的位置
end
plot3(TrueTraj{Dnolost}.TrajPos(:,2),TrueTraj{Dnolost}.TrajPos(:,3),TrueTraj{Dnolost}.TrajPos(:,1),'ro','Linewidth',1,'MarkerSize',3);
hold on;
plot3(Pos_pred(:,2),Pos_pred(:,3),Pos_pred(:,1),'bo','Linewidth',1,'MarkerSize',3);
plot3(GPSTraj{Dnolost}.TrajPos(:,2),GPSTraj{Dnolost}.TrajPos(:,3),GPSTraj{Dnolost}.TrajPos(:,1),'go','Linewidth',1,'MarkerSize',3);
xlim([-200,200]);
ylim([-60,-40]);
zlim([-100,100]);
grid on;
legend('True','Pred','GPS');
%% Fusion of predicted GPS position with IMU signals with the help of EKF 借助EKF，将预测出的GPS定位与IMU信号进行融合
    % Where the IMU data is used for state prediction and the predicted GPS position is used for filtering correction 
    % (i.e., only the predicted GPS volume measurements are used in the volume measurement equation, 
    % and the IMU measurements are used directly in the state prediction equation)
    % 其中，IMU数据用于状态预测，预测出的GPS定位用于滤波修正（即量测方程中只有预测出的GPS量测量，IMU测量值直接用在状态预测方程中）
delta_t=1;
S=6;%状态数
X=ones(6,T-tc+1);%Storage state vector存放状态向量
Fk=eye(S);% State Transfer Matrix状态转移矩阵
Fk(1,4)=delta_t;Fk(2,5)=delta_t;Fk(3,6)=delta_t;
Gk=[delta_t^2/2,0,0;0,delta_t^2/2,0;0,0,delta_t^2/2;delta_t,0,0;0,delta_t,0;0,0,delta_t];%噪声驱动矩阵
w_mu=[0,0,0];% Position noise mean value位置噪声均值
v_mu=[0,0,0];% Speed noise mean value速度噪声均值
X(:,1)=[IMUTraj{Dnolost}.TrajPos(tc,:),IMUTraj{Dnolost}.TrajVel(tc,:)]';%Initial state vector (6 dimensions)初始状态向量（6维)
P=diag([0.1,1,1,0.3,0.3,0.3]);%Initial covariance matrix 初始协方差矩阵 
Qk=0.001*eye(3);% Noise covariance matrix噪声协方差矩阵
R=10;% Measurement variance量测方差
for t=tc+1:T %Using EKF to predict trajectories after loss of GPS signal在丢失GPS信号之后使用EKF预测轨迹
    Posavai=[];
    k=t-tc+1;
    w=mvnrnd(w_mu',Qk);%Location noise位置噪声
    X(:,k)=Fk*X(:,k-1)+Gk*w';%EKF Equation 1: State Transfer  EKF公式1：状态转移    
    F=Fk;%Calculate the Jacobi matrix计算雅克比矩阵
    P=F*P*F'+Gk*Qk*Gk';%EKF Equation 2: Calculate the covariance matrixEKF公式2：计算协方差矩阵
    % Predicting the GPS signal of other drones with their location借助其他无人机位置预测其GPS信号
    for i =1:N
        if i~=Dnolost
            Posavai=[Posavai;GPSTraj{i}.TrajPos(t,:)];
        end
    end
    Pos=LocationEstimation(Formation,Dnolost,Posavai);%Predicted GPS location预测出的GPS定位
    H=[((X(1:3,k)-Pos')./measurement(Pos,X(1:3,k)))',0,0,0];%Calculate the Jacobi matrix of the measurement function计算量测函数的雅克比矩阵
    K=P*H'*inv(H*P*H'+R);%EKF Equation 3: Calculating Kalman GainEKF公式3：计算卡尔曼增益
    X(:,k)=X(:,k)+K*(measurement(Pos,Pos)-measurement(Pos,X(1:3,k)));%EKF Equation 4: Adjustment error based on the predicted GPS positionEKF公式4：根据预测出的GPS位置调整误差
    P=P-K*H*P;%EKF Equation 5: Update the state covariance matrixEKF公式5：更新状态协方差矩阵
end
figure;
plot3(TrueTraj{Dnolost}.TrajPos(:,2),TrueTraj{Dnolost}.TrajPos(:,3),TrueTraj{Dnolost}.TrajPos(:,1),'ro','Linewidth',1,'MarkerSize',3);  
xlim([-200,200]);
ylim([-80,-20]);
zlim([-100,100]);
hold on;
plot3(GPSTraj{Dnolost}.TrajPos(:,2),GPSTraj{Dnolost}.TrajPos(:,3),GPSTraj{Dnolost}.TrajPos(:,1),'bo','Linewidth',1,'MarkerSize',3);
plot3(IMUTraj{Dnolost}.TrajPos(tc:T,2),IMUTraj{Dnolost}.TrajPos(tc:T,3),IMUTraj{Dnolost}.TrajPos(tc:T,1),'yo','Linewidth',1,'MarkerSize',3);
plot3(X(2,:),X(3,:),X(1,:),'go','Linewidth',2,'MarkerSize',3);
legend('True','Pred','DR','GPSIMU')
grid on;
hold off;

%% Calculate RMSE and evaluate algorithm performance计算RMSE，评估算法性能
    % Perform 500 EKF simulations and calculate the RMSE for each moment进行500次EKF仿真，计算每一时刻的RMSE
    M=500;
    PosSE=[];
 for i=1:M
    Dnolost=round(rand()*4)+1;% Randomly generate a drone that loses GPS signal 随机产生一个丢失GPS信号的无人机
    tc=80;
    delta_t=1;
    S=6;
    X=ones(6,T-tc+1);
    Fk=eye(S);
    Fk(1,4)=delta_t;Fk(2,5)=delta_t;Fk(3,6)=delta_t;
    Gk=[delta_t^2/2,0,0;0,delta_t^2/2,0;0,0,delta_t^2/2;delta_t,0,0;0,delta_t,0;0,0,delta_t];
    w_mu=[0,0,0];
    v_mu=[0,0,0];
    X(:,1)=[IMUTraj{Dnolost}.TrajPos(tc,:),IMUTraj{Dnolost}.TrajVel(tc,:)]';
    P=diag([0.1,1,1,0.3,0.3,0.3]);
    Qk=0.001*eye(3);
    R=10;
    for t=tc+1:T 
        Posavai=[];
        k=t-tc+1;
        w=mvnrnd(w_mu',Qk);
        X(:,k)=Fk*X(:,k-1)+Gk*w';  
        F=Fk;
        P=F*P*F'+Gk*Qk*Gk';
        for i =1:N
            if i~=Dnolost
                Posavai=[Posavai;GPSTraj{i}.TrajPos(t,:)];
            end
        end
        Pos=LocationEstimation(Formation,Dnolost,Posavai);
        H=[((X(1:3,k)-Pos')./measurement(Pos,X(1:3,k)))',0,0,0];
        K=P*H'*inv(H*P*H'+R);
        X(:,k)=X(:,k)+K*(measurement(Pos,Pos)-measurement(Pos,X(1:3,k)));
        P=P-K*H*P;
    end
    % Calculation of squared error计算平方误差
    PosSE=[PosSE,sum((TrueTraj{Dnolost}.TrajPos(tc:T,:)-X(1:3,:)').^2,2)];
 end
 % Calculate RMSE, and compare with other data 计算RMSE,并与其他数据进行对比
 PosRMSE=sqrt(sum(PosSE,2)./M);
 GPSSE=[];
 IMUSE=[];
 INSSE=[];
 for Dnolost=1:N
     GPSSE=[GPSSE,sum((TrueTraj{Dnolost}.TrajPos(tc:T,:)-GPSTraj{Dnolost}.TrajPos(tc:T,:)).^2,2)];
     IMUSE=[IMUSE,sum((TrueTraj{Dnolost}.TrajPos(tc:T,:)-IMUTraj{Dnolost}.TrajPos(tc:T,:)).^2,2)];
     INSSE=[INSSE,sum((TrueTraj{Dnolost}.TrajPos(tc:T,:)-INSTraj{Dnolost}.TrajPos(tc:T,:)).^2,2)];
 end
 GPSRMSE=sqrt(sum(GPSSE,2)./N);
 IMURMSE=sqrt(sum(IMUSE,2)./N);
 INSRMSE=sqrt(sum(INSSE,2)./N);
 figure;
 plot(tc:T,GPSRMSE,'b','linewidth',2);
 hold on;
 plot(tc:T,IMURMSE,'y','linewidth',2);
 plot(tc:T,INSRMSE,'g','linewidth',2);
 plot(tc:T,PosRMSE,'r','linewidth',2);
 xlabel('t/s')
 ylabel('RMSE')
 title('Position-RMSE')
 grid on;
 legend('GPSRMSE','IMURMSE','PREDRMSE')
 








