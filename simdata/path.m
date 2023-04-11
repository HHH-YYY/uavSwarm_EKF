%%Showing 3 different preset paths
load('path1.mat')
load('path2.mat')
load('path3.mat')

plot3(path1(:,1),path1(:,2),path1(:,3),path2(:,1),path2(:,2),path2(:,3),path3(:,1),path3(:,2),path3(:,3),'LineWidth',2)
xlim([-150 150])
ylim([-250 220])
zlim([-150 50])
legend('path1','path2','path3');
xlabel('m')
ylabel('m')
zlabel('m')