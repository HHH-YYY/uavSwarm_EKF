clear

%Specify the leader's trajectory
leaderWaypoint = [0 -200 -50; 50 -175 -50; 100 -150 -50; 50 -125 -50; 0 -100 -50; 
                 -50 -75 -50; -100 -50 -50; -50 -25 -50; 0 0 -50; 50 25 -50; 100 50 -50; 
                 50 75 -50; 0 100 -50; -50 125 -50; -100 150 -50;];

%For basic generation of noise 
positionAccuracy = [1 1 1];
velocityAccuracy = 0.05;
accelerationAccuracy = 0.05;
angularVelocityAccuracy = 0.05;
accuracy = struct(...
            'AccuracyPos', positionAccuracy,... 
            'AccuracyVel', velocityAccuracy,...
            'AccuracyAcc', accelerationAccuracy,...
            'AccuracyAng', angularVelocityAccuracy);

%Specify formation shape
formation = vShapeFormationLosingLeader();

%Generating scenes
scene = scenario(leaderWaypoint,formation);
%scene.drawTraj();
ax = scene.showScenario();

%Generate the simulator and run it
sim = leaderFollowerSim(scene,accuracy);
path = sim.run(ax);

%Draw the paths of uav2,3,4,5
plot3(path(:,1),path(:,2),path(:,3),'b', ...
    path(:,1+16*1),path(:,2+16*1),path(:,3+16*1),'r',  ...
    path(:,1+16*2),path(:,2+16*2),path(:,3+16*2),'y',  ...
    path(:,1+16*3),path(:,2+16*3),path(:,3+16*3),'g',  ...
    'LineWidth',1)
xlim([-150 150])
ylim([-250 220])
zlim([-150 50])
legend('uav2','uav3','uav4','uav5');
xlabel('x')
ylabel('y')
zlabel('z')
grid on;