clear

%Specify the leader's trajectory
leaderWaypoint = [0 -200 -50; 0 -175 -50; 0 -150 -50; 0 -125 -50; 0 -100 -50; 
                  0 -75 -50; 0 -50 -50; 0 -25 -50; 0 0 -50; 0 25 -50; 0 50 -50; 
                  0 75 -50; 0 100 -50; 0 125 -50; 0 150 -50;];

% leaderWaypoint = [0 -200 -50; 50 -175 -50; 100 -150 -50; 50 -125 -50; 0 -100 -50; 
%                  -50 -75 -50; -100 -50 -50; -50 -25 -50; 0 0 -50; 50 25 -50; 100 50 -50; 
%                  50 75 -50; 0 100 -50; -50 125 -50; -100 150 -50;];

% leaderWaypoint = [0 -200 -50; 141 -141 -50; 100 0 -50; 141 141 -50;
%                  0 200 -50; -141 141 -50; -100 0 -50; -141 -141 -50;0 -200 -50];

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
formation = vShapeFormation();

%Generating scenes
scene = scenario(leaderWaypoint,formation);
scene.drawTraj();
ax = scene.showScenario();

%Generate the simulator and run it
sim = leaderFollowerSim(scene,accuracy);
path= sim.run(ax);