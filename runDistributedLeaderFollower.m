clear

%Specify the leader's trajectory
% leaderWaypoint = [0 -200 -50; 0 -175 -50; 0 -150 -50; 0 -125 -50; 0 -100 -50; 
%                   0 -75 -50; 0 -50 -50; 0 -25 -50; 0 0 -50; 0 25 -50; 0 50 -50; 
%                   0 75 -50; 0 100 -50; 0 125 -50; 0 150 -50;];

leaderWaypoint = [0 -200 -50; 50 -175 -50; 100 -150 -50; 50 -125 -50; 0 -100 -50; 
                  -50 -75 -50; -100 -50 -50; -50 -25 -50; 0 0 -50; 50 25 -50; 100 50 -50; 
                  50 75 -50; 0 100 -50; -50 125 -50; -100 150 -50;];

%For noise of GNSS generation
positionAccuracy = [1 1 1];
velocityAccuracy = 0.3;
accelerationAccuracy = 0.1;
angularVelocityAccuracy = 0.1;
accuracy = struct(...
            'AccuracyPos', positionAccuracy,... 
            'AccuracyVel', velocityAccuracy,...
            'AccuracyAcc', accelerationAccuracy,...
            'AccuracyAng', angularVelocityAccuracy);

%Specify formation shape
formation = cShapeFormation();

%Generating scenes
scene = scenario(leaderWaypoint,formation);
scene.drawTraj();
ax = scene.showScenario();

%Generate the simulator and run it
sim = leaderFollowerSim(scene,accuracy);
allPos= sim.run(ax);

