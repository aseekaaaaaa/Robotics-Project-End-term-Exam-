startLoc = [6.5; 7; 1.5]; % starting position of a robot
goalLoc = [6.5, 10,0]; % desired location to go to

% load an environment map
map = generateMap('maps\map1.png', 0.1, 10);
% process the map
boubds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
statespace = stateSpaceDubins(boubds);
statespace.MinTurningRadius = 0.2;
image = imread('maps\map1.png');
grayscale = rgb2gray(image);
bwimage = grayscale < .1;

% morphological dilation
se = strel('sphere',2);
bwimage = imdilate(bwimage,se);

% convert the map to  binaryOccupancyMap
mapDilatation = binaryOccupancyMap(bwimage,10);

show(mapDilatation)

% calculate the path for the robot
stateValidator = validatorOccupancyMap(statespace); 
stateValidator.Map = mapDilatation;
% check for an obstacle between this distance in the path
stateValidator.ValidationDistance = .1;
planner = plannerRRTStar(statespace, stateValidator);
planner.MaxConnectionDistance = .02;
planner.MaxIterations = 30000;
planner.GoalReachedFcn = @exampleHelperCheckIfGoal;
rng(1,'twister')
[pthObj, solnInfo] = plan(planner, startLoc.', goalLoc);

show(map)
hold on
plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-');
% plot path
interpolate(pthObj,300)
plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2)
hold off

wheelRadius = 0.3;
wheelBase = 0.5;
% initialize a differential drive robot
dd = DifferentialDrive(wheelRadius, wheelBase);

viz = Visualizer2D;
viz.robotRadius = 0.5;
% assign the map to the visual space
viz.mapName = 'map';

% PurePursuit for path tracking
controller = controllerPurePursuit;
% poses calculate by RRT algorithm
controller.Waypoints = [pthObj.States(:,1), pthObj.States(:,2) ]; 
% how far along the route the robot should look to find angular velocity
controller.LookaheadDistance = .05;
controller.DesiredLinearVelocity = 0.1;
controller.MaxAngularVelocity = pi/6;

currentLoc = startLoc;

[sampleTime,tVec,r] = simulTime(0.1,30);

for idx = 2:numel(tVec) 
    [referenceV,referenceW,lookAheadPt] = controller(currentLoc);
    bodyV = [referenceV;0;referenceW];

    % convert from robot body to world
    vel =  bodyToWorld(bodyV,currentLoc);  
   
    % update robot's current location
    currentLoc = currentLoc + vel * sampleTime; 

    % update its location in a 2d space
    viz(currentLoc);

    waitfor(r);   
end

function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
    isReached = false;
    threshold = 0.1;
    if planner.StateSpace.distance(newState, goalState) < threshold
        isReached = true;
    end
end