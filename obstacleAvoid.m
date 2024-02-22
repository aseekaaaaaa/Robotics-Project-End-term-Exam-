lidar = LidarSensor;
lidar.sensorOffset = [0, 0];
% number of sensors and their angle
lidar.scanAngles = linspace(-pi/2, pi/2, 51);
lidar.maxRange = 1; 

% load an environment map
map = generateMap('maps\map1.png', 0.1, 10);

wheelRadius = 0.3;
wheelBase = 0.5;
% initialize a differential drive robot
dd = DifferentialDrive(wheelRadius, wheelBase);

viz = Visualizer2D;
startLoc = [3.5; 8; 0]; % starting position of a robot
viz.robotRadius = 0.5;
% assign the map to the visual space
viz.mapName = 'map';

% attach lidar sensor to the robot
attachLidarSensor(viz, lidar);

v = 0.2;
w = 0;
bodyV = [v; 0; w];

vfh = controllerVFH;
vfh.DistanceLimits=[0.35 2];
vfh.RobotRadius = viz.robotRadius;
vfh.SafetyDistance = .3;
vfh.MinTurningRadius = .5;

currentLoc = startLoc;

[sampleTime,tVec,r] = simulTime(0.1,75);

for idx = 2:numel(tVec) 
    % obtain sensor data
    ranges = lidar(currentLoc);
    targetDir = 0.1;
    referenceW = vfh(ranges, lidar.scanAngles, targetDir);
    if isnan(referenceW)
        referenceW = 0.5;
    end

    bodyV = [v;0;referenceW];

    % convert from robot body to world
    vel =  bodyToWorld(bodyV,currentLoc);  
   
    % update robot's current location
    currentLoc = currentLoc + vel * sampleTime; 

    % update its location in a 2d space
    viz(currentLoc, ranges);

    waitfor(r);   
end