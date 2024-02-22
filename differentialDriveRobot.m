wheelRadius = 0.1;
wheelBase = 0.5;
% initialize a differential drive robot
dd = DifferentialDrive(wheelRadius, wheelBase) 

viz = Visualizer2D;
startLoc = [0, 0, 0]; % starting position of a robot
viz.robotRadius = 0.3;

sampleTime = 0.5;
tVec = 0:sampleTime:10; % time array
r= rateControl(1/sampleTime); % rate control

% compute velocity (v) and angular velocity (w)
wL = pi;
wR = pi/2;
[v,w] = forwardKinematics(dd,wL,wR);

bodyV = [v;0;w];

currentLoc = startLoc;

for idx = 2:numel(tVec) 
    % convert from robot body to world
    vel =  bodyToWorld(bodyV,currentLoc);
   
    % update robot's current location
    currentLoc = currentLoc + vel * sampleTime;
    
    % update its location in a 2d space
    viz(currentLoc);

    waitfor(r);    
end