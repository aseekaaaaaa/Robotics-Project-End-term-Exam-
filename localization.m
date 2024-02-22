wheelRadius = 0.1;
wheelBase = 0.5;
% initialize a differential drive robot
dd = DifferentialDrive(wheelRadius, wheelBase);

viz = Visualizer2D;
startLoc = [0, 0, 0]; % starting position of a robot
viz.robotRadius = 0.3;

% compute velocity (v) and angular velocity (w)
wL = pi;
wR = pi/2;
[v,w] = forwardKinematics(dd,wL,wR);

bodyV = [v;0;w];

varianceV = 0.03;
varianceW = 0.13;

[sampleTime,tVec,r] = simulTime(0.1,10);

% array of odometry poses
odom_pose = zeros(3,numel(tVec));
odom_pose(:,1) = startLoc;
currentLoc = startLoc;

for idx = 2:numel(tVec) 
    v = 0.3; % const linear velocity
    w = pi*(cos(idx/2)); % angular velocity that will change
    [wL,wR] = inverseKinematics(dd,v ,w);

    % add noise to the velocities
    noiseV = v  + normrnd(0,varianceV);
    noiseW = w + normrnd(0,varianceW);
    bodyV = [noiseV ;0;noiseW];

    % compute odometry
    d_sr = wheelRadius * wR * sampleTime;
    d_sl = wheelRadius * wL * sampleTime;
    d_s = (d_sr + d_sl)/2;
    d_theta = (d_sr - d_sl)/wheelBase;
    
    % store calculated odometries
    odom_pose(:,idx) = odom_pose(:,idx-1) + [ d_s *cos(odom_pose(3,idx-1) +  d_theta/2 ); 
                                              d_s *sin(odom_pose(3,idx-1) +  d_theta/2 );
                                              d_theta];             
    
    % convert from robot body to world
    vel =  bodyToWorld(bodyV,currentLoc);  
   
    % update robot's current location
    currentLoc = currentLoc + vel * sampleTime; 
      
    % update its location in a 2d space
    viz(currentLoc);

    waitfor(r);   
end

hold on
plot(odom_pose(1,:),odom_pose(2,:),'m');