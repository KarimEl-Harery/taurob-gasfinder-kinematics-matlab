clear, clc, close all;

%% Load the robot and define joint limits (unchanged section)
if isfile('Taurob_Gasfinder_Assembly\urdf\Taurob_Gasfinder_Assembly.urdf')
    robot = importrobot('Taurob_Gasfinder_Assembly\urdf\Taurob_Gasfinder_Assembly.urdf');
    removeBody(robot,"CS_Link");
    body1 = robot.getBody(robot.BodyNames{1});
    body3 = robot.getBody(robot.BodyNames{3});
    body4 = robot.getBody(robot.BodyNames{4});
    body5 = robot.getBody(robot.BodyNames{5});
    body6 = robot.getBody(robot.BodyNames{6});
    
    joint1 = body1.Joint;
    joint3 = body3.Joint;
    joint4 = body4.Joint;
    joint5 = body5.Joint;
    joint6 = body6.Joint;

    joint1.PositionLimits = deg2rad([0, 45]);
    joint3.PositionLimits = deg2rad([-180, 180]);
    joint4.PositionLimits = deg2rad([-90, 90]);
    joint5.PositionLimits = deg2rad([-180, 180]);
    joint6.PositionLimits = deg2rad([-180, 180]);
end

% Open a single figure
figure(1);
hold on;

% Display the robot structure initially
show(robot);
title('Robot Inverse Kinematics');

% Set the limits for the axes
xlim([-2.5, 2.5]); % Limits for the X-axis
ylim([-2.5, 2.5]); % Limits for the Y-axis
zlim([0, 3]); % Limits for the Z-axis

%% Define Inverse Kinematics Solver
ik = inverseKinematics('RigidBodyTree', robot);

% Solver settings
ik.SolverParameters.MaxIterations = 1500; % Increase max iterations
ik.SolverParameters.GradientTolerance = 1e-7; % Set tolerance for convergence

% Define Weights for position and orientation
weights = [1 1 1 1 1 1]; 

%% Define Circular Trajectory in YZ-Plane
% Circle parameters
radius = 1;          % Radius of the circle (meters)
center = [-1.5, 0.0, 1.5]; % Center of the circle [X, Y, Z]
numPoints = 100;       % Number of points along the circle
theta = linspace(0, 2*pi, numPoints); % Angle parameter from 0 to 2*pi

% Calculate Y and Z positions of the circle
yCircle = center(2) + radius * cos(theta); % Y-coordinates
zCircle = center(3) + radius * sin(theta); % Z-coordinates
xCircle = repmat(center(1), 1, numPoints); % X-coordinate (constant)

% Initialize storage for joint configurations and end-effector positions
trajectoryConfigs = repmat(homeConfiguration(robot), numPoints, 1);
endEffectorPath = zeros(numPoints, 3);

% Define orientation for the circle (constant roll, pitch, yaw)
targetEulerAngles = [0, -90, 0]; % Roll, Pitch, Yaw in degrees

% Solve IK for each point on the circle
for i = 1:numPoints
    % Define target position
    targetPosition = [xCircle(i), yCircle(i), zCircle(i)];
    
    % Convert Euler angles to a rotation matrix
    rotationMatrix = eul2rotm(deg2rad(targetEulerAngles), 'ZYX');
    
    % Create the homogeneous transformation matrix for the target pose
    targetPose = trvec2tform(targetPosition) * rotm2tform(rotationMatrix);
    
    % Solve IK for the current target pose
    [configSol, solInfo] = ik('EE', targetPose, weights, homeConfiguration(robot));
    
    % Store the solution
    trajectoryConfigs(i, :) = configSol;

    % Store the end-effector position
    endEffectorPath(i, :) = tform2trvec(targetPose);
end

% Plot the circular trajectory path
plot3(endEffectorPath(:, 1), endEffectorPath(:, 2), endEffectorPath(:, 3), 'b--', 'LineWidth', 2, 'DisplayName', 'End-Effector Path');

% Set view angle for better visualization
view(-45, 30); % Adjust azimuth (horizontal rotation) and elevation (vertical angle)
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on;

%% Animate the Robot Following the Trajectory and Record
% Enable lighting for the 3D scene
light('Position', [-3, -3, 5], 'Style', 'infinite'); % Add light from a specific position
lighting gouraud; % Use smooth shading for better visuals

% Set up the video writer
videoFileName = 'robot_trajectory.mp4';
v = VideoWriter(videoFileName, 'MPEG-4');
v.Quality = 100; % Set video quality
v.FrameRate = 20; % Set frames per second
open(v); % Open the video file for writing

for i = 1:numPoints
    % Update robot configuration
    show(robot, trajectoryConfigs(i, :), 'PreservePlot', false, 'Frames', 'off');
    drawnow; % Update the figure
    
    % Capture the current frame
    frame = getframe(gcf); % Get the current figure as a frame
    writeVideo(v, frame); % Write the frame to the video
    
    pause(0.05); % Pause for visualization
end

% Close the video file
close(v);

disp(['Trajectory visualization complete. Video saved as ', videoFileName]);
