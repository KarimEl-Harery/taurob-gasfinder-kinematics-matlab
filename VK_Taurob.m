clear, clc, close all;

%% Load the robot and define joint limits (unchanged section)
if isfile('Taurob_Gasfinder_Assembly\urdf\Taurob_Gasfinder_Assembly.urdf')
    robot = importrobot('Taurob_Gasfinder_Assembly\urdf\Taurob_Gasfinder_Assembly.urdf');
    removeBody(robot,"CS_Link_Flipper");
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

    joint1.PositionLimits = deg2rad([45, 0]);
    joint3.PositionLimits = deg2rad([-180, 180]);
    joint4.PositionLimits = deg2rad([-90, 0]);
    joint5.PositionLimits = deg2rad([-180, 180]);
    joint6.PositionLimits = deg2rad([-180, 180]);
end

%% Movements Within Limits
numSteps = 100; % Number of steps per joint
numJoints = numel(robot.Bodies);

% Generate angles for each joint within limits
jointAngles = zeros(numSteps, numJoints);
for j = 1:numJoints
    lowerLimit = robot.Bodies{j}.Joint.PositionLimits(1);
    upperLimit = robot.Bodies{j}.Joint.PositionLimits(2);
    
    % Angles: start at a random angle, end at another random angle
    startPoint = lowerLimit + (upperLimit - lowerLimit) * rand();
    endPoint = lowerLimit + (upperLimit - lowerLimit) * rand();
    jointAngles(:, j) = linspace(startPoint, endPoint, numSteps);
end

% Initialize storage for end-effector positions and orientations
endEffectorPositions = zeros(numSteps, 3);
endEffectorOrientations = zeros(numSteps, 3);

% End-effector name
endEffectorName = 'EE';

% Visualization: Single figure
figure(1);
hold on;

% Set axis limits and labels
xlim([-2.5, 2.5]);
ylim([-2.5, 2.5]);
zlim([-1, 5]);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on;

% Set a 3D view angle
view(-45, 20);

% Add lighting and set background color
light('Position', [-2.5, -2.5, 5], 'Style', 'infinite');
lighting gouraud;
set(gca, 'Color', [0.8, 0.9, 1]); % Light blue background for better contrast

% Show initial robot visualization
show(robot, homeConfiguration(robot), 'Frames', 'on');
title('Forward Kinematics');

% Add a fixed text object for pose information
poseTextPosition = [-2, 2, 4.5]; % Change this position to move the text
poseInfoText = text(poseTextPosition(1), poseTextPosition(2), poseTextPosition(3), ...
    '', 'FontSize', 10, 'Color', 'blue', 'BackgroundColor', 'white');

%% Video Recording Setup
videoFileName = 'robot_forward.mp4'; % Output video file name
v = VideoWriter(videoFileName, 'MPEG-4'); % Create video writer object
v.Quality = 100; % Set video quality
v.FrameRate = 20; % Set frames per second
open(v); % Open the video file for writing

%% Move joints along the angles 
for i = 1:numSteps
    % Create a new configuration for the current step
    config = homeConfiguration(robot);
    for j = 1:numel(config)
        config(j).JointPosition = jointAngles(i, j); 
    end

    % Compute forward kinematics for the end-effector
    tform = getTransform(robot, config, endEffectorName);

    % Extract translation and orientation
    position = tform2trvec(tform); % Extract translation (position)
    orientation = tform2eul(tform, 'ZYX'); % Extract orientation (roll, pitch, yaw)

    % Store results for visualization
    endEffectorPositions(i, :) = position;
    endEffectorOrientations(i, :) = orientation;

    % Update robot visualization
    show(robot, config, 'PreservePlot', false, 'Frames', 'off');
    drawnow;

    % Plot the position as a red dot
    plot3(position(1), position(2), position(3), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');

    % Update the fixed text with the current pose information
    poseInfoText.String = sprintf('Pose Information:\nX: %.2f, Y: %.2f, Z: %.2f\nR: %.2f °, P: %.2f °, Y: %.2f °', ...
        position(1), position(2), position(3), ...
        rad2deg(orientation(1)), rad2deg(orientation(2)), rad2deg(orientation(3)));

    % Capture the current frame for the video
    frame = getframe(gcf);
    writeVideo(v, frame);
end

% Close the video file
close(v);

legend({'End-Effector Path'}, 'Location', 'best');
disp(['Forward kinematics visualization complete. Video saved as ', videoFileName]);
