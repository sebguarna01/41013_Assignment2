%% Draft File to Test Movements of ABBIRB1200 robot
%define steps for movement of the robot
steps = 100;

%set up the workspace size
axis equal
axis([-1 2 -2 2 0 2])
hold on;

%initialise LinearUR3 robot as variable r
disp('Initialising...');
workspace = PlaceObject(['bar.ply'], [0,0,0]);

gin = PlaceObject('greenbottle.ply', [0,0.5,0.5]);
vodka = PlaceObject('vodkabottle.ply', [-0.1,0.5,0.5]);
rum = PlaceObject('rumbottle.ply', [-0.2,0.5,0.5]);
whiskey = PlaceObject('greenbottle.ply', [-0.3,0.5,0.5]);

% add mixers 1 - 4
% coke = PlaceObject('???.ply', [0,-0.5,0.5]);
% lemonade? = PlaceObject('???.ply', [-0.1,-0.5,0.5]);
% orangeJuice = PlaceObject('???.ply', [-0.2,-0.5,0.5]);
% number4 = PlaceObject('???.ply', [-0.3,-0.5,0.5]);

% Create an ABB IRB 120 model
robot = ABBIRB1200();

robot.model.base = robot.model.base.T * transl(0,0,0.5);
robot.model.animate(zeros(1, 6));
drawnow;

dobot = LinearDobotMagician();

disp('Press ENTER to Start');
pause;

% Define a list of Cartesian poses to move to
% targetJointPoses = [
%     transl(0.5, 0, 0.9) * rpy2tr(180, -90, 0, 'deg');
%     transl(0,0.5,0.5) * rpy2tr(180, -90, 0, 'deg');
%     transl(-0.1, 0.4, 0.9) * rpy2tr(180, -90, 0, 'deg');
%     ];

% Initialize the trajectory with the first pose
trajectory = targetJointPoses(1, :);

numSteps = 50;

% Initialize an empty cell array to store the trajectory
fullTrajectory = cell(1, 0);

% Generate trajectory for each pair of consecutive poses
for i = 1:size(targetJointPoses, 1) - 1
    startPose = targetJointPoses(i, :);
    endPose = targetJointPoses(i + 1, :);

    % Interpolate poses to create a smooth trajectory segment
    segmentTrajectory = interpolatePoses(startPose, endPose, numSteps);

    % % Concatenate the segment trajectory to the full trajectory
    % fullTrajectory = [fullTrajectory, segmentTrajectory];

    % Move the robot along the complete trajectory
    moveDobot(robot, segmentTrajectory, numSteps);

    % disp(['Transformation Matrix for Pose ', num2str(i), ':']);
    % disp(robot.model.fkine(endPose).T);

    startPose = targetJointPoses(i+1);

    pause(2);
end

disp(['DONE']);

function moveIRB1200(robot, trajectory, numSteps)
    % Move the UR3 robot along a given trajectory
    for i = 1:numSteps
        % Solve joint angles using inverse kinematics
        qSol = robot.model.ikine(trajectory{i}, 'q0', zeros(1, 6));
        
        robot.model.animate(qSol); % Animate the robot's motion
        drawnow;
    end
end

function trajectory = interpolatePoses(startPose, endPose, numSteps)
    % Interpolate between two joint configurations to generate a trajectory
    trajectory = cell(1, numSteps);
    for i = 1:numSteps
        t = (i - 1) / (numSteps - 1);
        % Linear interpolation between start and end joint configurations
        interpolatedPose = (1 - t) * startPose + t * endPose;
        trajectory{i} = interpolatedPose;
    end
end
