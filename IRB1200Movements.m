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
% IRB1200 = ABBIRB1200();
% 
% IRB1200.model.base = IRB1200.model.base.T * transl(0,0,0.5);
% IRB1200.model.animate(zeros(1, 6));
% drawnow;

dobot = LinearDobotMagician();

disp('Press ENTER to Start');
pause;

% Define a list of Cartesian poses to move to
% posesDrink1 = {
%     transl(0.5330, 0, 1.3911) * rpy2tr(0, 0, 0, 'deg');
%     transl(0.5330, 0, 1.3911) * rpy2tr(0, 0, 0, 'deg');
%     transl(0,0.55,0.6) * rpy2tr(0, 0, 0, 'deg');
%     transl(0,0.45,0.6) * rpy2tr(0, 0, 0, 'deg');
%     transl(0,0.45,0.6) * rpy2tr(0, 0, 0, 'deg');
%     transl(0.5330, 0, 1.3911) * rpy2tr(0, 0, 0, 'deg');
%     };
% posesDrink1 = {
%     transl(0.5330, 0, 1.3911) * rpy2tr(0, 0, 0, 'zyx');
%     % transl(0.6, 0, 1) * rpy2tr(0, 0, 0, 'zyx');
%     % transl(0.5330, 0, 1.3911) * rpy2tr(0, 0, 0, 'zyx');
%     transl(0,0.5,0.55) * rpy2tr(0, -pi/2, -pi/2, 'zyx');
%     % transl(0,0.45,0.6) * rpy2tr(0, 0, 0, 'zyx');
%     transl(0.5330, 0, 1.3911) * rpy2tr(0, 0, 0, 'zyx');
%     };

% Define a list of joint configurations (poses) to move to
poses = [
    0, 0, pi/4, pi/4, pi/2, 0; % start pose
    0, 0, pi/4, pi/4, pi/2, 0;
    0, pi, pi/4, pi/4, pi/2, 0;
    0, 0, pi/4, pi/4, pi/2, 0;
];

% Loop through each pose
for i = 1:size(poses, 1)
    % Get the current pose
    currentPose = poses(i, :);

    % Move the robot to the current pose
    dobot.model.animate(currentPose);

    % Pause to observe the transformation
    pause(2); % You can adjust the pause duration as needed

    % Display the transformation matrix of the end effector
    disp(['Transformation Matrix for Pose ', num2str(i), ':']);
    disp(dobot.model.fkine(currentPose).T);
end

initial_pose = transl(0.5406, -0.4, 1.0807) * rpy2tr(0, 0, 0, 'deg');

num_steps = 50;

for i = 1:(length(posesDrink1) - 1)
        trajectory = interpolatePoses(initial_pose, DOBOTMOVE{i+1}, num_steps);
        moveDOBOT(dobot, trajectory, num_steps);
        initial_pose = DOBOTMOVE{i+1}; % Update the initial pose for the next step

        % Display joint values
        disp(['Joint angles at pose ', num2str(i), ':']);
        jointAngles = dobot.model.getpos();
        disp(jointAngles);
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

function moveDOBOT(robot, trajectory, numSteps)
    % Move the UR3 robot along a given trajectory
    for i = 1:numSteps
        % Solve joint angles using inverse kinematics
        qSol = robot.model.ikine(trajectory{i}, 'q0', zeros(1, 6), 'mask', [1 1 1 0 0 0]);
        
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

