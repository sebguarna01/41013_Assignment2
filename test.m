%% Sample Draft File
% Set up the workspace size
axis equal
axis([-1 2 -2 2 0 2])
hold on;

% Initialise System
disp('Initialising...');
workspace = PlaceObject(['bar.ply'], [0,0,0]);

% add alcohol 1 - 4
gin = PlaceObject('greenbottle.ply', [0.0854, 0.5939,0.5]);
vodka = PlaceObject('vodkabottle.ply', [0,0.6,0.5]);
whiskey = PlaceObject('greenbottle.ply', [-0.1042,0.5909,0.5]);
% rum = PlaceObject('rumbottle.ply', [-0.2296,0.5543,0.5]);

% add mixers 1 - 3
coke = PlaceObject('greenbottle.ply', [0.0854, -0.5939,0.5]);
lemonade = PlaceObject('greenbottle.ply', [0,-0.6,0.5]);
orangeJuice = PlaceObject('greenbottle.ply', [-0.1042,-0.5909,0.5]);
% number4 = PlaceObject('greenbottle.ply', [-0.2296,-0.5543,0.5]);

% Create an ABB IRB 120 model
IRB1200 = ABBIRB1200();
IRB1200.model.base = IRB1200.model.base.T * transl(0,0,0.5);
IRB1200.model.animate(zeros(1, 6));
drawnow;

% Create DoBot Magician (mounted on Linear Rail) model
dobot = LinearDobotMagician();

disp('Press ENTER to Start');
pause;

% Definition of Drinks Poses in reference to ABB IRB 1200 joints
aboveBottle1 = [pi/2.2, pi/4, pi/4, 0, -pi/2, -pi/2];    % above bottle 1
Bottle1 = [pi/2.2, pi/2.55, pi/9, 0, -pi/2, -pi/2]; % bottle 1 offset

aboveBottle2 = [pi/2, pi/4, pi/4, 0, -pi/2, -pi/2];    % above bottle 2
Bottle2 = [pi/2, pi/2.55, pi/9, 0, -pi/2, -pi/2]; % bottle 2 offset

aboveBottle3 = [pi/1.8, pi/4, pi/4, 0, -pi/2, -pi/2];    % above bottle 3
Bottle3 = [pi/1.8, pi/2.55, pi/9, 0, -pi/2, -pi/2]; % bottle 3 offset

aboveMixer1 = [-pi/2.2, pi/4, pi/4, 0, -pi/2, -pi/2];    % above mixer 1
Mixer1 = [-pi/2.2, pi/2.55, pi/9, 0, -pi/2, -pi/2]; % mixer 1 offset

aboveMixer2 = [-pi/2, pi/4, pi/4, 0, -pi/2, -pi/2];    % above mixer 2
Mixer2 = [-pi/2, pi/2.55, pi/9, 0, -pi/2, -pi/2]; % mixer 2 offset

aboveMixer3 = [-pi/1.8, pi/4, pi/4, 0, -pi/2, -pi/2];    % above mixer 3
Mixer3 = [-pi/1.8, pi/2.55, pi/9, 0, -pi/2, -pi/2]; % mixer 3 offset

% % Define a list of joint configurations (poses) to move to
% targetJointPoses = [
%     0, 0, 0, 0, 0, 0; % start pose 0
% 
%     pi/2.2, pi/4, pi/4, 0, -pi/2, -pi/2;    % above bottle 1
%     pi/2.2, pi/2.55, pi/9, 0, -pi/2, -pi/2; % bottle 1 offset
% 
%     % 0, pi/2.55, pi/9, 0, -pi/2, 0;
%     0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
%     0, pi/4, pi/4, 0, -pi/2, pi/2;      % pour drink
%     0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
% 
%     -pi/2, pi/4, pi/4, 0, -pi/2, -pi/2;     % above mixer 1
%     -pi/2, pi/2.55, pi/9, 0, -pi/2, -pi/2;  % mixer 1 offset
% 
%     % 0, pi/2.55, pi/9, 0, -pi/2, 0;
%     0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
%     0, pi/4, pi/4, 0, -pi/2, pi/2;      % pour drink
%     0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
% 
%     pi/2, pi/4, pi/4, 0, -pi/2, -pi/2;      % above bottle 2
%     pi/2, pi/2.55, pi/9, 0, -pi/2, -pi/2;   % bottle 2 offset
% 
%     pi/1.8, pi/4, pi/4, 0, -pi/2, -pi/2;    % above bottle 3
%     pi/1.8, pi/2.55, pi/9, 0, -pi/2, -pi/2; % bottle 3 offset
% 
%     % pi/1.6, pi/4, pi/4, 0, -pi/2, 0;          % above no. 4
%     % pi/1.6, pi/2.55, pi/9, 0, -pi/2, -pi/2;   % no. 4 offset
% 
%     0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
%     0, pi/4, pi/4, 0, -pi/2, pi/2;      % pour drink
%     0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
% 
%     aboveBottle1;
% ];
% 
% % Initialize the trajectory with the first pose
% trajectory = targetJointPoses(1, :);
% 
% % Set number of steps
% numSteps = 50;
% 
% % Initialize an empty cell array to store the trajectory
% fullTrajectory = cell(1, 0);
% 
% % Generate trajectory for each pair of consecutive poses
% for i = 1:size(targetJointPoses, 1) - 1
%     startPose = targetJointPoses(i, :);
%     endPose = targetJointPoses(i + 1, :);
% 
%     % Interpolate poses to create a smooth trajectory segment
%     segmentTrajectory = interpolatePoses(startPose, endPose, numSteps);
% 
%     % Move the robot along the complete trajectory
%     moveIRB1200(robot, segmentTrajectory, numSteps);
% 
%     disp(['Transformation Matrix for Pose ', num2str(i), ':']);
%     disp(robot.model.fkine(endPose).T);
% 
%     startPose = targetJointPoses(i+1);
% end
% 
% disp(['DONE']);

makeVodkaLemonade(IRB1200, aboveBottle2, Bottle2, aboveMixer2, Mixer2);
pause(2);
makeWhiskeyANDCoke(IRB1200, aboveBottle3, Bottle3, aboveMixer1, Mixer1);
pause(2);

% OR
% makeDrink(IRB1200, aboveBottle2, Bottle2, aboveMixer3, Mixer3);

%% Drink Making Functions
% The drink making functions follow the same structure as the following
% function, replacing the bottle and mixer with the desired combination
function makeDrink(IRB1200, aboveBottle, Bottle, aboveMixer, Mixer)
disp(['Making Drink...'])
    posesDrink = [
        0, 0, 0, 0, 0, 0; % start pose 0
    
        aboveBottle;
        Bottle;
        aboveBottle;

        % 0, pi/2.55, pi/9, 0, -pi/2, 0;
        0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
        0, pi/4, pi/4, 0, -pi/2, pi/2;      % pour drink
        0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'

        aboveBottle;
        Bottle;         % put bottle back
        aboveBottle;
    
        aboveMixer;
        Mixer;
        aboveMixer;
    
        % 0, pi/2.55, pi/9, 0, -pi/2, 0;
        0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
        0, pi/4, pi/4, 0, -pi/2, pi/2;      % pour drink
        0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'

        aboveMixer;
        Mixer;          % put mixer back
        aboveMixer;
    
        0, 0, 0, 0, 0, 0;
    ];
    
    % Initialize the trajectory with the first pose
    trajectory = posesDrink(1, :);
    
    % Set number of steps
    numSteps = 50;
    
    % Generate trajectory for each pair of consecutive poses
    for i = 1:size(posesDrink, 1) - 1
        startPose = posesDrink(i, :);
        endPose = posesDrink(i + 1, :);
    
        % Interpolate poses to create a smooth trajectory segment
        segmentTrajectory = interpolatePoses(startPose, endPose, numSteps);
    
        % Move the robot along the complete trajectory
        moveIRB1200(IRB1200, segmentTrajectory, numSteps);
    
        startPose = posesDrink(i+1);
    end
    
    disp(['DONE.']);
end

function makeVodkaLemonade(IRB1200, aboveVodka, Vodka, aboveLemonade, Lemonade)
    disp(['Making Vodka Lemonade...'])
    posesVodkaLemonade = [
        0, 0, 0, 0, 0, 0; % start pose 0
    
        aboveVodka;
        Vodka;
        aboveVodka;

        % 0, pi/2.55, pi/9, 0, -pi/2, 0;
        0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
        0, pi/4, pi/4, 0, -pi/2, pi/2;      % pour drink
        0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'

        aboveVodka;
        Vodka;
        aboveVodka;
    
        aboveLemonade;
        Lemonade;
        aboveLemonade;
    
        % 0, pi/2.55, pi/9, 0, -pi/2, 0;
        0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
        0, pi/4, pi/4, 0, -pi/2, pi/2;      % pour drink
        0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
        
        aboveLemonade;
        Lemonade;
        aboveLemonade;

        0, 0, 0, 0, 0, 0;
    ];
    
    % Initialize the trajectory with the first pose
    trajectory = posesVodkaLemonade(1, :);
    
    % Set number of steps
    numSteps = 50;
    
    % Generate trajectory for each pair of consecutive poses
    for i = 1:size(posesVodkaLemonade, 1) - 1
        startPose = posesVodkaLemonade(i, :);
        endPose = posesVodkaLemonade(i + 1, :);
    
        % Interpolate poses to create a smooth trajectory segment
        segmentTrajectory = interpolatePoses(startPose, endPose, numSteps);
    
        % Move the robot along the complete trajectory
        moveIRB1200(IRB1200, segmentTrajectory, numSteps);
    
        startPose = posesVodkaLemonade(i+1);
    end
    
    disp(['DONE.']);
end

function makeWhiskeyANDCoke(IRB1200, aboveWhiskey, Whiskey, aboveCoke, Coke)
    disp(['Making Whiskey and Coke...'])
    posesWhiskeyANDCoke = [
        0, 0, 0, 0, 0, 0; % start pose 0
    
        aboveWhiskey;
        Whiskey;
        aboveWhiskey;

        % 0, pi/2.55, pi/9, 0, -pi/2, 0;
        0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
        0, pi/4, pi/4, 0, -pi/2, pi/2;      % pour drink
        0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
    
        aboveWhiskey;
        Whiskey;
        aboveWhiskey;

        aboveCoke;
        Coke;
        aboveCoke;
    
        % 0, pi/2.55, pi/9, 0, -pi/2, 0;
        0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
        0, pi/4, pi/4, 0, -pi/2, pi/2;      % pour drink
        0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
    
        aboveCoke;
        Coke;
        aboveCoke;

        0, 0, 0, 0, 0, 0;
    ];
    
    % Initialize the trajectory with the first pose
    trajectory = posesWhiskeyANDCoke(1, :);
    
    % Set number of steps
    numSteps = 50;
    
    % Generate trajectory for each pair of consecutive poses
    for i = 1:size(posesWhiskeyANDCoke, 1) - 1
        startPose = posesWhiskeyANDCoke(i, :);
        endPose = posesWhiskeyANDCoke(i + 1, :);
    
        % Interpolate poses to create a smooth trajectory segment
        segmentTrajectory = interpolatePoses(startPose, endPose, numSteps);
    
        % Move the robot along the complete trajectory
        moveIRB1200(IRB1200, segmentTrajectory, numSteps);
    
        startPose = posesWhiskeyANDCoke(i+1);
    end
    
    disp(['DONE.']);
end

function moveDobot(robot, trajectory, numSteps)
    % Move the Dobot Magician robot along a given trajectory
    for i = 1:numSteps
        % Calculate the end-effector transformation using forward kinematics
        endEffectorPose = robot.model.fkine(trajectory{i});

        % Solve joint angles using inverse kinematics
        qSol = robot.model.ikine(endEffectorPose, 'q0', zeros(1, 6));

        robot.model.animate(qSol); % Animate the robot's motion
        drawnow;
    end
end

function moveIRB1200(robot, trajectory, numSteps)
    % Move the UR3 robot along a given trajectory
    for i = 1:numSteps
        % Calculate the end-effector transformation using forward kinematics
        endEffectorPose = robot.model.fkine(trajectory{i});

        % Solve joint angles using inverse kinematics
        qSol = robot.model.ikine(endEffectorPose, 'q0', zeros(1, 6));
        
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
