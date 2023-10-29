%% Sample Draft File
% Set up the workspace size
axis equal
axis([-1 2 -2 2 0 2])
hold on;

% Set number of steps
numSteps = 50;

%% Initialising System
disp('Initialising...');
workspace = PlaceObject(['bar.ply'], [0,0,0]);
% Set number of steps
numSteps = 50;
% % add alcohol 1 - 4
% gin = PlaceObject('greenbottle.ply', [0.0854, 0.5939,0.5]);
% vodka = PlaceObject('vodkabottle.ply', [0,0.6,0.5]);
% whiskey = PlaceObject('greenbottle.ply', [-0.1042,0.5909,0.5]);
% % rum = PlaceObject('rumbottle.ply', [-0.2296,0.5543,0.5]);
% 
% % add mixers 1 - 3
% coke = PlaceObject('greenbottle.ply', [0.0854, -0.5939,0.5]);
% lemonade = PlaceObject('greenbottle.ply', [0,-0.6,0.5]);
% orangeJuice = PlaceObject('greenbottle.ply', [-0.1042,-0.5909,0.5]);
% % number4 = PlaceObject('greenbottle.ply', [-0.2296,-0.5543,0.5]);

% add alcohol 1 - 4

gin=PlaceObject('greenbottle.ply', [0 0 0]);
gin_vertices=get(gin,'Vertices');
gintr=transl(-0.3146, 0.5939,0.5);
transformedVertices=[gin_vertices,ones(size(gin_vertices,1),1)]*gintr';
set(gin,'Vertices',transformedVertices(:,1:3));

vodka=PlaceObject('vodkabottle.ply', [0 0 0]);
vodka_vertices=get(vodka,'Vertices');
vodkatr=transl(-0.4,0.6,0.5);
transformedVertices=[vodka_vertices,ones(size(vodka_vertices,1),1)]*vodkatr';
set(vodka,'Vertices',transformedVertices(:,1:3));

whiskey=PlaceObject('rumbottle.ply', [0 0 0]);
whiskey_vertices=get(whiskey,'Vertices');
whiskeytr=transl(-0.5042,0.5909,0.5);
transformedVertices=[whiskey_vertices,ones(size(whiskey_vertices,1),1)]*whiskeytr';
set(whiskey,'Vertices',transformedVertices(:,1:3));

% rum = PlaceObject('rumbottle.ply', [-0.2296,0.5543,0.5]);

% add mixers 1 - 3
coke=PlaceObject('greenbottle.ply', [0 0 0]);
coke_vertices=get(coke,'Vertices');
coketr=transl(-0.3146, -0.5939,0.5);
transformedVertices=[coke_vertices,ones(size(coke_vertices,1),1)]*coketr';
set(coke,'Vertices',transformedVertices(:,1:3));

lemonade=PlaceObject('greenbottle.ply', [0 0 0]);
lemonade_vertices=get(lemonade,'Vertices');
lemonadetr=transl(-0.4,-0.6,0.5);
transformedVertices=[lemonade_vertices,ones(size(lemonade_vertices,1),1)]*lemonadetr';
set(lemonade,'Vertices',transformedVertices(:,1:3));

orangeJuice=PlaceObject('greenbottle.ply', [0 0 0]);
orangeJuice_vertices=get(orangeJuice,'Vertices');
orangeJuicetr=transl(-0.5042,-0.5909,0.5);
transformedVertices=[orangeJuice_vertices,ones(size(orangeJuice_vertices,1),1)]*orangeJuicetr';
set(orangeJuice,'Vertices',transformedVertices(:,1:3));

% number4 = PlaceObject('greenbottle.ply', [-0.2296,-0.5543,0.5]);

% Create an ABB IRB 120 model
IRB1200 = ABBIRB1200();
IRB1200.model.base = IRB1200.model.base.T * transl(-0.4,0,0.5);
IRB1200.model.animate(zeros(1, 6));
drawnow;

%Setup Gripper
gripperOrigin = IRB1200.model.fkine(IRB1200.model.getpos());
gripperL = Gripper(gripperOrigin.T * trotx(pi/2));
gripperR = Gripper(gripperOrigin.T *trotx(-pi/2) * trotz(pi));

%gripper movement q values for opening gripper and closing the gripper
Open = [deg2rad(0) deg2rad(0) deg2rad(0)];
Close = [deg2rad(5) deg2rad(0) deg2rad(-5)];
%open gripper movement matrix
qMatrixOpen = jtraj(Close,Open,numSteps);
%close gripper movement matrix
qMatrixClose = jtraj(Open,Close,numSteps);

% Create DoBot Magician (mounted on Linear Rail) model
DOBOT = LinearDobotMagician();
segmentTrajectory = interpolatePoses([-0.01, 0, 0, 0, 0, 0], [-0.4, -pi/2, 0, 0, 0, 0], 50);
moveDobot(DOBOT, segmentTrajectory, 50);


disp('Press ENTER to Start');
pause;

% Definition of Drinks Poses in reference to ABB IRB 1200 joints
% Gin
aboveBottle1 = [pi/2.2, pi/4, pi/4, 0, -pi/2, -pi/2];    % above bottle 1
Bottle1 = [pi/2.2, pi/2.55, pi/9, 0, -pi/2, -pi/2]; % bottle 1 offset
% Vokda
aboveBottle2 = [pi/2, pi/4, pi/4, 0, -pi/2, -pi/2];    % above bottle 2
Bottle2 = [pi/2, pi/2.55, pi/9, 0, -pi/2, -pi/2]; % bottle 2 offset
% Whiskey
aboveBottle3 = [pi/1.8, pi/4, pi/4, 0, -pi/2, -pi/2];    % above bottle 3
Bottle3 = [pi/1.8, pi/2.55, pi/9, 0, -pi/2, -pi/2]; % bottle 3 offset
% Coke
aboveMixer1 = [-pi/2.2, pi/4, pi/4, 0, -pi/2, -pi/2];    % above mixer 1
Mixer1 = [-pi/2.2, pi/2.55, pi/9, 0, -pi/2, -pi/2]; % mixer 1 offset
% Lemonade
aboveMixer2 = [-pi/2, pi/4, pi/4, 0, -pi/2, -pi/2];    % above mixer 2
Mixer2 = [-pi/2, pi/2.55, pi/9, 0, -pi/2, -pi/2]; % mixer 2 offset
% Orange Juice
aboveMixer3 = [-pi/1.8, pi/4, pi/4, 0, -pi/2, -pi/2];    % above mixer 3
Mixer3 = [-pi/1.8, pi/2.55, pi/9, 0, -pi/2, -pi/2]; % mixer 3 offset

%% Run drink Making functions 

makeVodkaLemonade(IRB1200, aboveBottle2, Bottle2, aboveMixer2, Mixer2, gripperR, gripperL,numSteps);
pause(2);
makeWhiskeyANDCoke(IRB1200, aboveBottle3, Bottle3, aboveMixer1, Mixer1, gripperR, gripperL,numSteps);
pause(2);
makeVodkaOJ(IRB1200, aboveBottle2, Bottle2, aboveMixer3, Mixer3, gripperR, gripperL,numSteps);
pause(2);

% OR
% makeDrink(IRB1200, aboveBottle2, Bottle2, aboveMixer3, Mixer3);
% This function needs to be more modular to work better with gui

%% Drink Making Functions
% The drink making functions follow the same structure as the following
% function, replacing the bottle and mixer with the desired combination
% function makeDrink(IRB1200, aboveBottle, Bottle, aboveMixer, Mixer)
% disp(['Making Drink...'])
%     posesDrink = [
%         0, 0, 0, 0, 0, 0; % start pose 0
% 
%         aboveBottle;
%         Bottle;
%         aboveBottle;
% 
%         % 0, pi/2.55, pi/9, 0, -pi/2, 0;
%         0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
%         0, pi/4, pi/4, 0, -pi/2, pi/2;      % pour drink
%         0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
% 
%         aboveBottle;
%         Bottle;         % put bottle back
%         aboveBottle;
% 
%         aboveMixer;
%         Mixer;
%         aboveMixer;
% 
%         % 0, pi/2.55, pi/9, 0, -pi/2, 0;
%         0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
%         0, pi/4, pi/4, 0, -pi/2, pi/2;      % pour drink
%         0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
% 
%         aboveMixer;
%         Mixer;          % put mixer back
%         aboveMixer;
% 
%         0, 0, 0, 0, 0, 0;
%     ];
% 
%     % Initialize the trajectory with the first pose
%     trajectory = posesDrink(1, :);
% 
%     % Set number of steps
%     numSteps = 50;
% 
%     % Generate trajectory for each pair of consecutive poses
%     for i = 1:size(posesDrink, 1) - 1
%         startPose = posesDrink(i, :);
%         endPose = posesDrink(i + 1, :);
% 
%         % Interpolate poses to create a smooth trajectory segment
%         segmentTrajectory = interpolatePoses(startPose, endPose, numSteps);
% 
%         % Move the robot along the complete trajectory
%         moveIRB1200(IRB1200, segmentTrajectory, numSteps);
% 
%         startPose = posesDrink(i+1);
%     end
% 
%     disp(['DONE.']);
% end

function makeVodkaLemonade(IRB1200, aboveVodka, Vodka, aboveLemonade, Lemonade, gripperR, gripperL,numSteps)
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
    
    
    % Generate trajectory for each pair of consecutive poses
    for i = 1:size(posesVodkaLemonade, 1) - 1
        startPose = posesVodkaLemonade(i, :);
        endPose = posesVodkaLemonade(i + 1, :);
    
        % Interpolate poses to create a smooth trajectory segment
        segmentTrajectory = interpolatePoses(startPose, endPose, numSteps);
    
        % Move the robot along the complete trajectory
        moveIRB1200(IRB1200, segmentTrajectory, numSteps, gripperR, gripperL);
    
        startPose = posesVodkaLemonade(i+1);
    end
    
    disp(['DONE.']);
end

function makeWhiskeyANDCoke(IRB1200, aboveWhiskey, Whiskey, aboveCoke, Coke, gripperR, gripperL,numSteps)
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
    
    % Generate trajectory for each pair of consecutive poses
    for i = 1:size(posesWhiskeyANDCoke, 1) - 1
        startPose = posesWhiskeyANDCoke(i, :);
        endPose = posesWhiskeyANDCoke(i + 1, :);
    
        % Interpolate poses to create a smooth trajectory segment
        segmentTrajectory = interpolatePoses(startPose, endPose, numSteps);
    
        % Move the robot along the complete trajectory
        moveIRB1200(IRB1200, segmentTrajectory, numSteps, gripperR, gripperL);
    
        startPose = posesWhiskeyANDCoke(i+1);
    end
    
    disp(['DONE.']);
end

function makeVodkaOJ(IRB1200, aboveVodka, Vodka, aboveOrangeJuice, OrangeJuice, gripperR, gripperL,numSteps)
    disp(['Making Screwdriver...'])
    posesVodkaOJ = [
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
    
        aboveOrangeJuice;
        OrangeJuice;
        aboveOrangeJuice;
    
        % 0, pi/2.55, pi/9, 0, -pi/2, 0;
        0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
        0, pi/4, pi/4, 0, -pi/2, pi/2;      % pour drink
        0, pi/4, pi/4, 0, -pi/2, -pi/2;     % above 'cup home'
        
        aboveOrangeJuice;
        OrangeJuice;
        aboveOrangeJuice;

        0, 0, 0, 0, 0, 0;
    ];
    
    % Initialize the trajectory with the first pose
    trajectory = posesVodkaOJ(1, :);
    
    % Generate trajectory for each pair of consecutive poses
    for i = 1:size(posesVodkaOJ, 1) - 1
        startPose = posesVodkaOJ(i, :);
        endPose = posesVodkaOJ(i + 1, :);
    
        % Interpolate poses to create a smooth trajectory segment
        segmentTrajectory = interpolatePoses(startPose, endPose, numSteps);
    
        % Move the robot along the complete trajectory
        moveIRB1200(IRB1200, segmentTrajectory, numSteps, gripperR, gripperL);
    
        startPose = posesVodkaOJ(i+1);
    end
    
    disp(['DONE.']);
end


%% Robot moving functions
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

% function moveIRB1200(robot, trajectory, numSteps)
%     % Move the UR3 robot along a given trajectory
%     for i = 1:numSteps
%         % Calculate the end-effector transformation using forward kinematics
%         endEffectorPose = robot.model.fkine(trajectory{i});
% 
%         % Solve joint angles using inverse kinematics
%         qSol = robot.model.ikine(endEffectorPose, 'q0', zeros(1, 6));
% 
%         robot.model.animate(qSol); % Animate the robot's motion
%         drawnow;
%     end
% end

function moveIRB1200(robot, trajectory, numSteps, gripperR, gripperL)
    % Move the UR3 robot along a given trajectory
    for i = 1:numSteps
        % Calculate the end-effector transformation using forward kinematics
        endEffectorPose = robot.model.fkine(trajectory{i});

        % Solve joint angles using inverse kinematics
        qSol = robot.model.ikine(endEffectorPose, 'q0', zeros(1, 6));
        
        % Animate the robot's motion
        robot.model.animate(qSol); 

        % Adjust the location of the R and L gripper to the updated end effector pose
        gripperR.model.base = robot.model.fkine(robot.model.getpos).T * trotx(-pi/2) * trotz(pi); 
        gripperL.model.base = robot.model.fkine(robot.model.getpos).T * trotx(pi/2);
        
        % Animate the change of position for gripper
        gripperR.model.animate(gripperR.model.getpos);
        gripperL.model.animate(gripperL.model.getpos);
        
        drawnow;
    end
end

function closeGripper(gripperR,gripperL)
        pause(0.5)
        %close gripper R and L
        display('Closing Gripper...');
        gripperR.model.animate(qMatrixClose);
        gripperL.model.animate(qMatrixClose);
        pause(0.5)
end

function openGripper(gripperR,gripperL)
        pause(0.5)
        %open gripper R and L
        display('Open Gripper...');
        gripperR.model.animate(qMatrixOpen);
        gripperL.model.animate(qMatrixOpen);
        pause(0.5)
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
