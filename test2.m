%% Sample Draft File
% Set up the workspace size
axis equal
axis([-3 3 -3 3 0 2])
hold on;

% Set number of steps
numSteps = 20;

%% Initialising System
disp('Initialising...');
workspace = PlaceObject(['bar.ply'], [0,0,0]);

%insert concrete floor
surf([-3,-3;3,3],[-3,3;-3,3],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

%insert safety structure
PlaceObject('fireExtinguisherElevated.ply', [0 -1.5 0.5]);
PlaceObject('emergencyStopButton.ply', [0.8 -1 0.5]);
PlaceObject('barrier1.5x0.2x1m.ply',[0 -1.3 0]);
PlaceObject('barrier1.5x0.2x1m.ply',[0 1.3 0]);
PlaceObject('barrier1.5x0.2x1m.ply',[-2 0 0]);
% add alcohol 1 - 4
gin=PlaceObject('greenbottle.ply', [0 0 0]);
gin_vertices=get(gin,'Vertices');
gintr=transl(-0.3,0.6,0.5);
transformedVertices=[gin_vertices,ones(size(gin_vertices,1),1)]*gintr';
set(gin,'Vertices',transformedVertices(:,1:3));

vodka=PlaceObject('vodkabottle.ply', [0 0 0]);
vodka_vertices=get(vodka,'Vertices');
vodkatr=transl(-0.4,0.6,0.5);
transformedVertices=[vodka_vertices,ones(size(vodka_vertices,1),1)]*vodkatr';
set(vodka,'Vertices',transformedVertices(:,1:3));

whiskey=PlaceObject('rumbottle.ply', [0 0 0]);
whiskey_vertices=get(whiskey,'Vertices');
whiskeytr=transl(-0.5,0.6,0.5);
transformedVertices=[whiskey_vertices,ones(size(whiskey_vertices,1),1)]*whiskeytr';
set(whiskey,'Vertices',transformedVertices(:,1:3));

% rum = PlaceObject('rumbottle.ply', [-0.2296,0.5543,0.5]);

% add mixers 1 - 3
coke=PlaceObject('greenbottle.ply', [0 0 0]);
coke_vertices=get(coke,'Vertices');
coketr=transl(-0.3,-0.6,0.5);
transformedVertices=[coke_vertices,ones(size(coke_vertices,1),1)]*coketr';
set(coke,'Vertices',transformedVertices(:,1:3));

lemonade=PlaceObject('greenbottle.ply', [0 0 0]);
lemonade_vertices=get(lemonade,'Vertices');
lemonadetr=transl(-0.4,-0.6,0.5);
transformedVertices=[lemonade_vertices,ones(size(lemonade_vertices,1),1)]*lemonadetr';
set(lemonade,'Vertices',transformedVertices(:,1:3));

orangeJuice=PlaceObject('greenbottle.ply', [0 0 0]);
orangeJuice_vertices=get(orangeJuice,'Vertices');
orangeJuicetr=transl(-0.5,-0.6,0.5);
transformedVertices=[orangeJuice_vertices,ones(size(orangeJuice_vertices,1),1)]*orangeJuicetr';
set(orangeJuice,'Vertices',transformedVertices(:,1:3));



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
Close = [deg2rad(25) deg2rad(0) deg2rad(-25)];
%open gripper movement matrix
qMatrixOpen = jtraj(Close,Open,numSteps);
%close gripper movement matrix
qMatrixClose = jtraj(Open,Close,numSteps);

%IMPORTANT positions
mixingpos = [0.3 0 0.8];

%Create DoBot Magician (mounted on Linear Rail) model
DOBOT = LinearDobotMagician();



disp('Press ENTER to Start');
%pause;

% Definition of Drinks Poses in reference to ABB IRB 1200 joints

%% Run drink Making functions 

makeVodkaLemonade(IRB1200, gripperR, gripperL, numSteps,mixingpos,qMatrixClose,qMatrixOpen,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)
pause(2);
% makeWhiskeyANDCoke(IRB1200, gripperR, gripperL,numSteps);
% pause(2);
% makeVodkaOJ(IRB1200, gripperR, gripperL,numSteps);
% pause(2);

% OR
% makeDrink(IRB1200, aboveBottle2, Bottle2, aboveMixer3, Mixer3);
% waitfor
% This function needs to be more modular to work better with gui

%% Drink Making Functions

function makeVodkaLemonade(IRB1200, gripperR, gripperL, numSteps,mixingpos,qMatrixClose,qMatrixOpen,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)
    disp(['Making Vodka Lemonade...'])
    
    RedSoloCup=PlaceObject('RedSoloCup.ply', [0 0 0]);
    RedSoloCup_vertices=get(RedSoloCup,'Vertices');
    RedSoloCuptr=transl(0.3,0,0.5);
    transformedVertices=[RedSoloCup_vertices,ones(size(RedSoloCup_vertices,1),1)]*RedSoloCuptr';
    set(RedSoloCup,'Vertices',transformedVertices(:,1:3));

    position = [-0.4 0.5 0.8];
    pourDrink = 0;
    b = 0;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice); 

    position = [-0.4 0.5 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)
    closeGripper(gripperR,gripperL,qMatrixClose);
    b = 1;
    position = mixingpos;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)

    pourDrink = 1;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)

    pourDrink = 2;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)
   
    pourDrink = 0;
    position = [-0.4 0.5 0.8];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)
    
    position = [-0.4 0.5 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)
    b = 0;
    openGripper(gripperR,gripperL,qMatrixOpen);
%%%%%Lenonade
    position = [-0.4 -0.5 0.8];
    pourDrink = 0;
    b = 0;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice); 

    position = [-0.4 -0.5 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)
    closeGripper(gripperR,gripperL,qMatrixClose);
    b = 5;
    position = mixingpos;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)

    pourDrink = 1;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)

    pourDrink = 2;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)
   
    pourDrink = 0;
    position = [-0.4 -0.5 0.8];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)
    
    position = [-0.4 -0.5 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)
    b = 0;
    openGripper(gripperR,gripperL,qMatrixOpen);
    
    
    
    disp(['DONE.']);
end

function makeWhiskeyANDCoke(IRB1200, aboveWhiskey, Whiskey, aboveCoke, Coke, gripperR, gripperL,numSteps)
    disp(['Making Whiskey and Coke...'])

   
 

    
    disp(['DONE.']);
end

function makeVodkaOJ(IRB1200, aboveVodka, Vodka, aboveOrangeJuice, OrangeJuice, gripperR, gripperL,numSteps)
    disp(['Making Screwdriver...'])
    
    
    
    
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

function moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice)
     
        q1 = IRB1200.model.ikcon(transl(position)*trotx(270,"deg"),[1 1 1 0 0 0])
        
        if pourDrink == 1
            q1 = IRB1200.model.getpos() + [0 0 0 0 0 deg2rad(135)];
        end
        if pourDrink == 2
            q1 = IRB1200.model.getpos() + [0 0 0 0 0 deg2rad(-135)];
        end 
        %move over the bricks position 
        qMatrix = jtraj(IRB1200.model.getpos(),q1,numSteps);
        for i=1:numSteps
            %animate robot model to go to over the brick for pickup
            IRB1200.model.animate(qMatrix(i,:)); 
            %changes the location of the  R and L gripper to the updated end effector pose
            gripperR.model.base = IRB1200.model.fkine(IRB1200.model.getpos).T * trotx(-pi/2) * trotz(pi); 
            gripperL.model.base = IRB1200.model.fkine(IRB1200.model.getpos).T * trotx(pi/2);
            %animate the change of position for gripper
            gripperR.model.animate(gripperR.model.getpos);
            gripperL.model.animate(gripperL.model.getpos);
            if b ~= 0
                movebottle(IRB1200,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice,b);
            end
            drawnow();
        end
end

function movebottle(IRB1200,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice,b)
    tr=IRB1200.model.fkine(IRB1200.model.getpos()).T * trotx(90,'deg') * transl(0,0.1,-0.2);
    switch b
        case 0
                %no bottle being held
        case 1  
                transformedVertices=[vodka_vertices,ones(size(vodka_vertices,1),1)]*(double(tr))';
                set(vodka,'Vertices',transformedVertices(:,1:3));
        case 2                
                transformedVertices=[gin_vertices,ones(size(gin_vertices,1),1)]*(double(tr))';
                set(gin,'Vertices',transformedVertices(:,1:3));
        case 3             
                transformedVertices=[whiskey_vertices,ones(size(whiskey_vertices,1),1)]*(double(tr))';
                set(whiskey,'Vertices',transformedVertices(:,1:3));
        case 4                
                transformedVertices=[coke_vertices,ones(size(coke_vertices,1),1)]*(double(tr))';
                set(coke,'Vertices',transformedVertices(:,1:3));
        case 5                
                transformedVertices=[lemonade_vertices,ones(size(lemonade_vertices,1),1)]*(double(tr))';
                set(lemonade,'Vertices',transformedVertices(:,1:3));
        case 6
                transformedVertices=[orangeJuice_vertices,ones(size(orangeJuice_vertices,1),1)]*(double(tr))';
                set(orangeJuice,'Vertices',transformedVertices(:,1:3));

    end
end


function closeGripper(gripperR,gripperL,qMatrixClose)
        pause(0.5)
        %close gripper R and L
        display('Closing Gripper...');
        gripperR.model.animate(qMatrixClose);
        gripperL.model.animate(qMatrixClose);
        pause(0.5)
end

function openGripper(gripperR,gripperL,qMatrixOpen)
        pause(0.5)
        %open gripper R and L
        display('Open Gripper...');
        gripperR.model.animate(qMatrixOpen);
        gripperL.model.animate(qMatrixOpen);
        pause(0.5)
end

% e stop pressed by boolean is 1 
%somhow restart loop
