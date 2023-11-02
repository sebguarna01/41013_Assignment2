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
gintr=transl(-0.3,0.5,0.5);
transformedVertices=[gin_vertices,ones(size(gin_vertices,1),1)]*gintr';
set(gin,'Vertices',transformedVertices(:,1:3));

vodka=PlaceObject('vodkabottle.ply', [0 0 0]);
vodka_vertices=get(vodka,'Vertices');
vodkatr=transl(-0.4,0.5,0.5);
transformedVertices=[vodka_vertices,ones(size(vodka_vertices,1),1)]*vodkatr';
set(vodka,'Vertices',transformedVertices(:,1:3));

whiskey=PlaceObject('rumbottle.ply', [0 0 0]);
whiskey_vertices=get(whiskey,'Vertices');
whiskeytr=transl(-0.5,0.5,0.5);
transformedVertices=[whiskey_vertices,ones(size(whiskey_vertices,1),1)]*whiskeytr';
set(whiskey,'Vertices',transformedVertices(:,1:3));

% add mixers 1 - 3
coke=PlaceObject('rumbottle.ply', [0 0 0]);
coke_vertices=get(coke,'Vertices');
coketr=transl(-0.3,0.8,0.5);
transformedVertices=[coke_vertices,ones(size(coke_vertices,1),1)]*coketr';
set(coke,'Vertices',transformedVertices(:,1:3));

lemonade=PlaceObject('greenbottle.ply', [0 0 0]);
lemonade_vertices=get(lemonade,'Vertices');
lemonadetr=transl(-0.4,0.8,0.5);
transformedVertices=[lemonade_vertices,ones(size(lemonade_vertices,1),1)]*lemonadetr';
set(lemonade,'Vertices',transformedVertices(:,1:3));

orangeJuice=PlaceObject('orangebottle.ply', [0 0 0]);
orangeJuice_vertices=get(orangeJuice,'Vertices');
orangeJuicetr=transl(-0.5,0.8,0.5);
transformedVertices=[orangeJuice_vertices,ones(size(orangeJuice_vertices,1),1)]*orangeJuicetr';
set(orangeJuice,'Vertices',transformedVertices(:,1:3));


%Create DoBot Magician (mounted on Linear Rail) model
DOBOT = LinearDobotMagician();

hold on

disp('Press ENTER to Start');
pause;




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
mixingpos = [0.2 -0.15 0.7];
% Definition of Drinks Poses in reference to ABB IRB 1200 joints

%% Run drink Making functions 

makeVodkaLemonade(IRB1200, gripperR, gripperL, numSteps,mixingpos,qMatrixClose,qMatrixOpen,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice,DOBOT);
pause(2);
makeWhiskeyANDCoke(IRB1200, gripperR, gripperL, numSteps,mixingpos,qMatrixClose,qMatrixOpen,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice,DOBOT);
pause(2);
makeVodkaOJ(IRB1200, gripperR, gripperL, numSteps,mixingpos,qMatrixClose,qMatrixOpen,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice,DOBOT);  
pause(2);

% OR
% makeDrink(IRB1200, aboveBottle2, Bottle2, aboveMixer3, Mixer3);
% waitfor
% This function needs to be more modular to work better with gui

%% Drink Making Functions

function makeVodkaLemonade(IRB1200, gripperR, gripperL, numSteps,mixingpos,qMatrixClose,qMatrixOpen,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice,DOBOT)
    disp(['Making Vodka Lemonade...'])

    RedSoloCup=PlaceObject('RedSoloCup.ply', [0 0 0]);
    RedSoloCup_vertices=get(RedSoloCup,'Vertices');
    RedSoloCuptr=transl(0.7,0,0.5);
    transformedVertices=[RedSoloCup_vertices,ones(size(RedSoloCup_vertices,1),1)]*RedSoloCuptr';
    set(RedSoloCup,'Vertices',transformedVertices(:,1:3));

    movecup = 0;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.65 0 0.65];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0);
   
    movecup = 1;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.5 0 0.8];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0);
     
    movecup = 1;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.25 0 0.65];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0);

    movecup = 0;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.5 0 0.7];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0); 

    position = [-0.4 0.4 0.8];
    pourDrink = 0;
    b = 0;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice); 

    position = [-0.4 0.4 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    closeGripper(gripperR,gripperL,qMatrixClose);
    b = 1;
    
    position = [-0.4 0.4 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    

    position = mixingpos;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);

    pourDrink = 1;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);

    pourDrink = 2;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
   
    pourDrink = 0;
    position = [-0.4 0.4 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
    position = [-0.4 0.4 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    b = 0;
    openGripper(gripperR,gripperL,qMatrixOpen);

    position = [-0.4 0.4 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
%%%%%Lemonade
    position = [-0.4 0.7 1];
    pourDrink = 0;
    b = 0;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice); 

    position = [-0.4 0.7 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    closeGripper(gripperR,gripperL,qMatrixClose);
    b = 5;

    position = [-0.4 0.7 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
    position = mixingpos;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);

    pourDrink = 1;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);

    pourDrink = 2;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
   
    pourDrink = 0;
    position = [-0.4 0.7 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
    position = [-0.4 0.7 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    b = 0;
    openGripper(gripperR,gripperL,qMatrixOpen);

    position = [-0.4 0.7 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
    movecup = 0;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.25 0 0.65];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0);
   
    movecup = 1;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.5 0 0.8];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0);
     
    movecup = 1;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.65 -0.5 0.65];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0);

    movecup = 0;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.5 0 0.7];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0); 
    
    
    disp(['DONE.']);
end

function makeWhiskeyANDCoke(IRB1200, gripperR, gripperL, numSteps,mixingpos,qMatrixClose,qMatrixOpen,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice,DOBOT)
    disp(['Making Whiskey and Coke...'])

    RedSoloCup=PlaceObject('RedSoloCup.ply', [0 0 0]);
    RedSoloCup_vertices=get(RedSoloCup,'Vertices');
    RedSoloCuptr=transl(0.7,0,0.5);
    transformedVertices=[RedSoloCup_vertices,ones(size(RedSoloCup_vertices,1),1)]*RedSoloCuptr';
    set(RedSoloCup,'Vertices',transformedVertices(:,1:3));

    movecup = 0;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.65 0 0.65];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0);
   
    movecup = 1;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.5 0 0.8];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0);
     
    movecup = 1;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.25 0 0.65];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0);

    movecup = 0;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.5 0 0.7];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0);
    
    position = [-0.5 0.4 0.8];
    pourDrink = 0;
    b = 0;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice); 

    position = [-0.5 0.4 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    closeGripper(gripperR,gripperL,qMatrixClose);
    b = 3;
    
    position = [-0.5 0.4 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    

    position = mixingpos;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);

    pourDrink = 1;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);

    pourDrink = 2;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
   
    pourDrink = 0;
    position = [-0.5 0.4 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
    position = [-0.5 0.4 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    b = 0;
    openGripper(gripperR,gripperL,qMatrixOpen);

    position = [-0.5 0.4 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
%%%%%Coke
    position = [-0.3 0.7 1];
    pourDrink = 0;
    b = 0;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice); 

    position = [-0.3 0.7 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    closeGripper(gripperR,gripperL,qMatrixClose);
    b = 4;

    position = [-0.3 0.7 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
    position = mixingpos;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);

    pourDrink = 1;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);

    pourDrink = 2;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
   
    pourDrink = 0;
    position = [-0.3 0.7 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
    position = [-0.3 0.7 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    b = 0;
    openGripper(gripperR,gripperL,qMatrixOpen);

    position = [-0.3 0.7 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
    movecup = 0;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.25 0 0.65];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0);
   
    movecup = 1;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.5 0 0.8];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0);
     
    movecup = 1;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.65 0 0.65];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0);

    movecup = 0;
    position1 = (DOBOT.model.fkine(DOBOT.model.getpos()).t)';
    position2 = [0.5 0 0.7];
    q0 = DOBOT.model.getpos();
    [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0); 
    

    disp(['DONE.']);
end

function makeVodkaOJ(IRB1200, gripperR, gripperL, numSteps,mixingpos,qMatrixClose,qMatrixOpen,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice,DOBOT)
    disp(['Making Screwdriver...'])
    position = [-0.4 0.4 0.8];
    pourDrink = 0;
    b = 0;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice); 

    position = [-0.4 0.4 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    closeGripper(gripperR,gripperL,qMatrixClose);
    b = 1;
    
    position = [-0.4 0.4 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice); 

    position = mixingpos;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);

    pourDrink = 1;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);

    pourDrink = 2;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
   
    pourDrink = 0;
    position = [-0.4 0.4 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
    position = [-0.4 0.4 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    b = 0;
    openGripper(gripperR,gripperL,qMatrixOpen);

    position = [-0.4 0.4 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
%%%%%OrangeJuice
    position = [-0.5 0.7 1];
    pourDrink = 0;
    b = 0;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice); 

    position = [-0.5 0.7 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    closeGripper(gripperR,gripperL,qMatrixClose);
    b = 6;

    position = [-0.5 0.7 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
    position = mixingpos;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);

    pourDrink = 1;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);

    pourDrink = 2;
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
   
    pourDrink = 0;
    position = [-0.5 0.7 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
    position = [-0.5 0.7 0.7];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    b = 0;
    openGripper(gripperR,gripperL,qMatrixOpen);

    position = [-0.5 0.7 1];
    moveIRB1200(IRB1200, numSteps, gripperR, gripperL, position,pourDrink,b,vodka_vertices,gin_vertices,whiskey_vertices,coke_vertices,lemonade_vertices,orangeJuice_vertices,vodka,gin,whiskey,coke,lemonade,orangeJuice);
    
    
    
    disp(['DONE.']);
end


%% Robot moving functions
% function moveDobot(DOBOT,numSteps,position,RedSoloCup_vertices,RedSoloCup,movecup)
%     % Move the Dobot Magician robot along a given trajectory
%     q2 = DOBOT.model.ikcon(transl(position),[-0.3601    2.3562    1.3963    1.4835    0    1.4835])
%     qMatrix = jtraj(DOBOT.model.getpos(),q2,numSteps);
%     
%     for i = 1:numSteps   
%         DOBOT.model.animate(qMatrix(i,:)); 
% 
%         if movecup == 1
%             tr=DOBOT.model.fkine(DOBOT.model.getpos()).T * transl(0,0,-0.1);
%             transformedVertices=[RedSoloCup_vertices,ones(size(RedSoloCup_vertices,1),1)]*(double(tr))';
%             set(RedSoloCup,'Vertices',transformedVertices(:,1:3));
%         end
%         drawnow;
%     end
%      keyboard
% end

function [qMatrix,steps]=RMRCmoveDobot(DOBOT,numSteps,position1,position2,RedSoloCup_vertices,RedSoloCup,movecup,q0)
            t = 1;                          % Total time (s)
            deltaT = 0.05;                % Control frequency
            steps = t/deltaT;                 % No. of steps for simulation
            % delta = 2*pi/steps;             % Small angle change
            epsilon = 0.05;                   % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
 
            % % 1.2) Allocate array data
            m = zeros(steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(steps,6);       % Array for joint angles
            qdot = zeros(steps,6);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,steps);             % Array for x-y-z trajectory
            % positionError = zeros(3,steps); % For plotting trajectory error
            % angleError = zeros(3,steps);    % For plotting trajectory error
 
            % 1.3) Set up trajectory, initial pose
            x1 = position1;
            x2 = position2;
            
            % 3.7
            s = lspb(0,1,steps); % Create interpolation scalar
            for i = 1:steps
                x(:,i) = x1*(1-s(i)) + s(i)*x2; % Create trajectory in x-y plane
                theta(1,i) = 0;
                theta(2,i) = 0;
                theta(3,i) = 0;
            end
 
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            % q0 = zeros(1,6);                                                          % Initial guess for joint angles
            qMatrix(1,:) = DOBOT.model.ikcon(T,q0);                                     % Solve joint angles to achieve first waypoint                        
 
            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                % UPDATE: fkine function now returns an SE3 object. To obtain the
                % Transform Matrix, access the variable in the object 'T' with '.T'.
                T = DOBOT.model.fkine(qMatrix(i,:)).T;                                  % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                % deltaTheta = tr2rpy(Rd*Ra');                                          % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = DOBOT.model.jacob0(qMatrix(i,:));                                   % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < DOBOT.model.qlim(j,1)          % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > DOBOT.model.qlim(j,2)      % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                      	% Update next joint state based on joint velocities
                % positionError(:,i) = x(:,i+1) - T(1:3,4);                             % For plotting
                % angleError(:,i) = deltaTheta;                                         % For plotting
            end
            for i = 1:steps   
                    DOBOT.model.animate(qMatrix(i,:));
                    if movecup == 1
                        tr=DOBOT.model.fkine(DOBOT.model.getpos()).T * transl(0.05,0,-0.15);
                        transformedVertices=[RedSoloCup_vertices,ones(size(RedSoloCup_vertices,1),1)]*(double(tr))';
                        set(RedSoloCup,'Vertices',transformedVertices(:,1:3));
                    end
                    drawnow;
            end
             %plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)                            % Uncomment this to check the trajectory
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
            %animate the chaßßnge of position for gripper
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
