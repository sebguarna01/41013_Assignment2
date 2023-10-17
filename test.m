%% Sample Draft File
% test
% workspace = PlaceObject('Workspace.ply', [0,0,0]);
% workspace = PlaceObject('Workspace_Simple.ply', [0,0,0]);
% 
% hold on;
% 
% % Create an ABB IRB 120 model
% IRB120 = IRB120();

% Define DH parameters of the ABB IRB 120
L1 = Link('d', 0.290, 'a', 0, 'alpha', pi/2, 'offset', 0);
L2 = Link('d', 0, 'a', -0.270, 'alpha', 0, 'offset', -pi/2);
L3 = Link('d', 0, 'a', -0.070, 'alpha', pi/2, 'offset', 0);
L4 = Link('d', 0.302, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2);
L5 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', 0);
L6 = Link('d', 0.072, 'a', 0, 'alpha', 0, 'offset', 0);

L1.qlim = [-165 165]*pi/180;
L2.qlim = [-110 110]*pi/180;
L3.qlim = [-70 110]*pi/180;
L4.qlim = [-160 160]*pi/180;
L5.qlim = [-120 120]*pi/180;
L6.qlim = [-360 360]*pi/180;

% Create robot model
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'robot120');

% Initial joint angles (all zeros)
q = zeros(1, 6);

% Define workspace limits
workspace = [-2 2 -2 2 0 2];

% Plot robot
robot.plot(q, 'workspace', workspace);

% Teach the robot
robot.teach(q);