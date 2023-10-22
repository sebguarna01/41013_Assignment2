%% Sample Draft File
% test
% workspace = PlaceObject('Workspace.ply', [0,0,0]);
% workspace = PlaceObject('Workspace_Simple.ply', [0,0,0]);
% 
% hold on;
% 
% % Create an ABB IRB 120 model
% robot = ABBIRB1200();
% 
% disp('Press ENTER to Start');
% % pause;
% 
% % Define a list of joint configurations (poses) to move to
% poses = [
%     0, pi/4, pi/4, pi/2, 0, 0; % start pose
% 
%     0, pi/4, pi/4, pi/2, 0, 0;
%     0, pi/6, pi/4, pi/2, 0, 0;
% 
%     pi/2, pi/4, pi/4, pi/2, 0, 0;
%     pi/2, pi/6, pi/2, pi/2, 0, 0;
%     -pi/2, pi/4, pi/2, pi/2, 0, 0;
%     -pi/2, pi/6, pi/4, pi/2, 0, 0;
% ];
% 
% % Loop through each pose
% for i = 1:size(poses, 1)
%     % Get the current pose
%     currentPose = poses(i, :);
% 
%     % Move the robot to the current pose
%     robot.model.animate(currentPose);
% 
%     % Pause to observe the transformation
%     pause(2); % You can adjust the pause duration as needed
% 
%     % Display the transformation matrix of the end effector
%     disp(['Transformation Matrix for Pose ', num2str(i), ':']);
%     disp(robot.model.fkine(currentPose).T);
% end

% % Define DH parameters of the ABB IRB 120
% L1 = Link('d', 0.290, 'a', 0, 'alpha', pi/2, 'offset', 0);
% L2 = Link('d', 0, 'a', -0.270, 'alpha', 0, 'offset', -pi/2);
% L3 = Link('d', 0, 'a', -0.070, 'alpha', pi/2, 'offset', 0);
% L4 = Link('d', 0.302, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2);
% L5 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', 0);
% L6 = Link('d', 0.072, 'a', 0, 'alpha', 0, 'offset', 0);
% 
% L1.qlim = [-165 165]*pi/180;
% L2.qlim = [-110 110]*pi/180;
% L3.qlim = [-70 110]*pi/180;
% L4.qlim = [-160 160]*pi/180;
% L5.qlim = [-120 120]*pi/180;
% L6.qlim = [-360 360]*pi/180;
% 
% % Create robot model
% robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'robot120');
% 
% % Initial joint angles (all zeros)
% q = zeros(1, 6);
% 
% % Define workspace limits
% workspace = [-2 2 -2 2 0 2];
% 
% % Plot robot
% robot.plot(q, 'workspace', workspace);
% 
% % Teach the robot
% robot.teach(q);