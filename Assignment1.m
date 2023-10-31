function Assignment1()

%define steps for movement of the robot
steps = 20;


%define q 
q = zeros(1,7);

%set up the workspace size
axis equal
axis([-3 3 -3 3 0 2])
hold on;

%initialise LinearUR3 robot as variable r
display('Initialising...');
r = LinearUR3;

%set the Gripper Origin to Robot End Effector Pose and mount them to the
%robot with the right rotation that the left and right Gripper face each
%other
gripperOrigin = r.model.fkine(r.model.getpos());
gripperL = Gripper(gripperOrigin.T * trotx(pi/2));
gripperR = Gripper(gripperOrigin.T *trotx(-pi/2) * trotz(pi));

%change base position of robot
r.model.base = transl(0, 0, 0.5) *trotx(pi/2) * troty(pi/2);

%insert concrete floor
surf([-3,-3;3,3],[-3,3;-3,3],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
surf([-3,3;-3,3],[-3,-3;-3,-3],[-0,-0;3,3],'CData',flipdim(flip(imread('background1.jpg')),2),'FaceColor','texturemap');
surf([-3,-3;-3,-3],[-3,3;-3,3],[-0,-0;3,3],'CData',flipdim(flip(imread('background1.jpg')),2),'FaceColor','texturemap');

%insert man that operates robot
PlaceObject('personMaleConstruction.ply', [-1.5 -2 0]);
PlaceObject('Lego.ply', [0 2.3 0.7]);



%position of the robot used for different calculations on other objects
posRobot = r.model.base();
posRobot1 = posRobot.T;%*transl(0,0,-0.5);
posRobot = posRobot.t'+[0 0 -0.5]; 

%Show Radius of the Robot End effector
%PlaceObject('Radius2.ply', posRobot+[-0.4 0 0.5]);
%Reach maximumUR3Reach = 0.5

%insert table
table = PlaceObject('tableBrown2.1x1.4x0.5m.ply', posRobot+[0, 0, 0]);

%insert safety structure
PlaceObject('fireExtinguisherElevated.ply', posRobot+[2.5 2.5 0.5]);
PlaceObject('emergencyStopWallMounted.ply', [-1 -2.95 1.25]);
PlaceObject('barrier1.5x0.2x1m.ply', posRobot+[-0.75 -1 0]);
PlaceObject('barrier1.5x0.2x1m.ply', posRobot+[-0.75 1 0]);
PlaceObject('barrier1.5x0.2x1m.ply', posRobot+[0.75 -1 0]);
PlaceObject('barrier1.5x0.2x1m.ply', posRobot+[0.75 1 0]);

%place cones next to table
PlaceObject('cone.ply', posRobot+[-1.5 0.5 0]);
PlaceObject('cone.ply', posRobot+[-1.5 0 0]);
PlaceObject('cone.ply', posRobot+[-1.5 -0.5 0]);
PlaceObject('cone.ply', posRobot+[1.5 0.5 0]);
PlaceObject('cone.ply', posRobot+[1.5 0 0]);
PlaceObject('cone.ply', posRobot+[1.5 -0.5 0]);


%position of the row of bricks
coordinatex = 0;
coordintatey = 0.4;


%position of the gripper above the brick on table for pickup
posB1 = posRobot+[coordinatex+0.3 coordintatey 0.62];
posB2 = posRobot+[coordinatex+0.2 coordintatey 0.62];
posB3 = posRobot+[coordinatex+0.1 coordintatey 0.62];
posB4 = posRobot+[coordinatex+0 coordintatey 0.62];
posB5 = posRobot+[coordinatex-0.1 coordintatey 0.62];
posB6 = posRobot+[coordinatex-0.2 coordintatey 0.62];
posB7 = posRobot+[coordinatex-0.3 coordintatey 0.62];
posB8 = posRobot+[coordinatex-0.4 coordintatey 0.62];
posB9 = posRobot+[coordinatex-0.5 coordintatey 0.62];

%Initialise Bricks

%PlaceBricks at 000
B1=PlaceObject('Brick.ply', [0 0 0]);
%get Verticies of Brick
B1_vertices=get(B1,'Vertices');
%Translate Brick to designated position
B1tr=transl(coordinatex+0.3, coordintatey, 0.5)*trotz(pi);
%get the transformed verticies matrix by translation
transformedVertices=[B1_vertices,ones(size(B1_vertices,1),1)]*B1tr';
%set transformed Verticies for Brick
set(B1,'Vertices',transformedVertices(:,1:3));

%continue with the other 8 bricks and doing the same as for Brick 1

B2=PlaceObject('Brick.ply', [0 0 0]);
B2_vertices=get(B2,'Vertices');
B2tr=transl(coordinatex+0.2, coordintatey, 0.5)* trotz(pi);
transformedVertices=[B2_vertices,ones(size(B2_vertices,1),1)]*B2tr';
set(B2,'Vertices',transformedVertices(:,1:3));

B3=PlaceObject('Brick.ply', [0 0 0]);
B3_vertices=get(B3,'Vertices');
B3tr=transl(coordinatex+0.1, coordintatey, 0.5)* trotz(pi);
transformedVertices=[B3_vertices,ones(size(B3_vertices,1),1)]*B3tr';
set(B3,'Vertices',transformedVertices(:,1:3));

B4=PlaceObject('Brick.ply', [0 0 0]);
B4_vertices=get(B4,'Vertices');
B4tr=transl(coordinatex+0, coordintatey, 0.5)* trotz(pi);
transformedVertices=[B4_vertices,ones(size(B4_vertices,1),1)]*B4tr';
set(B4,'Vertices',transformedVertices(:,1:3));

B5=PlaceObject('Brick.ply', [0 0 0]);
B5_vertices=get(B5,'Vertices');
B5tr=transl(coordinatex-0.1, coordintatey, 0.5)* trotz(pi);
transformedVertices=[B5_vertices,ones(size(B5_vertices,1),1)]*B5tr';
set(B5,'Vertices',transformedVertices(:,1:3));

B6=PlaceObject('Brick.ply', [0 0 0]);
B6_vertices=get(B6,'Vertices');
B6tr=transl(coordinatex-0.2, coordintatey, 0.5)* trotz(pi);
transformedVertices=[B6_vertices,ones(size(B6_vertices,1),1)]*B6tr';
set(B6,'Vertices',transformedVertices(:,1:3));

B7=PlaceObject('Brick.ply', [0 0 0]);
B7_vertices=get(B7,'Vertices');
B7tr=transl(coordinatex-0.3, coordintatey, 0.5)* trotz(pi);
transformedVertices=[B7_vertices,ones(size(B7_vertices,1),1)]*B7tr';
set(B7,'Vertices',transformedVertices(:,1:3));

B8=PlaceObject('Brick.ply', [0 0 0]);
B8_vertices=get(B8,'Vertices');
B8tr=transl(coordinatex-0.4, coordintatey, 0.5)* trotz(pi);
transformedVertices=[B8_vertices,ones(size(B8_vertices,1),1)]*B8tr';
set(B8,'Vertices',transformedVertices(:,1:3));

B9=PlaceObject('Brick.ply', [0 0 0]);
B9_vertices=get(B9,'Vertices');
B9tr=transl(coordinatex-0.5, coordintatey, 0.5)* trotz(pi);
transformedVertices=[B9_vertices,ones(size(B9_vertices,1),1)]*B9tr';
set(B9,'Vertices',transformedVertices(:,1:3));


%brick size values
brickheight=0.0334;
brickwidth=0.133;

%define position of the wall
posBWallx = 0.3;
posBWally = 0.1;

%define target position of each brick of the brickwall
p1=transl(posBWallx,-brickwidth+posBWally,0.58+brickheight);
p2=p1*transl(0,brickwidth,0);
p3=p2*transl(0,brickwidth,0);
p4=p1*transl(0,0,brickheight);
p5=p2*transl(0,0,brickheight);
p6=p3*transl(0,0,brickheight);
p7=p4*transl(0,0,brickheight);
p8=p5*transl(0,0,brickheight);
p9=p6*transl(0,0,brickheight);

%gripper movement q values for opening gripper and closing the gripper
Open = [deg2rad(0) deg2rad(0) deg2rad(0)];
Close = [deg2rad(5) deg2rad(0) deg2rad(-5)];
%open gripper movement matrix
qMatrixOpen = jtraj(Close,Open,steps);
%close gripper movement matrix
qMatrixClose = jtraj(Open,Close,steps);


display('Starting Program...');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %TASK1 GIVEN POSE GET Q VALUES
% position = posRobot+[0.3 0.32 0.62];
% 
% q1 = r.model.ikcon(transl(position)*trotx(180,"deg"),[0 0 -1 1 -1 1 1])
% %TASK3 DEMONSTRATE THAT JOINT STATES SATISFY GIVEN POSE
% %move robot on brick for pickup 
%         qMatrix=jtraj(r.model.getpos(),q1,steps);
%         for i=1:steps
%             %animate robot model to go to on the brick for pickup
%             r.model.animate(qMatrix(i,:));
%             %changes the location of the  R and L gripper to the updated end effector pose
%             gripperR.model.base = r.model.fkine(r.model.getpos).T * trotx(-pi/2) * trotz(pi);
%             gripperL.model.base = r.model.fkine(r.model.getpos).T * trotx(pi/2);
%             %animate the change of position for gripper
%             gripperR.model.animate(gripperR.model.getpos);
%             gripperL.model.animate(gripperL.model.getpos);
%             drawnow();
%         end
% %GET POSE 
% endposition = r.model.fkine(r.model.getpos()).t
%     

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%origin=r.model.base;
%r.model.base = origin * transl(0.9,0,0);

%specify values for each iteration of the for loop
%each iteration changes the position of the brick and gets the different
%vertices that they belong to

counter = 1;
    for x=1:9 
        if counter == 1
            display('Beginning of moving Brick 1...');
            position1 = posB1;
            position2 = p1 ;
            Bx_vertices = B1_vertices;
            Bx = B1;
        end
        if counter == 2
            display('Beginning of moving Brick 2...');
            position1 = posB2;
            position2 = p2;
            Bx_vertices = B2_vertices;
            Bx = B2;
        end
        if counter == 3
            display('Beginning of moving Brick 3...');
            position1 = posB3;
            position2 = p3;
            Bx_vertices = B3_vertices;
            Bx = B3;
        end
        if counter == 4
            display('Beginning of moving Brick 4...');
            position1 = posB4;
            position2 = p4;
            Bx_vertices = B4_vertices;
            Bx = B4;
        end
        if counter == 5
            display('Beginning of moving Brick 5...');
            position1 = posB5;
            position2 = p5;
            Bx_vertices = B5_vertices;
            Bx = B5;
        end
        if counter == 6
            display('Beginning of moving Brick 6...');
            position1 = posB6;
            position2 = p6;
            Bx_vertices = B6_vertices;
            Bx = B6;
        end
        if counter == 7
            display('Beginning of moving Brick 7...');
            position1 = posB7;
            position2 = p7;
            Bx_vertices = B7_vertices;
            Bx = B7;
        end
        if counter == 8
            display('Beginning of moving Brick 8...');
            position1 = posB8;
            position2 = p8;
            Bx_vertices = B8_vertices;
            Bx = B8;
        end
        if counter == 9
            display('Beginning of moving Brick 9...');
            position1 = posB9;
            position2 = p9;
            Bx_vertices = B9_vertices;
            Bx = B9;
        end
      
        %get q values for moving robot to brick
        display('q values to get to Bricks position:');
        q1 = r.model.ikcon(transl(position1)*trotx(180,"deg"),[0 0 -1 1 -1 1 1])

        %move over the bricks position 
        qMatrix=jtraj(r.model.getpos(),q1+[0 0 deg2rad(20) deg2rad(-20) 0 0 0],steps); 
        for i=1:steps
            %animate robot model to go to over the brick for pickup
            r.model.animate(qMatrix(i,:)); 
            %changes the location of the  R and L gripper to the updated end effector pose
            gripperR.model.base = r.model.fkine(r.model.getpos).T * trotx(-pi/2) * trotz(pi); 
            gripperL.model.base = r.model.fkine(r.model.getpos).T * trotx(pi/2);
            %animate the change of position for gripper
            gripperR.model.animate(gripperR.model.getpos);
            gripperL.model.animate(gripperL.model.getpos);
            drawnow();
        end


        %move robot on brick for pickup 
        qMatrix=jtraj(r.model.getpos(),q1,steps);
        for i=1:steps
            %animate robot model to go to on the brick for pickup
            r.model.animate(qMatrix(i,:));
            %changes the location of the  R and L gripper to the updated end effector pose
            gripperR.model.base = r.model.fkine(r.model.getpos).T * trotx(-pi/2) * trotz(pi);
            gripperL.model.base = r.model.fkine(r.model.getpos).T * trotx(pi/2);
            %animate the change of position for gripper
            gripperR.model.animate(gripperR.model.getpos);
            gripperL.model.animate(gripperL.model.getpos);
            drawnow();
        end

        pause(0.5)

        %close gripper R and L
        display('Closing Gripper...');
        gripperR.model.animate(qMatrixClose);
        gripperL.model.animate(qMatrixClose);
        

        pause(0.5)
       
        %move robot with the brick up 
        qMatrix=jtraj(r.model.getpos(),q1+[0 0 deg2rad(20) deg2rad(-20) 0 0 0],steps); 
        for i=1:steps
            %animate robot model to go up over the bricks first position
            r.model.animate(qMatrix(i,:));
            %changes the location of the  R and L gripper to the updated end effector pose
            gripperR.model.base = r.model.fkine(r.model.getpos).T * trotx(-pi/2) * trotz(pi);
            gripperL.model.base = r.model.fkine(r.model.getpos).T * trotx(pi/2);
            %animate the change of position for gripper
            gripperR.model.animate(gripperR.model.getpos);
            gripperL.model.animate(gripperL.model.getpos);
            %set new position of the brick according to movement of the end
            %effector
            tr=r.model.fkine(r.model.getpos()).T * transl(0,0,0.08);
            transformedVertices=[Bx_vertices,ones(size(Bx_vertices,1),1)]*(double(tr))';
            set(Bx,'Vertices',transformedVertices(:,1:3));
            drawnow(); 
        end

        %transport brick over the location specified
        %calculate the new q position of the bricks dropoff location
            display('q values to get to Bricks final dropoff position:');
            q1 = r.model.ikcon((position2)*trotx(180,"deg"),[0 0 1 -1 -1 1 -1])
        %qMatrix for moving robot with brick over the dropoff location
            qMatrix=jtraj(r.model.getpos(),q1+[0 0 deg2rad(20) deg2rad(-10) 0 0 0],steps);
        for i=1:steps
            %animate robot model to go over the dropoff location
            r.model.animate(qMatrix(i,:));
            %changes the location of the  R and L gripper to the updated end effector pose
            gripperR.model.base = r.model.fkine(r.model.getpos).T * trotx(-pi/2) * trotz(pi);
            gripperL.model.base = r.model.fkine(r.model.getpos).T * trotx(pi/2);
            %animate the change of position for gripper
            gripperR.model.animate(gripperR.model.getpos);
            gripperL.model.animate(gripperL.model.getpos);
            %set new position of the brick according to movement of the end
            %effector
            tr=r.model.fkine(r.model.getpos()).T * transl(0,0,0.08);%distance pos and brick placement
            transformedVertices=[Bx_vertices,ones(size(Bx_vertices,1),1)]*(double(tr))';
            set(Bx,'Vertices',transformedVertices(:,1:3));
            drawnow(); 
        end


        %transport brick to location specified 
        %qMatrix for moving robot with brick to the dropoff location
            qMatrix=jtraj(r.model.getpos(),q1,steps);
        for i=1:steps
            %animate robot
            r.model.animate(qMatrix(i,:));
            %changes the location of the  R and L gripper to the updated end effector pose
            gripperR.model.base = r.model.fkine(r.model.getpos).T * trotx(-pi/2) * trotz(pi);
            gripperL.model.base = r.model.fkine(r.model.getpos).T * trotx(pi/2);
            %animate the change of position for gripper
            gripperR.model.animate(gripperR.model.getpos);
            gripperL.model.animate(gripperL.model.getpos);
            %set new position of the brick according to movement of the end
            %effector
            tr=r.model.fkine(r.model.getpos()).T * transl(0,0,0.08);
            transformedVertices=[Bx_vertices,ones(size(Bx_vertices,1),1)]*(double(tr))';
            set(Bx,'Vertices',transformedVertices(:,1:3));
            drawnow(); 
        end
            
            pause(0.5)
            
            %open gripper R and L
            display('Opening Gripper...');
            gripperR.model.animate(qMatrixOpen);
            gripperL.model.animate(qMatrixOpen);

            pause(0.5)

            %qMatrix for moving robot over the location the brick was
            %dropped off
            qMatrix=jtraj(r.model.getpos(),q1+[0 0 deg2rad(20) deg2rad(-10) 0 0 0],steps);
        for i=1:steps
            %animate robot
            r.model.animate(qMatrix(i,:));
            %changes the location of the  R and L gripper to the updated end effector pose
            gripperR.model.base = r.model.fkine(r.model.getpos).T * trotx(-pi/2) * trotz(pi);
            gripperL.model.base = r.model.fkine(r.model.getpos).T * trotx(pi/2);
            %animate the change of position for gripper
            gripperR.model.animate(gripperR.model.getpos);
            gripperL.model.animate(gripperL.model.getpos);
            drawnow();
        end
        
        %move on to next brick
        counter = counter+1;
    end
    display('Brickwall is finished!');
%% Create Point Cloud 1 hour 
        function CreatePointCloud()
            %Initialise Robot
            r = LinearUR3;
            q = zeros(1,7);
            
            %define steps and qlim
            stepRads = deg2rad(60);
            qlim = r.model.qlim;

            %define Cloud size wiht qlim and initialise point cloud
            pointCloudeSize = prod(floor((qlim(1:6,2)-qlim(1:6,1))/stepRads + 1));
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
           
            tic
            
            %test for each q value the whole range and then go to the next
            %one same as in the LAB code
            for q1 = qlim(1,1):0.1:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                %joint 7 is just assumed to be 0
                                q7 = 0
                                for q6 = qlim(6,1):stepRads:qlim(6,2)
                                    q = [q1,q2,q3,q4,q5,q6,q7];
                                    if mod(counter/1000,1) == 0
                                        q
                                    end
                                    %get position
                                    tr = r.model.fkine(q).T;
                                    e = size(tr);

                                    d = tr(1:3,4);
                                    if tr(3,4) > 0.05
                                        pointCloud(counter,:) = tr(1:3,4);
                                        counter = counter + 1;

                                    end

                                    if mod(counter/pointCloudeSize * 100,1) == 0
                                        display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                                    end
                                end
                            end
                        end
                    end
                end
            end
            display('Saving the results from the pointcloud to a file...');
            save('volumecloud2.mat',"pointCloud",'-mat');

            % 2.6 Create a 3D model showing where the end effector can be over all these samples.
            plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');

        end
%% 

            display('Loading PointCloud...');
            load("volumecloud2.mat", "pointCloud");
            
            for i = 1:size(pointCloud(:,3))
            z(i) = pointCloud(i,3) + 0.5;
            end
            
            display('Plotting PointCloud...');
            plot3(pointCloud(:,1),pointCloud(:,2),z(1,:),'r.');
            
            %Compute the convex hull of the point cloud
            [k, volume] = convhull(pointCloud(:,1), pointCloud(:,2), z(:));
            
            % Display the volume
            display('The calculated volume for the end effector reach is:');
            volume
        
end

