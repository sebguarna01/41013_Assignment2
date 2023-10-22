classdef AbbIrb120 < RobotBaseClass
    %% ABB IRB 120

    properties(Access = public)              
        plyFileNameStem = 'AbbIrb120';
    end

    methods
        %% Constructor
        function self = AbbIrb120(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end

            self.model.base = self.model.base.T * baseTr;
            % self.model.base = transl(0,0,0.5);%* trotx(pi/2) * troty(pi/2);
            self.PlotAndColourRobot();
        end

        %% Create the robot model
        function CreateModel(self)
            % Create the ABB IRB 120 model
            link(1) = Link([0       0.290    0      -pi/2    0]);
            link(2) = Link([0       0       -0.270   0      -pi/2]);
            link(3) = Link([0       0       -0.070  -pi/2    0]);
            link(4) = Link([0       0.302    0       pi/2   -pi/2]);
            link(5) = Link([0       0        0      -pi/2    0]);
            link(6) = Link([0       0.072    0       0       0]);

            % Incorporate joint limits
            link(1).qlim = [-165 165]*pi/180;
            link(2).qlim = [-110 110]*pi/180;
            link(3).qlim = [-70 110]*pi/180;
            link(4).qlim = [-160 160]*pi/180;
            link(5).qlim = [-120 120]*pi/180;
            link(6).qlim = [-360 360]*pi/180;

            % Set offset angles for specific joints
            link(2).offset = -pi/2;
            link(4).offset = -pi/2;

            self.model = SerialLink(link,'name',self.name);
        end
    end
end