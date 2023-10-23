classdef ABBIRB1200 < RobotBaseClass
    %% ABBIRB1200

    properties(Access = public)  
        plyFileNameStem = 'ABBIRB1200';       
    end
    methods (Access = public)
%% Constructor 
        function self = ABBIRB1200(baseTr)
			self.CreateModel();
            if nargin == 1			
				self.model.base = self.model.base.T * transl(0.8,-0.4,0.5) * baseTr ;
            end            

            % Overiding the default workspace for this small robot
            % self.workspace = [-0.6 0.6 -0.6 0.6 -0.01 0.8];   
            self.PlotAndColourRobot();         
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link([0      0.3991     0          -pi/2  0]);
            link(2) = Link([-pi/2  0          0.450      0      0]);
            link(3) = Link([0      0          0.042     -pi/2  0]);
            link(4) = Link([0      0.451      0          pi/2   0]);
            link(5) = Link([0      0          0         -pi/2  0]);
            link(6) = Link([0      0.082      0          0     0]);

            % % Incorporate joint limits
            link(1).qlim = [-165 165]*pi/180;
            link(2).qlim = [-110 110]*pi/180;
            link(3).qlim = [-110 70]*pi/180;
            link(4).qlim = [-160 160]*pi/180;
            link(5).qlim = [-120 120]*pi/180;
            link(6).qlim = [-400 400]*pi/180;

            % link(1).offset =  -pi;
            link(2).offset =  -pi/2;

            self.model = SerialLink(link,'name',self.name); 
        end    
    end
end