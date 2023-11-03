classdef Gripper < RobotBaseClass
  

    properties(Access = public)   
        plyFileNameStem = 'Gripper';
    end
    
    methods
%% Constructor
function self = Gripper(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * troty(pi);
            %self.model.plot(zeros(1,3));
            self.PlotAndColourRobot();         
        end

%% CreateModel
        function CreateModel(self)
%             link(1) = Link('d',0,'a',0.03,'alpha',0,'qlim',deg2rad([-360 360]), 'offset',pi/6); % PRISMATIC Link
%             link(2) = Link('d',0,'a',0.03,'alpha',0,'qlim',deg2rad([-360 360]), 'offset',pi/4);
%             link(3) = Link('d',0,'a',0.03,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',pi/4);
%             
%             
%              
%             self.model = SerialLink(link,'name',self.name
             
            link(1) = Link([0      0      0.07     0      0]);
            link(2) = Link([0      0      0.04      0      0]);
            link(3) = Link([0      0      0.025      0      0]);

            link(1).qlim = [-360 360]*pi/180;
            link(2).qlim = [-360 360]*pi/180;
            link(3).qlim = [-360 360]*pi/180;

            link(1).offset = 20*pi/180;
            link(2).offset = 80*pi/180;
            link(3).offset = -10*pi/180;
            
            
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
