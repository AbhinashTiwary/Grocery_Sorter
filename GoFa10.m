classdef GoFa10 < RobotBaseClass
    %% ABB GoFa 10 Robot with 10kg payload
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'GoFa10';
    end
    
    methods
%% Constructor
        function self = GoFa10(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();

            drawnow
        end

%% CreateModel
        function CreateModel(self)
          
            link(1) = Link('d',0.4,'a',0.15,'alpha',pi/2,'qlim',deg2rad([-270 270]), 'offset',0);
            link(2) = Link('d',0,'a',0.707,'alpha',0,'qlim',deg2rad([-180 180]), 'offset',0);
            link(3) = Link('d',0,'a',-0.11,'alpha',-pi/2,'qlim',deg2rad([-225 85]), 'offset',0); 
            link(4) = Link('d',0.637,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]), 'offset',0);
            link(5) = Link('d',0,'a',-0.08,'alpha',-pi/2,'qlim',deg2rad([-180 180]), 'offset',0);
            link(6) = Link('d',0,'a',0,'alpha',0,'qlim',deg2rad([-270 270]), 'offset',0);
            
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
