classdef HebiPlotter < handle
    % HebiPlotter visualize realistic looking HEBI modules
    %
    %   HebiPlotter now supports the following HEBI component:
    %       -X5-9, X5-4, X5-1
    %       -X5Link
    %       -X5BaseBracket - This is the base bracket used on the robotic
    %                        arms of the lab. 
    %       -X5WristBracket- Bracket connecting the final X-Module at wrist
    %           
    %       -FieldableElbowJoint
    %       -FieldableStraightLink 
    %       -FieldableElbowLink
    %       -FieldableGripper
    %       -Foot - Foot of Snake Monster
    %           
    %Note:  X5 series graphics are currently only fully supported in Low
    %       Resolution
    %
    %
    %   HebiPlotter Methods (constructor):
    %      HebiPlotter  - constructor
    %
    %   HebiPlotter Methods:
    %      plot         - plots the robot in the specified configuation
    %      setBaseFrame - sets the frame of the first link
    %
    %   Examples:
    %      plt = HebiPlotter();
    %      plt.plot([.1,.1]);
    %
    %      plt = HebiPlotter(16, 'resolution', 'high');
    %      plt.plot(zeros(16,1));
    
    methods(Access = public)
        %Constructor
        function this = HebiPlotter(varargin)
        %HEBIPLOTTER
        %Arguments:
        %
        %Optional Parameters:
        %  'resolution'        - 'low' (default), 'high' 
        %  'lighting'          - 'on' (default), 'off'
        %  'frame'             - 'base' (default), 'VC', 'gravity'
        %  'JointTypes'        - cell array of joint types
        %
        %Example (S-Modules):
        %  plt = HebiPlotter()
        %  plt = HebiPlotter('resolution', 'high')
        %
        %  links = {{'FieldableElbowJoint'},
        %           {'FieldableStraightLink', 'ext1', .1, 'twist', 0},
        %           {'Foot', 'ext1', .1, 'twist', 0}};
        %  plt = HebiPlotter('JointTypes', links)  
        %
        %Example 5DOF arm:
        %     links = {{'X5-9'},...
        %     {'X5BaseBracket',...
        %          'com',[.01,.01,.01]','out',...
        %          ((rotx(pi()/2)+ [zeros(4,3),[0 .025 .055 0]'])*rotz(pi/2)),...
        %          'mass',.1},...
        %     {'X5-9'},...
        %     {'X5Link','ext',.28,'twist',pi},...
        %     {'X5-9'},...
        %     {'X5Link','ext',.285,'twist',pi},...
        %     {'X5-9'},...;
        %     {'X5WristBracket',...
        %          'com',[.01,.01,.01]','out',...
        %          (roty(pi()/2)+ [zeros(4,3),[.043 0 .04 0]'])*rotz(0),...
        %          'mass',.1},...
        %     {'X5-9'}};
        %
        %     plt = HebiPlotter('JointTypes', links,'resolution','low');


            p = inputParser;
            expectedResolutions = {'low', 'high'};
            expectedLighting = {'on','off', 'far'};
            expectedFrames = {'Base', 'VC', 'gravity','head'};
            
            addParameter(p, 'resolution', 'low', ...
                         @(x) any(validatestring(x, ...
                                                 expectedResolutions)));
            addParameter(p, 'frame', 'Base',...
                         @(x) any(validatestring(x, expectedFrames)));

            addParameter(p, 'lighting', 'on',...
                         @(x) any(validatestring(x, ...
                                                 expectedLighting)));
            addParameter(p, 'JointTypes', {});
            addParameter(p, 'drawWhen', 'now');
            addParameter(p, 'accelOffsets', []);     
            addParameter(p, 'gyroOffsets', []);   
            addParameter(p, 'figureHandle', []);


            parse(p, varargin{:});

            lowResolution = strcmpi(p.Results.resolution, 'low');

            this.firstRun = true;
            this.lighting = p.Results.lighting;
            this.setKinematicsFromJointTypes(p.Results.JointTypes);
            this.frameType = p.Results.frame;
            this.drawNow = strcmp(p.Results.drawWhen, 'now');
            this.accelOffsets = p.Results.accelOffsets;
            this.gyroOffsets = p.Results.gyroOffsets;
            this.figureHandle = p.Results.figureHandle;
            this.baseFrame = eye(4);

            this.patcher = HebiModulePatcher(lowResolution);
        end
        
        
        function plot(this, anglesOrFbk)
        % PLOT plots the robot in the configuration specified by
        % angles
                       
            if (isnumeric(anglesOrFbk))
                angles = anglesOrFbk;
                fbk = [];
                if (strcmpi(this.frameType, 'gravity'))
                    error(['Input needs to be a feedback  '...
                        '(you chose gravity frame)']);
                end
            else
                try
                    angles = anglesOrFbk.position;
                    fbk = anglesOrFbk;
                catch
                    error(['Input needs to be either a list of angles ' ...
                           'or feedback']);
                end
            end

            if(this.firstRun)
                initialPlot(this, angles, fbk);
                this.firstRun = false;
            else
                updatePlot(this, angles, fbk);
            end
            if(this.drawNow)
                drawnow
            end
        end
        
        function setBaseFrame(this, frame)
            %SETBASEFRAME sets the frame of the first link in the kinematics
            %chain
            %
            % Arguments:
            % frame (required)    -  a 4x4 homogeneous matrix 
            this.baseFrame = frame;
            this.kin.setBaseFrame(frame);
        end
        
        function setHeadFrame(this, headFrame)
        %Sets the frame of the snake head in the world coordinates. This is only used if
        %plotting in the head mode.
            this.THead = headFrame;
        end
       
    end
    
    methods(Access = private, Hidden = true)
        
        function updatePlot(this, angles, fbk)
        %UPDATEPLOT updates the link patches that were previously plotted
        %by initialPlot. 
            
            h = this.handles;
            if(~ishandle(h{1}(1)))
                error('Plotting window has been closed. Exiting program.');
            end
            
            set(groot,'CurrentFigure',this.figureHandle);
            
            this.computeAndSetBaseFrame(fbk);
            
            %--------------------------------------------------------------
            fk_in = this.kin.getForwardKinematics('Output', ...
                                                  angles);
            fk_in(:,:,2:end) = fk_in(:,:,1:end-1);
            fk_in(:,:,1) = this.kin.getBaseFrame();
            %--------------------------------------------------------------
            fk_com = this.kin.getForwardKinematics('CoM', angles);
            fk_out = this.kin.getForwardKinematics('Output',angles);       

            angleInd = 1;
            for i=1:this.kin.getNumBodies()
                this.patcher.update(h{i}, this.jointTypes{i}, ...
                                    fk_in(:,:,i),...
                                    fk_com(:,:,i), fk_out(:,:,i), ...
                                    angles(angleInd));
                if(this.patcher.usesAngle(this.jointTypes{i}{1}))
                    angleInd = min(angleInd+1, length(angles));
                end
            end
            axis([-inf inf -inf inf -inf inf]);
        end
        
        function initializeKinematics(this, numLinks)
        %INITIALIZEKINEMATICS creates a default kinematics object
        %if one has not already been assigned.
            if(this.kin.getNumBodies > 0)
                return;
            end
            
            for i=1:numLinks
                this.kin.addBody('FieldableElbowJoint');
                this.jointTypes{i} = {'FieldableElbowJoint'};
            end
        end
        
        function setKinematicsFromJointTypes(this, types)
        %Creates a HebiKinematics object that will be used to calculate the
        %Forward Kinematics when plotting. 
        %types  is a struct of structs. Each struct can be fed into
        %HebiKinematics as a link type with necessary parameters
        %There is a custom type of "Foot" for plotting the end caps.
            this.kin = HebiKinematics();
            this.jointTypes = types;
            if(length(types) == 0)
                return;
            end
            for i = 1:length(types)
                if(strcmp(types{i}{1}, 'Foot'))
                    this.kin.addBody('FieldableStraightLink', ...
                        types{i}{2:end});
           %---------------Added Code--------------------------------------         
                elseif(strcmp(types{i}{1}, 'X5WristBracket'))
                    this.kin.addBody('GenericLink',...
                        types{i}{2:end});
                elseif(strcmp(types{i}{1}, 'X5BaseBracket'))
                    this.kin.addBody('GenericLink',...
                        types{i}{2:end});
           %--------------------------------------------------------------        
                else
                    this.kin.addBody(types{i}{:});
                end
            end
        end

        function this = initialPlot(this, angles, fbk)
        %INITIALPLOT creates patches representing the CAD of the
        %manipulator, sets lighting, and labels graph
            
            if(isempty(this.figureHandle))
                this.figureHandle = figure(42);
            end
            
            if (strcmp(this.frameType, 'gravity'))
                snakeData = setupSnakeData( 'SEA Snake', length(angles));
                this.CF = ComplementaryFilter(snakeData, ...
                                              'accelOffsets', this.accelOffsets, ...
                                              'gyroOffsets', this.gyroOffsets);
            end
        
            this.initializeKinematics(length(angles));
            
            this.tempFrame = eye(4);
            this.computeAndSetBaseFrame(fbk);
            
            if(strcmp(this.lighting, 'on'))
                light('Position',[0,0,10]);
                light('Position',[5,0,10]);
                light('Position',[-5,0,10]);
                lightStyle = 'gouraud';
                strength = .3;
            elseif(strcmp(this.lighting, 'far'))
                c = [.7,.7,.7];
                light('Position',[0,0,100],'Color',c);
                light('Position',[-100,0,0],'Color',c);
                light('Position',[100,0,0],'Color',c);
                light('Position',[0,-100,0], 'Color',c);
                light('Position',[0,100,0],'Color',c);
                lightStyle = 'flat';
                strength = 1.0;
            else
                lightStyle = 'flat';
                strength = 1.0;
            end
            
            %--------------------------------------------------------------
            %Get the input frame for the module, which is actually
            %the output frame of the previous module
            fk_in = this.kin.getForwardKinematics('Output', ...
                                                  angles);
            fk_in(:,:,2:end) = fk_in(:,:,1:end-1);
            fk_in(:,:,1) = this.kin.getBaseFrame();
            %--------------------------------------------------------------
            fk_com = this.kin.getForwardKinematics('CoM', angles);
            fk_out = this.kin.getForwardKinematics('Output', angles);
            
            angleInd = 1;
            for i=1:this.kin.getNumBodies
                %------------fk_in added below-----------------------------
                this.handles{i} = this.patcher.patch(this.jointTypes{i}, ...
                                                     fk_in(:,:,i),...
                                                     fk_com(:,:,i), ...
                                                     fk_out(:,:,i), angles(angleInd));
                %----------------------------------------------------------
                if(this.patcher.usesAngle(this.jointTypes{i}{1}))
                    angleInd = min(angleInd+1, length(angles));
                end
            end
            
            axis('image');
            view([45, 35]);
            xlabel('x');
            ylabel('y');
            zlabel('z');
        end
        
        
        
        function computeAndSetBaseFrame(this, fbk)
        %fk is the forward kinematics of the modules at their center of
        %mass, fk_out is at their distal end (starting from the
        %tail)
            
            if(strcmpi(this.frameType,'Base'))
                return;
            end
            
            angles = fbk.position;
            
            %--------------------------------------------------------------
            %Get the input frame for the module, which is actually
            %the output frame of the previous module
            fk_in = this.kin.getForwardKinematics('Output', ...
                                                  angles);
            fk_in(:,:,2:end) = fk_in(:,:,1:end-1);
            fk_in(:,:,1) = this.kin.getBaseFrame();
            %--------------------------------------------------------------
            fk_com = this.kin.getForwardKinematics('CoM', angles);
            fk_out = this.kin.getForwardKinematics('Output', angles);

            if(strcmpi(this.frameType, 'VC'))
                this.tempFrame = this.tempFrame*unifiedVC(fk_com, eye(3), eye(3));
                this.setBaseFrame(inv(this.tempFrame));
            elseif (strcmpi(this.frameType, 'gravity'))
                this.CF.update(fbk);
                tailInGravity = this.CF.getInGravity('tail');
                if isempty(tailInGravity)
                    tailInGravity = eye(4);
                end
                %The module reference frames in CF are aligned for zero
                %joint angle, while in HebiKinematics they rotate from
                %tail to head of pi/2 per module around the z-axis 
                %(screw convention)
                %There is also an offset of -pi to rotate from one
                %convention to the other                   
                tailInGravity_screwConvention = tailInGravity*...
                                            rotz(-pi+length(fk_com)*pi/2);
                this.setBaseFrame(tailInGravity_screwConvention);
            elseif (strcmpi(this.frameType, 'head'))
                this.setBaseFrame(this.THead*inv(fk_out(:,:,end))*this.baseFrame);
            end
        end

    end
    
    properties(Access = public, Hidden = true)
        kin;            % Kinematics for the arm (HebiKinematics)
        jointTypes;     % list of the Hebi Joint Types (Struct)
        handles;        % reference to the plots of each link (handle)
        firstRun;       % first time plotting (boolean)
        lighting;       % lighting style (string)
        frameType;      % style of origin of plot (string)
        tempFrame;      % base frame for hebi kinematics (matrix)
        baseFrame       % Base Frame of the Hebi Kinematics Object
        drawNow;        % redraw on each iteration (boolean)
        CF;             % Complementary filter (ComplementaryFilter)
        THead;          % known frame of the head (matrix)
        accelOffsets;   % 
        gyroOffsets;
        figureHandle;   % handle for this figure (handle)
        
        patcher
    end
end
