classdef SnakeMonsterPlotter < handle
%SNAKEMONSTERPLOTTER plots snake monster
%
%   This function is incomplete and does not 
%   plot the true version of snake monster
%
%   Snake monster is robot composed of a base
%    and 6 legs. Each leg has a total of three
%    HEBI elbow joint modules as well as two 
%    extension links
%
% SnakeMonsterPlotter methods (constructor):
%  SnakeMonsterPlotter

    methods(Access = public)
        function this = SnakeMonsterPlotter(varargin)
        %SNAKEMONSTERPLOTTER 
        %
        %Examples:
        %  plt= SnakeMonsterPlotter()
        %  plt.plot(zeros(16,1));
        %
            
            p = inputParser;
            addParameter(p,'grippers',0,@isnumeric);
            addParameter(p,'resolution','low');
            parse(p,varargin{:});
            
            this.resolution = p.Results.resolution;
            
            this.figureHandle = figure('Name','Snake Monster');
            
        
            this.rmLeg = this.getLeg();
            this.lmLeg = this.getLeg();
            this.rbLeg = this.getLeg();
            this.lbLeg = this.getLeg();
            this.rfLeg = this.getLeg();
            this.lfLeg = this.getLeg();

            grips = p.Results.grippers;
            if grips<0 || grips >2 || floor(grips) ~= grips
                message('Error.Number of grippers must be 0,1,or 2');
                error(message)
            end

            if grips >= 1
                this.rfLeg = this.getLegwithGripper('side','right');
            end
            if grips >= 2
                this.lfLeg = this.getLegwithGripper('side','left');
            end
            

            c = [.7,.7,.7];
            light('Position',[0,0,100],'Color',c);
            light('Position',[-100,0,0],'Color',c);
            light('Position',[100,0,0],'Color',c);
            light('Position',[0,-100,0], 'Color',c);
            light('Position',[0,100,0],'Color',c);
            
            this.width = .093; % based on reality
            this.length = .15;
            this.height = .03;

            
            this.firstRun = true;
            this.bodyHandle = this.patchCube(this.width, this.length, ...
                                             this.height);
            this.setBaseFrame(eye(4));
            this.grip = grips;
        end
        
        function plot(this,angles)
        %PLOT - plots snake monster in the specified configuration
        %
        %Arguments:
        %angles   - A 18 element vector of joint angles
        %
            if(this.firstRun)
                if this.grip >= 1
                    this.oldGrip1 = 0;
                end
                if this.grip >= 2
                    this.oldGrip2 = 0;
                end
            end
        
            if this.grip == 1
                if max(size(angles)) == 19
                    this.rfLeg.plot(angles([1:3 19]));
                    this.oldGrip1 = angles(19);
                else
                    this.rfLeg.plot([angles(1:3) this.oldGrip1]);
                end
                this.lfLeg.plot(angles(4:6));
            elseif this.grip == 2
                if max(size(angles)) == 19
                    this.rfLeg.plot(angles([1:3 19]));
                    this.lfLeg.plot([angles(4:6) this.oldGrip2]);
                    this.oldGrip1 = angles(19);
                elseif max(size(angles)) == 20
                    this.rfLeg.plot(angles([1:3 19]));
                    this.lfLeg.plot(angles([4:6 20]));
                    this.oldGrip1 = angles(19);
                    this.oldGrip2 = angles(20);
                end
            else
                this.rfLeg.plot(angles(1:3));
                this.lfLeg.plot(angles(4:6));
            end
            this.rmLeg.plot(angles(7:9));
            this.lmLeg.plot(angles(10:12));
            this.rbLeg.plot(angles(13:15));           
            this.lbLeg.plot(angles(16:18));
            
            
            if(this.firstRun)
                this.firstRun = false;
            end
            
            drawnow
        end
        
        function setBaseFrame(this, fr)
        %Set base frame of each joint
        %Note: There is a 1.5 degree rotation
            
            h = this.bodyHandle;
            stl = this.getCube(this.width, this.length, this.height);
            stl = HebiModulePatcher.transformSTL(stl, fr);
            set(h, 'Vertices', stl.vertices(:,:));
            set(h, 'Faces', stl.faces(:,:));

            width = this.width;
            length = .097;

            R = roty(1.5*pi/180);
            offset = 1.5*pi/180;
            
            tr = @this.trans;
            
            this.rfLeg.setBaseFrame(fr*...
                tr([width,length,0,pi/2+offset,0,pi/2]) * ...
                rotz(pi));
            this.rmLeg.setBaseFrame(fr*...
                tr([width,0,0,pi/2+offset,0,pi/2]) * ...
                rotz(pi));
            this.rbLeg.setBaseFrame(fr*...
                tr([width,-length,0,pi/2+offset,0,pi/2]) * ...
                rotz(pi));
            this.lfLeg.setBaseFrame(fr*...
                tr([-width,length,0,-pi/2-offset,0,pi/2]));
            this.lmLeg.setBaseFrame(fr*...
                tr([-width,0,0,-pi/2-offset,0,pi/2]));
            this.lbLeg.setBaseFrame(fr*...
                tr([-width,-length,0,-pi/2-offset,0,pi/2]));
        end
    end
    
    methods(Access = private, Hidden = true)
        function plt = getLeg(this)
        %Get the link types for a leg
        %This will be used to create a HebiKinematics object in HebiPlotter
        %The values come from previous values used on snake monster
        %A lot of the additions and subtractions below are because the
        %HebiKinematics objects have some base length we need to compensate
        %for.
  
            links = {{'FieldableElbowJoint'},
                     {'FieldableElbowJoint'},
                     {'FieldableStraightLink', 'ext1', .063-.0122, 'twist', pi/2},
                     %Straight links have a base length of 0.0122
                     {'FieldableElbowJoint'},
                     {'FieldableElbowLink', ...
                     'ext1', 0.046 - 0.0360, 'twist1', -pi/2, ...
                     'ext2', 0.0318 - 0.0336, 'twist2', pi},
                     %HEBI kinematics defaults to the elbow joint having
                     %0.0336m on a side. Ours (apparently) has 0.046 on the
                     %upper portion and 0.0318 on the lower
                     {'FieldableStraightLink', 'ext1', .1062+.022-0.0122, 'twist', 0},
                     %Desired length + part of foot (for pretty graphing)
                     % - base length of straight link 
                     {'Foot', 'ext1', 0.025, 'twist', 0}};
                     %Don't compensate for base length of straight link
                     %here because the sphere foot will not go all the way 
                     %to the endpoint
            plt = HebiPlotter('JointTypes', links, 'figureHandle', this.figureHandle,...
                              'lighting','off',...
                              'drawWhen','later',...
                              'resolution',this.resolution);
        end
        
        function h = patchCube(this,l,w,h)
            h = patch(this.getCube(l,w,h),...
                      'FaceColor',[.5,.5,.5],...
                      'EdgeColor',[.6,.6,.6],...
                      'FaceLighting', 'gouraud');
        end
        
        function m = trans(this, xyzrpy)
            m = eye(4);
            m(1:3, 4) = xyzrpy(1:3);
            m = m*rotz(xyzrpy(6)) *roty(xyzrpy(5))*...
                rotx(xyzrpy(4));
        end
        
        function cube = getCube(this,l,w,h)
            vert = [ l, w,-h;
                     -l, w,-h;
                     -l, w, h;
                     l, w, h;
                     -l,-w, h;
                     l,-w, h;
                     l,-w,-h;
                     -l,-w,-h;];
            fac = [1 2 3 4; 
                   4 3 5 6; 
                   6 7 8 5; 
                   1 2 8 7; 
                   6 7 1 4; 
                   2 3 5 8];
            cube.faces = fac;
            cube.vertices = vert;
        end
    end
    
    methods(Static, Access = private, Hidden = true)
        function plt = getLegwithGripper(varargin)
            %Get the link types for a leg
            %This will be used to create a HebiKinematics object in HebiPlotter
            %The values come from previous values used on snake monster
            %A lot of the additions and subtractions below are because the
            %HebiKinematics objects have some base length we need to compensate
            %for.
            
            p = inputParser;
            
            expectedSides = {'right','left'};
            addParameter(p, 'side', 'right', ...
                         @(x) any(validatestring(x,expectedSides)));
                     
            parse(p, varargin{:});
            
            right = strcmpi(p.Results.side, 'right');
            
            if right
                elbow_orient = -pi/2;
            else
                elbow_orient = pi/2;
            end
  
            links = {{'FieldableElbowJoint'},
                     {'FieldableElbowJoint'},
                     {'FieldableStraightLink', 'ext1', .063-.0122, 'twist', pi/2},
                     %Straight links have a base length of 0.0122
                     {'FieldableElbowJoint'},
                     {'FieldableElbowLink', ...
                     'ext1', 0.046 - 0.0360, 'twist1', -pi/2, ...
                     'ext2', 0.0318 - 0.0336, 'twist2', pi},
                     %HEBI kinematics defaults to the elbow joint having
                     %0.0336m on a side. Ours (apparently) has 0.046 on the
                     %upper portion and 0.0318 on the lower
                     {'FieldableStraightLink', 'ext1', 0.0551-0.0122, 'twist', 0},
                     %Desired length + part of foot (for pretty graphing)
                     % - base length of straight link
                     {'FieldableElbowLink', ...
                     'ext1', 0.046 - 0.0360, 'twist1', elbow_orient, ...
                     'ext2', 0.0318- 0.0336, 'twist2', pi},
                     %HEBI kinematics defaults to the elbow joint having
                     %0.0336m on a side. Ours (apparently) has 0.046 on the
                     %upper portion and 0.0318 on the lower
                     {'FieldableStraightLink', 'ext1', 0.0142-0.0122, 'twist', -pi/2},
                     %Adding small connector piece before gripper
                     {'FieldableGripper'}};
                     %Gripper treated as a fieldable elbow joint
            plt = HebiPlotter('JointTypes', links, 'lighting','off',...
                              'drawWhen','later','resolution',this.resolution);
        end
        
    end
    
    properties(Access = private, Hidden = true)
        lfLeg;
        lmLeg;
        lbLeg;
        rfLeg;
        rmLeg;
        rbLeg;
        firstRun;
        grip;
        oldGrip1;
        oldGrip2;
        figureHandle;
        resolution
        
        bodyHandle;
        width;
        length;
        height;
    end
end