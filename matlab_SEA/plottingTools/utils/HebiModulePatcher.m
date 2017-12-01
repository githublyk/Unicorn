%This updated patchfile adds the graphics of the Hebi X-Modules and their 
% associated components to the existing fieldjoints (S-Modules)
%
%The visualization of the following graphics is now supported through the 
%patch file
%           X5-9, X5-4, X5-1
%           X5Link
%           X5BaseBracket - This is the base bracket used on the robotic
%                       arms of the lab. 
%           X5WristBracket- Bracket connecting the final X-Module at wrist
%           
%           FieldableElbowJoint
%           FieldableStraightLink 
%           FieldableElbowLink
%           FieldableGripper
%           Foot - Foot of Snake Monster
%           
%Note:      X5 series graphics are currently only fully supported in Low
%           Resolution


classdef HebiModulePatcher < handle
    methods(Access = public)
        function this = HebiModulePatcher(lowResolution)
            this.lowResolution = lowResolution;
            this.loadMeshes();
            this.lightStyle = 'gouraud';
            this.lightStrength = .3;
        end
        
        function loadMeshes(this)
        %Loads the relevant meshes
        %Based on low_res different resolution meshes will be loaded
            stldir = [fileparts(mfilename('fullpath')), '/../stl'];
            
            if(this.lowResolution)
                meshes = load([stldir,'/FieldableKinematicsPatchLowRes.mat']);
            else
                meshes = load([stldir,'/FieldableKinematicsPatch.mat']);
            end
            this.lowerS = meshes.lower;
            this.upperS = meshes.upper;
            this.elbow = meshes.elbow;
            this.grip_mobile = meshes.grip_mobile;
            this.grip_static = meshes.grip_static;
            %------------below are newly added parts----------------
            this.Base_Bracket = meshes.Base_Bracket;
            this.Lower_Clamp_ArmJoint = meshes.Lower_Clamp_ArmJoint;
            this.Upper_Clamp_ArmJoint = meshes.Upper_Clamp_ArmJoint;
            this.Wrist_Bracket = meshes.Wrist_Bracket;
            this.X_Motor = meshes.X_Motor;
            %------------above are the newly added parts------------
        end
        
        function h = patch(this, moduleType,fk_in, fk_com, fk_out, angle)
            moduleIs = @(str) strcmp(moduleType{1}, str);
            color = this.getColor(moduleType);
            stl = this.getSTL(moduleType,fk_in, fk_com, fk_out, angle);
            h = [];
            for i= 1:length(stl)
                h(1,i) = patch(stl{i},...
                      'FaceColor', color,...
                      'EdgeColor', 'none',...
                      'FaceLighting', this.lightStyle, ...
                      'AmbientStrength', this.lightStrength);
            end
        end
        
        function update(this, h, moduleType,fk_in, fk_com, fk_out, angle)
            stl = this.getSTL(moduleType,fk_in, fk_com, fk_out, angle);
            
            for i = 1:length(stl)
                set(h(1,i), 'Vertices', stl{i}.vertices(:,:));
                set(h(1,i), 'Faces',    stl{i}.faces(:,:));
            end
        end
    end
    
    methods(Static)
        
        %Below section updated to give X5-9,X5-4,X5-1 rotating drive
        function angleUsed = usesAngle(jointName)
            angleJoints = {'FieldableElbowJoint', ...
                           'FieldableGripper',...
                           'X5-9','X5-4','X5-1'};
            angleUsed = strmatch(jointName, angleJoints);
        end
        
        %color function updated to include colors of X-series components
        function color = getColor(moduleType)
            moduleIs = @(str) strcmp(moduleType{1}, str);
            color = [.5,.1,.2];
            if(moduleIs('FieldableStraightLink') || ...
               moduleIs('X5Link')...
                    || moduleIs('GenericLink')...
                    || moduleIs('X5BaseBracket'))
                color = [.5,.5,.5];
            elseif(moduleIs('Foot'))
                color = [0,0,0];
             %-------------------------------------------------------------
            elseif(moduleIs('X5WristBracket'))
                color = [1,1,1];
             %-------------------------------------------------------------
            end
        end
        
        function fv = transformSTL(fv, trans)
        %Transforms from the base frame of the mesh to the correctly
        %location in space
            fv.vertices = (trans * [fv.vertices, ones(size(fv.vertices,1), ...
                                                      1)]')';
            fv.vertices = fv.vertices(:,1:3);
        end

    end
    
    methods(Hidden = true)
        function stl = getSTL(this, moduleType,fk_in, fk_com, fk_out, angle)
            moduleIs = @(str) strcmp(moduleType{1}, str);
            tr = @this.transformSTL;
            
            if(moduleIs('FieldableElbowJoint'))
                stl{1} = tr(this.lowerS, fk_com);
                stl{2} = tr(this.upperS, fk_com*roty(angle));
            elseif(moduleIs('FieldableStraightLink'))
                stl{1} = tr(this.getCylinder(moduleType), fk_com);
            elseif(moduleIs('FieldableElbowLink'))
                stl{1} = tr(this.elbow, fk_out);
            elseif(moduleIs('FieldableGripper'))
                stl{1} = tr(this.lowerS, fk_com);
                stl{2} = tr(this.grip_mobile, fk_com*roty(angle));
                stl{3} = tr(this.grip_static, fk_com);
            elseif(moduleIs('Foot'))
                stl{1} = tr(this.getSphere(moduleType), fk_com);
            %-------------Below are the newly added modules-----
            elseif(moduleIs('X5-9'))
                stl{1} = tr(this.X_Motor, fk_in);
            elseif(moduleIs('X5-4'))
                stl{1} = tr(this.X_Motor, fk_in);
            elseif(moduleIs('X5-1'))
                stl{1} = tr(this.X_Motor, fk_in);
            elseif(moduleIs('X5BaseBracket'))
                stl{1} = tr(this.Base_Bracket, rotz(-pi/2)*fk_in);
            elseif(moduleIs('X5WristBracket'))
                stl{1} = tr(this.Wrist_Bracket,fk_in);
            elseif(moduleIs('X5Link'))
                stl{1} = tr(this.Lower_Clamp_ArmJoint,...
                         fk_in);
                stl{2} = tr(this.getCylinderX(moduleType),...
                        fk_in*[eye(4,3),[.036 0 .02 1]']*roty(pi/2));
                stl{3} = tr(this.Upper_Clamp_ArmJoint,...
                        fk_out*rotx(pi)*rotz(pi));
            else
                stl = [];
            end
        end
        
        function cyl = getCylinder(this, types)
        %Returns the patch faces and vertices of a cylinder
            p = inputParser();
            p.addParameter('ext1', .4);
            p.addParameter('twist', 0);
            parse(p, types{2:end});
            r = .025;
            h = p.Results.ext1 + .015; %Add a bit for connection section
            [x,y,z] = cylinder;
            cyl = surf2patch(r*x, r*y, h*(z -.5));
        end
        
        %--------------Cylinder plotting for Xmodule arm joint---------
        function cyl = getCylinderX(this, types)
        %Returns the patch faces and vertices of a cylinder that is used
        %for the arm Joints connecting Hebi X-Modules
            p = inputParser();
            p.addParameter('ext', .4);
            p.addParameter('twist', 0);
            parse(p, types{2:end});
            r = .015;
            h = p.Results.ext-.072; %Add a bit for connection section
            [x,y,z] = cylinder;
            cyl = surf2patch(r*x, r*y, h*(z));
 
        end
        
        function sph = getSphere(this, types)
        %Returns the patch faces and vertices of a sphere
        %The top of t'he cylinder is at the origin
            p = inputParser();
            p.addParameter('ext1', .4);
            p.addParameter('twist', 0);
            parse(p, types{2:end});
            r = p.Results.ext1;
            [x,y,z] = sphere;
            sph = surf2patch(r*x, r*y, r*z -r);
        end
    end
    
    properties
        lowerS
        upperS
        elbow
        grip_mobile
        grip_static
        
   %----X-5 addition--------
        Base_Bracket;         %bracket mounted to base x-module
        Lower_Clamp_ArmJoint; %lower clamp of armjoint
        Upper_Clamp_ArmJoint; %Upper clamp of armjoint
        Wrist_Bracket;        %bracket at arm writs
        X_Motor;              %Hebi X-Modules
   %----X-5 addition--------  
   
        lightStyle
        lightStrength
        
        lowResolution
    end
end