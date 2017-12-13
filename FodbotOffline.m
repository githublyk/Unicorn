
classdef FodbotOffline < handle
    
    methods(Access = public)
        function this = FodbotOffline(numModules)
            camera = true;
            
            if(nargin < 1)
                numModules = 8;
            end
               
            this.xKin = this.createXKinematics();
            this.fullKin = this.addSKinematics(this.createXKinematics(), ...
                                               numModules - 2, ...
                                               camera);                           
            th3 = -pi/2; %base frame rotated -pi/2 about Z to allign gravity
            R3 = [1 0 0;
                     0 cos(th3) -sin(th3); 
                     0 sin(th3) cos(th3)];
            BaseFrame = [R3 [0;0;0]; [0 0 0 1]];
            this.fullKin.setBaseFrame(BaseFrame);
            this.sPlotter = HebiPlotter();

        end
        
        
        function plot(this, jointAngles)
            xEndFrame = this.xKin.getFK('EndEffector', ...
                                        jointAngles(1:2));
            
            this.sPlotter.setBaseFrame(xEndFrame);
            this.sPlotter.plot(jointAngles(3:end));
        end

    end
        
    methods(Access = private, Hidden = true)
        function kin = createXKinematics(this)
            kin = HebiKinematics();
            kin.addBody('X5-4','PosLim',[-pi/2 pi/2]); % kin.addBody('X5Joint');
            kin.addBody('X5Bracket');
            kin.addBody('X5Link','Extension',0,'Twist',pi);
            kin.addBody('X5-9','PosLim',[-inf inf]); %kin.addBody('X5Joint');
        end
        
        function kin = addSKinematics(this, kin, numModules, camera)
            for i=1:numModules
                kin.addBody('FieldableElbowJoint');
            end
            if(camera)
                T = eye(4);
                T(1:3,4) = [0;0;.06];
                % kin.addBody('GenericLink', ...
                %             'CoM', eye(4), ...
                %             'Mass', .280, ...
                %             'Output', T);
                kin.addBody('GenericLink', ...
                            'CoM', ones(3,1),...%T, ...
                            'Mass', 0, ...
                            'Output', T);
            end
        end
    end
    
    properties
        xKin
        fullKin
        sPlotter
    end
end

