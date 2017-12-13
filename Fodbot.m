
classdef Fodbot < FodbotOffline
    
    methods(Access = public)
        function this = Fodbot()

            sGroup = HebiLookup.newConnectedGroupFromName('*','SA065');

            this@FodbotOffline(2 + sGroup.getNumModules);
            
            this.xGroup = HebiLookup.newGroupFromNames('*', ...
                               {'X-00036', ...
                                'X-00031'});
            this.sGroup = sGroup;
            
            this.fullGroup = HebiLookup.newGroupFromNames('*',...
                [this.xGroup.getInfo.name;
                 this.sGroup.getInfo.name]);
            setgains2; %%
            
            this.arm = HebiArm(this.fullGroup, this.fullKin);
%             this.arm = ContactArm(this.fullGroup, this.fullKin);
            
            this.allModules = 1:this.fullGroup.getNumModules();

            this.fullGroup.setCommandLifetime(0);
            this.setWaypoints();
        end
        
        function plot(this)
            plot@FodbotOffline(this,...
                               this.fullGroup.getNextFeedback().position);
        end

        function setWaypoints(this)
            z = zeros(1,10);
            
%            wp.Forward = [zeros(1,10) z]; %[.5 0 1.1 .7 0 .5 -.5 -.2 z];
            wp.RightLeft = [0 0.6 0.3837    1.1710    1.0434   -0.2602   -1.1788    0.8500    0.1684    0.7568 z];
            wp.ForwardLeft = [wp.RightLeft(1) wp.RightLeft(2)+pi/2 wp.RightLeft(3:end)];
            wp.LeftLeft = [wp.RightLeft(1) wp.RightLeft(2)+pi wp.RightLeft(3:end)];
            wp.BackLeft = [wp.RightLeft(1) wp.RightLeft(2)+3*pi/2 wp.RightLeft(3:end)];
            
            wp.Forward = [0 0 -0.0011    0.9212    0.0211   -0.1799   -0.0403    0.4679   -0.0635    0.1457];
            wp.Left = [wp.Forward(1) wp.Forward(2)+pi/2 wp.Forward(3:end)];
            wp.Back = [wp.Forward(1) wp.Forward(2)+pi wp.Forward(3:end)];
            wp.Right = [wp.Forward(1) wp.Forward(2)-pi/2 wp.Forward(3:end)];
            
            wp.ForwardRight = [0 0.9 0.2165   -1.1749    1.1806    1.1149   -0.1271    0.9039    0.8553   -0.2136 z];
            wp.LeftRight = [wp.ForwardRight(1) wp.ForwardRight(2)+pi/2 wp.ForwardRight(3:end)];
            wp.BackRight = [wp.ForwardRight(1) wp.ForwardRight(2)+pi wp.ForwardRight(3:end)];
            wp.RightRight = [wp.ForwardRight(1) wp.ForwardRight(2)-pi/2 wp.ForwardRight(3:end)];
            
            wp.stowed = 1.57*[0 -1 -1 1 -1 -1 1 0  -1 -1 z];
            wp.stowedMid = wp.stowed - 0.4*sign(wp.stowed);%[0 0 -0.95 0.98 1.28 -0.80 -0.70 0.13 z];
            wp.extend = [zeros(1,10) z];
            
            wp = structfun(@(field) field(this.allModules), wp,...
                           'UniformOutput', false);
            
            this.waypoints = wp;
        end

        function endEffector = getFK(this, type)
            if nargin < 2
                type = 'EndEffector';
            end
            
            endEffector = this.fullKin.getFK(type, ...
                          this.fullGroup.getNextFeedback.position);
        end            
        
        function stow(this)
            this.arm.moveJoint([this.waypoints.stowedMid], 3);
            this.arm.moveJoint(this.waypoints.stowed, 1.5);
        end
        
        function unstow(this)
            this.arm.moveJoint(this.waypoints.stowedMid, 1.5);
        end
        
        % CENTER OF EACH
        function lookForward(this)
%            this.waypoints.Forward
            this.arm.moveJoint(this.waypoints.Forward);
        end
        
        function lookRight(this)
            this.arm.moveJoint(this.waypoints.Right);
        end
        
        function lookLeft(this)
            this.arm.moveJoint(this.waypoints.Left);
        end
        
        function lookBack(this)
            this.arm.moveJoint(this.waypoints.Back);
        end

        % LEFT SIDE OF EACH
        function lookForwardLeft(this)
            this.arm.moveJoint(this.waypoints.ForwardLeft);
        end
        
        function lookRightLeft(this)
            this.arm.moveJoint(this.waypoints.RightLeft);
        end
        
        function lookLeftLeft(this)
            this.arm.moveJoint(this.waypoints.LeftLeft);
        end
        
        function lookBackLeft(this)
            this.arm.moveJoint(this.waypoints.BackLeft);
        end      
        
        % RIGHT SIDE OF EACH
        function lookForwardRight(this)
            this.arm.moveJoint(this.waypoints.ForwardRight);
        end
        
        function lookRightRight(this)
            this.arm.moveJoint(this.waypoints.RightRight);
        end
        
        function lookLeftRight(this)
            this.arm.moveJoint(this.waypoints.LeftRight);
        end
        
        function lookBackRight(this)
            this.arm.moveJoint(this.waypoints.BackRight);
        end      
        
        function lookExtend(this)
            this.arm.moveJoint(this.waypoints.extend);
        end

    end
        
    properties
        sGroup
        xGroup
        fullGroup
        arm
        allModules
        waypoints
    end
end