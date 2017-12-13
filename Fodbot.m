
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
            wp.ForwardLeft = wp.RightLeft  + [0 pi/2 0 0 0 0 0 0 0 0 z];
            wp.LeftLeft = wp.RightLeft  + [0 +pi 0 0 0 0 0 0 0 0 z];
            wp.BackLeft = wp.RightLeft  + [0 -pi/2 0 0 0 0 0 0 0 0 z];
            
            wp.Forward = [0 0 -0.0011    0.3212    0.0211   -0.6799   -0.0403    0.7679   -0.0635    0.4457 z];
            wp.Left = wp.Forward + [0 pi/2 0 0 0 -0.3 0 0 0 0 z];
            wp.Back = wp.Forward + [0 -pi 0 0 0 0 0 0 0 0 z];
            wp.Right = wp.Forward + [0 -pi/2 0 0 0 -0.3 0 0 0 0 z];
            
            wp.ForwardRight = [0 0.9 0.2165   -1.1749    1.1806    1.1149   -0.1271    0.9039    0.8553   -0.2136 z];
            wp.LeftRight = wp.ForwardRight + [0 pi/2 0 0 0 0 0 0 0 0 z];
            wp.BackRight = wp.ForwardRight + [0 -pi 0 0 0 0 0 0 0 0 z];
            wp.RightRight = wp.ForwardRight + [0 -pi/2 0 0 0 0 0 0 0 0 z];
            
            wp.Inspect51 = [-0.15 -1.8 -0.6241    1.0304    0.0323   -1.1165   -0.5344   -0.7792   -1.0377   0.97];
            wp.Inspect52 = [0.32 -1.65 -0.9447    0.7239   -0.1706   -0.9530    1.0305   -0.8029   -0.5901    0.0132];
            wp.Inspect53 = [0.7 -1.93 -0.9578    0.5536   -0.9503    0.3142    0.2471   -0.3178    0.2734   -1.1246];
            wp.Inspect54 = [0.8 -1.58 -0.6333    0.1132   -0.5328    0.4594    0.0009    0.3459   -0.0193   -0.8411];
            
            wp.Inspect61 = [0 0  0 0 0 0 0 0 0 0];
            wp.Inspect62 = [0 0  0 0 0 0 0 0 0 0];
            wp.Inspect63 = [0 0  0 0 0 0 0 0 0 0];
            wp.Inspect64 = [0 0  0 0 0 0 0 0 0 0];

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

        %% Inspection Sequence 1
        function lookInspect51(this)
            this.arm.moveJoint(this.waypoints.Inspect51);
        end
        
        function lookInspect52(this)
            this.arm.moveJoint(this.waypoints.Inspect52);
        end
        
        function lookInspect53(this)
            this.arm.moveJoint(this.waypoints.Inspect53);
        end

        function lookInspect54(this)
            this.arm.moveJoint(this.waypoints.Inspect54);
        end

        %% Inspection Sequence 2
        function lookInspect61(this)
            this.arm.moveJoint(this.waypoints.Inspect61);
        end
        
        function lookInspect62(this)
            this.arm.moveJoint(this.waypoints.Inspect62);
        end
        
        function lookInspect63(this)
            this.arm.moveJoint(this.waypoints.Inspect63);
        end

        function lookInspect64(this)
            this.arm.moveJoint(this.waypoints.Inspect64);
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