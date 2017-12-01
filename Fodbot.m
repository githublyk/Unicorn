
classdef Fodbot < FodbotOffline
    
    methods(Access = public)
        function this = Fodbot()
%             sGroup = HebiLookup.newGroupFromNames('*', ...
            sGroup = HebiLookup.newConnectedGroupFromName('*','SA065');
%           {'SA065', 'SA068', 'SA069', ...
%            'SA067', 'SA064'
           % 'SA067', ...
            % 'SA068', 'SA069',...
            % 'SA065', 'SA076'...
%                    });

            this@FodbotOffline(2 + sGroup.getNumModules);
            
            this.xGroup = HebiLookup.newGroupFromNames('*', ...
                               {'X-00036', ...
                                'X-00031'});
            this.sGroup = sGroup;
            
            this.fullGroup = HebiLookup.newGroupFromNames('*',...
                [this.xGroup.getInfo.name;
                 this.sGroup.getInfo.name]);
            
            
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
            
            wp.Forward = [.5 0 1.1 .7 0 .5 -.5 -.2 z];
            wp.Right = [-.5 0 .5 .5 0 .5 .5 0 z];
            wp.Left = [.5 0 -.5 .5 0 .5 -.5 0 z];
            wp.stowed = [0 0 -1.54 1.4 1.5 -1.73 -0.21 1.53 z];
            wp.stowedMid = [0 0 -0.95 0.98 1.28 -0.80 -0.70 0.13 z];
            
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
        
        function lookForward(this)
            this.waypoints.Forward
            this.arm.moveJoint(this.waypoints.Forward);
        end
        
        function lookRight(this)
            this.arm.moveJoint(this.waypoints.Right);
        end
        
        function lookLeft(this)
            this.arm.moveJoint(this.waypoints.Left);
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