classdef RosFodbot < Fodbot
%Expansion of the Fodbot class to add some listeners and publishers
%to allow communication with ros
    properties
        timerPublishTF;
        tftree;
        tfStampedMsg;
        commandSub;
        commandBufSub;
        cmdBufLength = 5;
        cmdBuf;
    end
    
    methods
        function this = RosFodbot()
            this@Fodbot();

            disp('DELETING ALL TIMERS TO SHUT DOWN PREV. OBJECTS')
            delete(timerfind);

                        
            ROS_IP = '10.10.10.37';
            ROS_MASTER_URI = 'http://10.10.11.216:11311';
            % ROS_MASTER_URI = 'http://localhost:11311';
            
            setenv('ROS_IP', ROS_IP);
            setenv('ROS_MASTER_URI', ROS_MASTER_URI);

            rosinit
                        
            this.tftree = rostf;
            this.tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');

            this.commandSub = rossubscriber('fodbot/command', ...
                                            'geometry_msgs/Pose',...
                                            @this.moveToRosPose);
            this.commandBufSub = rossubscriber('fodbot/command_buf', ...
                                               'geometry_msgs/Pose',...
                                               @this.moveToRosPoseBuf);
            this.cmdBuf = cell(1,this.cmdBufLength);
            
            this.timerPublishTF = timer;
            this.timerPublishTF.TimerFcn = @(~,~) this.publishTF();
            this.timerPublishTF.Period = 0.05;
            this.timerPublishTF.ExecutionMode = 'fixedRate';
            start(this.timerPublishTF);
        end
        
        
        function restartTimer(this)
        %The timer sometimes dies. This give a way to restart it.
            stop(this.timerPublishTF);
            start(this.timerPublishTF);
        end
        
        function delete(this)
            disp('deleting timer');
            % this.timerPublishTF.TimerFcn = '';
            % this.stopTimer();
            delete(this.timerPublishTF);
            this.timerPublishTF = [];
            disp('deleted timer')
            
            rosshutdown

            disp('delete successful')

        end
        
        function publishTF(this)
            fk = this.fullArm.getFK();
            this.setTfMsg(fk);
            this.tfStampedMsg.ChildFrameId = 'fodbot_base';
            this.tfStampedMsg.Header.FrameId = 'fodbot_ee';
            sendTransform(this.tftree, this.tfStampedMsg);
            
            blaser = eye(4);
            blaser(1:3,4) = [0; -0.03; 0.03];
            blaser = blaser*rotz(3.1415);
            this.setTfMsg(fk*blaser);
            this.tfStampedMsg.Header.FrameId = 'blaser';
            sendTransform(this.tftree, this.tfStampedMsg);            
        end

    end
    
    methods(Access = private, Hidden = true)
        function moveToRosPose(this, src, poseMsg)
            % poseMsg
            pose = eye(4);
            pose(1,4) = poseMsg.Position.X;
            pose(2,4) = poseMsg.Position.Y;
            pose(3,4) = poseMsg.Position.Z;
            quatrot(1) = poseMsg.Orientation.W;
            quatrot(2) = poseMsg.Orientation.X;
            quatrot(3) = poseMsg.Orientation.Y;
            quatrot(4) = poseMsg.Orientation.Z;

            pose(1:3,1:3) = quat2rotm(quatrot);
            
            this.fullArm.movePose(pose,3);

        end
        
        function moveToRosPoseBuf(this,src,poseMsg)
        %Implement the buffer, to delay motion when performing
        %blaser scan
        %This is a poor man's queue since queue does
        %not exist in matlab
            if(~isempty(this.cmdBuf{1}))
                this.moveToRosPose(src, this.cmdBuf{1});
            end
            l = this.cmdBufLength;
            this.cmdBuf(1:l-1) = this.cmdBuf(2:l);
            this.cmdBuf{l} = poseMsg;
        end
        
        
        function setTfMsg(this, fk)
            this.tfStampedMsg.Transform.Translation.X = fk(1,4);
            this.tfStampedMsg.Transform.Translation.Y = fk(2,4);
            this.tfStampedMsg.Transform.Translation.Z = fk(3,4);

            quatrot = rotm2quat(fk(1:3,1:3));
            this.tfStampedMsg.Transform.Rotation.W = quatrot(1);
            this.tfStampedMsg.Transform.Rotation.X = quatrot(2);
            this.tfStampedMsg.Transform.Rotation.Y = quatrot(3);
            this.tfStampedMsg.Transform.Rotation.Z = quatrot(4);
            
            this.tfStampedMsg.Header.Stamp = rostime('now');
        end
    end
end