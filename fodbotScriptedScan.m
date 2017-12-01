function fodbotScriptedScan(fodbot, camera)
    if(nargin < 2)
        camera = false
    end
    
    j1 = [0.7288    0.8971   -0.4568   -0.6184   -0.0816   -0.9941];
    j2 = [0.7987   0.0510    0.5683   -0.6179   -0.0201   -0.7549];
    if(camera)
        forwardPose = [1 0 0 -.1; 0 0 -1 -.38; 0 1 0 .25; 0 0 0 1];
    else
        forwardPose = [1 0 0 -.1; 0 0 -1 -.37; 0 1 0 .2; 0 0 0 1];
    end
    
    
    % fodbot.holdXArm([0,0]);
    fodbot.arm.moveJoint([0,0,j1;], 2);
    % fodbot.holdXArm([0,0]);
    fodbot.arm.movePose(forwardPose, 2);
    fodbot.arm.movePoseRel([.3,0,0], 5);
    fodbot.arm.movePoseRel([0, 0, -.10], 3);
    fodbot.arm.movePoseRel([-.40, 0, 0], 5);


end
