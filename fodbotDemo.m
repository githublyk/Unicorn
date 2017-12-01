function fodbotDemo
    fodbot = Fodbot();

    fodbot.update();
    
    % fodbot.holdXArm([0,0]);
    

    j1 = [0.7288    0.8971   -0.4568   -0.6184   -0.0816   -0.9941];
    % j2 = [0.0308    0.8571    0.0861   -0.6491   -0.1988   -0.9743];
    % j3 = [-1.0386    0.0626    1.2519   -0.3142   -0.6602   -0.7299];
    % j4 = [-0.5440    1.2078    0.1359   -1.7585   -0.5249    0.2287];
    j2 = [0.7987   0.0510    0.5683   -0.6179   -0.0201   -0.7549];
    % j3 = [0.5438   -0.0282    0.8411   -0.4793    0.0370   -0.7530];
    % j4 = [-0.3164   0.0572    0.4799   -0.3882    0.5482   -0.2797];
    % j5 = [-0.2852    0.1861    1.0326   -0.2750   -0.1570   -0.2599];
    % j6 = [-1.2556    0.3240    1.5832    0.0102    0.8152   -0.2138];
    
    % j7 = [-1.5796    1.5982    0.2341    0.0362    0.3037   -1.3722];
    % j8 = [1.5252    1.3088   -0.0362   -0.0594   -0.1200   -1.1327];
    
    % p1 = [-0.12, -0.25, .23];
    % p2 = [.11, -.27, .14];
    
    forwardPose = [1 0 0 -.1; 0 0 -1 -.3; 0 1 0 .2; 0 0 0 1];
    
    % fodbot.sArm.moveJoint(j1,10);
    
    % fodbot.sArm.moveLinear(j1, 10);
    % fodbot.sArm.moveLinear(j2, 10);
    

    % fodbot.unstow()
    
    fodbotScriptedScan(fodbot, true);
    % fodbot.sArm.moveJoint([j1;], 2);
    
    % fodbot.sArm.movePose(forwardPose, 2);
    % fodbot.fullArm.movePoseRel([.3,0,0], 5);
    % fodbot.fullArm.movePoseRel([0, 0, -.10], 3);
    % fodbot.fullArm.movePoseRel([-.40, 0, 0], 5);
    
%     fodbot.sArm.moveJoint([j2;], 3);
%     disp('j2');
%     fodbot.sArm.moveJoint([j3;j4;j5;j6], 10);
%     disp('j6');
%     fodbot.sArm.moveJoint([j7], 2);
%     fodbot.sArm.moveJoint([j8], 10);
    % fodbot.stow()
end
