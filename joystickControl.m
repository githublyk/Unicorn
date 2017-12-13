function joystickControl
startup;
    joy = vrjoystick(1);
    fodbot = Fodbot();
    
    fbk = fodbot.fullGroup.getNextFeedback();
    fodbot.arm.setAngles(    fbk.position );

    
    cartMotion = zeros(1,6);
    
    stowed = false;
    running = true;
    t = tic;
    while(running)
        [axes, buttons, povs] = read(joy);
        axes % buttons
        dt = toc(t);
        
        joyCommands = mapJoystick(axes, buttons);
        
        cartMotion = calculateMotion(joyCommands, cartMotion, dt);
        
        updateFodbot(fodbot, cartMotion);
        
        runSpecialCommands(fodbot, buttons, joy);
        
%        if(any(buttons(10)))
        if buttons(10)
            disp('ending')
            running = false;
        end
        
        t = tic;
        pause(0.01)
    end
    fodbot.arm.goLimp();
end

function joyCmd = mapJoystick(axes, buttons)
    axes = axes .* (abs(axes)>.2)
    joyCmd.x = -axes(1);
    joyCmd.y = axes(2);
    joyCmd.z = buttons(6) - buttons(5);
    joyCmd.roll = buttons(8) - buttons(7);
    joyCmd.pitch = -axes(4);
    joyCmd.yaw = axes(3);
end

function cartMotion = calculateMotion(joyCmd, cartMotion, dt)
    accelLimit = .0005;
    velLimit = .01;
    
    xyz = [joyCmd.x, joyCmd.y, joyCmd.z]*dt*.1;
    
    xyz = cartMotion(1:3) + ...
          bound(xyz-cartMotion(1:3), -accelLimit, accelLimit);
    xyz = bound(xyz, -velLimit, velLimit);
    
    roll = 0;
    pitch = joyCmd.pitch*dt*2;
    yaw = joyCmd.yaw*dt*2;
    cartMotion = [xyz, roll, pitch, yaw];
end

function updateFodbot(fodbot, dP)
    dP(5) = .5*dP(5);
    dP(6) = -.5*dP(6);
    PITCH = [cos(dP(5)) -sin(dP(5)) 0;
                 sin(dP(5)) cos(dP(5)) 0;
                 0 0 1];
    YAW = [cos(dP(6)) 0 sin(dP(6));
                 0 1 0;
                -sin(dP(6)) 0 cos(dP(6))];
    ROT = YAW*PITCH*eye(3);
    dT = [ROT, [dP(1); dP(2); dP(3)]; [0,0,0,1]];
%    dT = [eye(3), [dP(1); dP(2); dP(3)]; [0,0,0,1]];
    
    kin=  fodbot.fullKin;
    fbk = fodbot.fullGroup.getNextFeedback();
    fk = fodbot.arm.getNominalFK();
    newFK = dT *fk;
    
    % angles = kin.getIK(...
    %     'xyz', newFK(1:3,4), ...
    %     'InitialPositions', fbk.positionCmd);

    angles = kin.getIK(...
        'xyz', newFK(1:3,4), ...
        'InitialPositions', fbk.positionCmd, ...
        'so3', newFK(1:3,1:3));

%     angles(end-1) = angles(end-1) + dP(6);
%     angles(end) = angles(end) + dP(5);
    fodbot.arm.setAngles(angles);
end

function runSpecialCommands(fodbot, buttons, joy)
    if(isempty(find(buttons)))
        return
    end
    
    if(buttons(9))
        runScriptedRoutines(fodbot, buttons(1:4))
        return
    end
    
    if(buttons(7))
        switch(find(buttons, 1))
          case 1
            fodbot.lookLeftLeft;
          case 2
            fodbot.lookBackLeft;
          case 3
            fodbot.lookRightLeft;
          case 4
            fodbot.lookForwardLeft;
        end

    elseif(buttons(8))
        switch(find(buttons, 1))
          case 1
            fodbot.lookLeftRight;
          case 2
            fodbot.lookBackRight;
          case 3
            fodbot.lookRightRight;
          case 4
            fodbot.lookForwardRight;
        end
    else
        switch(find(buttons, 1))
          case 1
            fodbot.lookLeft;
          case 2
            fodbot.lookBack;
          case 3
            fodbot.lookRight;
          case 4
            fodbot.lookForward;
          case 12 
            fodbot.stow;
            waitForUnstow(joy);
            fodbot.unstow;
        end
    end
    
end

function runScriptedRoutines(fodbot, buttons)
    if(isempty(find(buttons)))
        return
    end
    
    switch(find(buttons,1))
      case 1
        fodbotScriptedScan(fodbot, true); %Has Camera
    end
end

function waitForUnstow(joy)
    [~, buttons] = read(joy);
    while(isempty([find(buttons(1:4)),find(buttons(12))]))
        [~, buttons] = read(joy);
    end
end

% function stop = checkEnd(buttons)
%     stop = false;
%     if buttons(7) && buttons(8)
%         stop = true;
%         error('Stop requested, exiting')
%     end
% end