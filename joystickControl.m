function joystickControl
    joy = vrjoystick(1);
    fodbot = Fodbot();
    
    
    cartMotion = zeros(1,6);
    
    stowed = false;
    
    t = tic;
    while(true)
        [axes, buttons, povs] = read(joy);
        % buttons
        dt = toc(t);
        
        joyCommands = mapJoystick(axes, buttons);
        
        cartMotion = calculateMotion(joyCommands, cartMotion, dt);
        
        updateFodbot(fodbot, cartMotion);
        
        runSpecialCommands(fodbot, buttons, joy);
        
        if(checkEnd(buttons))
            disp('ending')
            break
        end
        
        t = tic;
        pause(0.01)
    end
end

function joyCmd = mapJoystick(axes, buttons)
    axes = axes .* (abs(axes)>.2);
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
    
    dT = [eye(3), [dP(1); dP(2); dP(3)]; [0,0,0,1]];
    
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

    angles(end-1) = angles(end-1) + dP(6);
    angles(end) = angles(end) + dP(5);
    angles;
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

    switch(find(buttons, 1))
      case 1
        fodbot.lookLeft;
      case 2
        fodbot.stow;
        waitForUnstow(joy);
        fodbot.unstow;
      case 3
        fodbot.lookRight;
      case 4
        fodbot.lookForward;
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
    while(isempty(find(buttons(1:4))))
        [~, buttons] = read(joy);
    end
end

function stop = checkEnd(buttons)
    stop = false;
    if buttons(7) && buttons(8)
        stop = true;
        error('Stop requested, exiting')
    end
end