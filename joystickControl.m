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
        
        if(any(buttons(9:12)))
            disp('ending')
            running = false;
        end
        
        t = tic;
        pause(0.01)
    end
    fodbot.arm.goLimp();
end

function joyCmd = mapJoystick(axes, buttons)
    axes = axes .* (abs(axes)>.2);
    joyCmd.x = -axes(1);
    joyCmd.y = axes(2);
    joyCmd.z = buttons(6) - buttons(5);
    joyCmd.roll = buttons(8) - buttons(7);
    joyCmd.pitch = -axes(4);
    joyCmd.yaw = axes(3);
    joyCmd.mode = buttons(7);
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
    mode = joyCmd.mode;
    cartMotion = [xyz, roll, pitch, yaw, mode];
end


function updateFodbot(fodbot, dP)
    dP(5) = .5*dP(5);
    dP(6) = -.5*dP(6);
  
    thetap = dP(6)*(1-dP(7));
    thetay = dP(5)*(1-dP(7));
    PITCH = [cos(thetap) -sin(thetap) 0;
                 sin(thetap) cos(thetap) 0;
                 0 0 1];
    YAW = [cos(thetay) 0 sin(thetay);
                 0 1 0;
                -sin(thetay) 0 cos(thetay)];
    ROT = YAW*PITCH*eye(3);
%    dT = [ROT, [0; 0; 0]; [0,0,0,1]];
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
      frames = kin.getForwardKinematics('output', fbk.position);
      f = frames(1:3,1:3,end-1);
%       for i = 1:size(angles)-1
%           f = frames(1:3,1:3,i)*f;
%       end
%     angles(end-1) = angles(end-1) + (1-dP(7))*dP(6);
%     angles(end) = angles(end) + (1-dP(7))*dP(5);
    
    alpha = atan(f(1,1)/f(2,1))*180/pi;
    beta = atan(sqrt(f(3,2)^2 + f(3,3)^2)/(-f(3,1)))*180/pi;
    gamma = atan(f(3,2)/f(3,3))*180/pi;
    R =[alpha beta gamma]
    alpha = alpha*pi/180;
    beta = beta*pi/180;
    gamma = gamma*pi/180;
    
     if abs(beta)>0 && abs(beta) <=pi/4
        angles(end-1) = angles(end-1) - dP(7)*dP(6);
        angles(end-3) = angles(end-3) + dP(7)*dP(6);
        angles(end) = angles(end) + dP(7)*dP(5);
        angles(end-2) = angles(end-2) - dP(7)*dP(5);

      end
    if abs(beta)>pi/4 && abs(beta) <=pi/2
        angles(end-1) = angles(end-1) - dP(7)*dP(5);
        angles(end-3) = angles(end-3) + dP(7)*dP(5);
        angles(end) = angles(end) + dP(7)*dP(6);
        angles(end-2) = angles(end-2) - dP(7)*dP(6);
    end
    
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

% function stop = checkEnd(buttons)
%     stop = false;
%     if buttons(7) && buttons(8)
%         stop = true;
%         error('Stop requested, exiting')
%     end
% end