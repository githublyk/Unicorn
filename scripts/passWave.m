function passWave(f, offset)
%Passes a wave down the fodbot
    dir = offset/norm(offset);
    cmd = CommandStruct;

    fbk = f.fullGroup.getNextFeedback();
    errorBaseline = fbk.torque - fbk.torqueCmd;

    % basePosition = [-.25, -.15, -.09, -1.1, .4,...
    %                 -1.2, -.07, -.06, -.17, -.8, 0, 0];
    % basePosition = [-.25, -.15, -.09, -.7, .4,...
    %                 -0, -.07, 0.6, -.17, -.8, 0, 0];
    % basePosition = [-.25, -.15, -.09, -.5, .4,...
    %                 -0, -.07, 0.1, -.17, -.8, 0, 0];
    basePosition = [-.25, -.15, -.09, -.3, .4,...
                    -0, -.07, -0.3, -.17, 0, 0, 0.5];
    % basePosition = zeros(1,12);
    cmd.position = basePosition;
    cmd.torque = f.fullKin.getGravCompTorques(cmd.position, ...
                                              [0 0 1]);
    % cmd.position = zeros(1,10);

    f.fullGroup.set(cmd);
    pause(1)
    contact = f.arm.isContact(1)

    n = f.fullGroup.getInfo.numModules;
    
    % t = tic;
    J = f.fullKin.getJacobian('CoM', cmd.position);
    [thetaOffsets, start] = calcWaveJoints(J(:,3:end,:), offset);
    
    thetaOffsets = [thetaOffsets; zeros(1,size(thetaOffsets,2))];
    
    % numOffsets = size(thetaOffsets,1) - start + 1;
    numBodies = f.fullKin.getNumBodies;
    
    actualDisp = [];
    cmdDisp = [];
    timeLog = [];
    
    t = tic;
    body = start;
    tic
    
    tauMax = 0;
    baseTau = 0;% f.arm.getTorqueError;
    collisionBody = 0;
    
    bodyLog = [];
    tauLog = [];
    
    while body < numBodies + 1
        % toc
        % tic
        o = interp1([1:numBodies+1], thetaOffsets, body);
        cmd.position = basePosition + [0,0,o];
        f.fullGroup.set(cmd)
        
        % fbk = f.fullGroup.getNextFeedback();
        % fkAct = f.fullKin.getFK('CoM', fbk.position);
        % fkCmd = f.fullKin.getFK('CoM', fbk.positionCmd);
        
        % for i=1:numBodies
        %     actDispTmp(i) = fkAct(1:3,4,i)' * dir;
        %     cmdDispTmp(i) = fkCmd(1:3,4,i)' * dir;
        % end
        % actDispTmp(10)
        % fkAct(1:3,4,10)


        tauError = max(abs(f.arm.getTorqueError() - baseTau));
        
        bodyLog = [bodyLog, body];
        tauLog = [tauLog, tauError];
        fprintf('body %5.1f, \t tau %5.2f\n', body, tauError);
        if(tauError > tauMax)
            collisionBody = body;
            tauMax = tauError;
        end
        
        
        % actualDisp = [actualDisp; actDispTmp];
        % cmdDisp = [cmdDisp; cmdDispTmp];
        % timeLog = [timeLog, toc(t)];
        
        body = 4*toc(t) + start;
        pause(.01);
    end
    
    collisionBody
    plot(bodyLog, tauLog)
    
    % cmdDisp = cmdDisp - repmat(cmdDisp(1,:), size(cmdDisp,1), 1);
    % actualDisp = actualDisp - ...
    %     repmat(actualDisp(1,:), size(actualDisp,1), 1);
    
    % size(timeLog)
    % max(actualDisp) - max(cmdDisp)
    % max(actualDisp) - max(cmdDisp) < -.3*norm(offset)
    
    
    % for i=1:size(thetaOffsets,1)
    %     cmd.position = basePosition + [0,0,thetaOffsets(i,:)];
    %     f.fullGroup.set(cmd);
    %     pause(.5)
    % end
end
