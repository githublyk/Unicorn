function holdFodbotPosition(fodbot, duration, varargin)
    
    
    p = inputParser();
    p.addParameter('numControllableModules', 6);
    p.addParameter('display', 'off');
    
    p.parse(varargin{:});
    re = p.Results;
    
    c = 6;
    
    g = fodbot.fullGroup;

    n = 8;
    kin = fodbot.fullKin;
    % firstKin = fodbot.xKin;
    lastKin = fodbot.sKin;
    
    goal = fodbot.getFK();

    fbk = g.getNextFeedback;
    startAngles = fbk.position;

    % lastKin.setBaseFrame(firstKin.getFK('EndEffector', ...
    %                                     fbk.position(1:n-c)));
    fodbot.update();
    lastKin.getFK('EndEffector', fbk.position(n-c+1:n)) - goal;

    cmd = CommandStruct();
    cmd.velocity = nan * zeros(1,n);
    cmd.position = nan * zeros(1,n);
    cmd.position(n-c+1:n) = fbk.position(n-c+1:n);


    tic

    while toc < duration
        
        fbk = g.getNextFeedback(fbk);
        angles = fbk.position;
        if(isempty(fbk))
            disp('Feedback Missed')
            continue
        end
        startInd = 1:n-c;
        endInd = n-c+1:n;
        
        fodbot.update();
        % lastKin.setBaseFrame(firstKin.getFK('EndEffector', ...
        %                                     fbk.position(1:n-c)));
        
        % cmd.position(startInd) = startAngles(startInd) + ...
        %     (angles(startInd) - startAngles(startInd)) * .9;

        cmd.position = angles;
        cmd.position(endInd) = ...
            lastKin.getIK('xyz', goal(1:3,4), 'axis', goal(1:3,1:3)*[0;0;1], ...
                                 'InitialPositions', ...
                                 angles(endInd));

        % cmd.position = kin.getIK('xyz', goal(1:3,4), 'axis', goal(1:3,1:3)*[0;0;1], ...
        %                                 'InitialPositions', ...
        %                                 fbk.position);
        
        % cmd.position = cmd.position + (startAngles - cmd.position)*.9;
        
        J = kin.getJacobian('EndEffector', angles);
        error = startAngles - angles;
        % scalingVector = [1 1, 0 0 1 0 0 1];
        % error = error .* scalingVector;
        restoring = calculateRestoringVector(null(J), error')';
        
        magError = norm(error);
        disp(error)
        magError
        magCorrection = min(magError*.1, .1);
        
        cmd.position = cmd.position + restoring * magCorrection;
        
        cmd.torque = kin.getGravCompTorques(fbk.position, [0 0 1]);
        
        
        if(strcmpi(re.display, 'on'))
            fk = lastKin.getFK('EndEffector',fbk.position(endInd));
            positionErr = fk(1:3,4) - goal(1:3,4)
        end

        g.set(cmd);
        pause(0.01);
    end
end

function v = calculateRestoringVector(space, restoringDirection)
    restoringDirection = restoringDirection / norm(restoringDirection);
    v = zeros(size(restoringDirection));
    % restoringDirection
    % space
    for spaceInd = 1:size(space,2)
        space(:,spaceInd) = space(:,spaceInd)/norm(space(:, spaceInd));
        v = v + dot(restoringDirection, space(:,spaceInd)) * space(:,spaceInd);
    end
    v = v /norm(v);
end