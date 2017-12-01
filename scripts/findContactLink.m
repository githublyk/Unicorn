
% f = Fodbot();
function findContactLink(f)
    cmd = CommandStruct;

    fbk = f.fullGroup.getNextFeedback();
    errorBaseline = fbk.torque - fbk.torqueCmd;

    basePosition = [-.25, -.15, -.09, -.8, .4,...
                    -.8, -.07, -.06, -.17, -.8];
    cmd.position = basePosition;
    cmd.torque = f.fullKin.getGravCompTorques(cmd.position, ...
                                              [0 0 1]);
    % cmd.position = zeros(1,10);

    f.fullGroup.set(cmd);
    pause(1)
    contact = f.arm.isContact(1)



    n = f.fullGroup.getInfo.numModules;

    for i = n:-1:1
        for sign = [-1 1]
            pos = basePosition;
            pos(i) = pos(i) + .3 * sign;
            
            cmd.position = pos;
            cmd.torque = f.fullKin.getGravCompTorques(cmd.position, ...
                                                      [0 0 1]);
            f.fullGroup.set(cmd);
            pause(1)
            contact = f.arm.isContact(1)
        end
    end

end
