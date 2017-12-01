
% f = Fodbot();
cmd = CommandStruct;

errorFiltered = zeros(1,10);
t = tic;
timeC = .5;

fbk = f.fullGroup.getNextFeedback();
errorBaseline = fbk.torque - fbk.torqueCmd;

cmd.position = [-.25, -.15, -.09, -.8, .4,...
                -.8, -.07, -.06, -.17, -.8];
cmd.torque = f.fullKin.getGravCompTorques(cmd.position, ...
                                          [0 0 1]);
% cmd.position = zeros(1,10);

f.fullGroup.set(cmd);
pause(1)
contact = f.arm.isContact(1)

return

while true
    dt = toc(t);
    t = tic;
    
    alpha = timeC/(timeC+dt);
    
    fbk = f.fullGroup.getNextFeedback();
    expectedTau = f.fullKin.getGravCompTorques(fbk.position, ...
                                               [0 0 1]);
    
    cmd.torque = expectedTau;
    f.fullGroup.set(cmd);
    measuredTau = fbk.torque;
    % error =  measuredTau - expectedTau - errorBaseline;
    error =  measuredTau - expectedTau;
    % error = fbk.position - fbk.positionCmd; %
    errorFiltered = errorFiltered*alpha + (1-alpha)*error
    
    J = f.fullKin.getJacobian('EndEffector',fbk.position);
    tau = fbk.torque;
    
    % J*errorFiltered'
    

    
end