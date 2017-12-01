function [offsets, start] = calcWaveJoints(J, disp)

    numLinks = size(J, 3);
    numJoints = size(J, 2);

    J_tmp = J(1:3,:,:);

    
    J = zeros(3*size(J,3), size(J,2));

    for i=1:numLinks
        ind = (3*i - 2):(3*i);
        J(ind,:) = J_tmp(1:3, :, i);
    end


    offsets = zeros(numLinks, numJoints);
    for linkNum = numLinks:-1:1

        ind = (linkNum*3-2):(linkNum*3); 
        d = zeros(numLinks*3,1);
        d(ind) = disp;
        
        options = optimoptions('lsqlin','Algorithm','interior-point',...
                               'Display', 'off');
        
        angles = lsqlin(J, d, [], [], J(ind,:), d(ind), [], [], [], ...
                       options);
        
        if(isempty(angles))
            start = linkNum;
            break;
        end
        offsets(linkNum, :) = angles';
    end

    % J*angle
    
    % offset = [0;0;angle]';
    % J_new(:,:,linkNum)
end
