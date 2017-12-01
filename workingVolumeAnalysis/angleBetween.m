function angle =  angleBetween(rotationMatrix1,rotationMatrix2)
%Returns the magnitude of angle of rotation between two rotation matricies
    newRotMat = rotationMatrix1*inv(rotationMatrix2);
    newRotMat = newRotMat(1:3,1:3); %Extract pure rotation matrix, in
                                   %case large matrix was added

    % Calculation of axis of rotation: Not needed
    % h = newRotMat(3,2);
    % f = newRotMat(2,3);
    % c = newRotMat(1,3);
    % g = newRotMat(3,1);
    % d = newRotMat(2,1);
    % b = newRotMat(1,2);
    
    % magnitude = ( ((h-f)^2) + ((c-g)^2) + ((d-b)^2 ))^0.5;
    % angle = asin(magnitude/2);



    %from wikipedia: https://en.wikipedia.org/wiki/Rotation_matrix#Determining_the_angle
    %Tr(R) = 1+2cos(theta)
    angle = acos((trace(newRotMat) - 1)/2);

end
