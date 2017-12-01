function workSpace

resolution = 0.05;
    
TransalationTol = 0.01;
angleTol = 0.01;
temp  =  FodbotOffline;
fod = temp.fullKin;
plotX = [];
plotY = [];
plotZ = [];

% rotationMatrix = eye(3);
rotationMatrix = rotx(pi/2);
%rotationMatrix(1) = 1;

rotationMatrix = rotationMatrix(1:3,1:3);

prevValid = false;
found = 45;            

for x = -0.5:resolution:0.5
    for y = -0.7:resolution:0
        for z = -0.5:resolution:0.5

            if(prevValid)
                %Seed IK with previous angles, if previous solution valid
                ang = fod.getInverseKinematics('xyz', [x,y,z], ...
                                               'so3', rotationMatrix, ...
                                               'initial', ang);
            else
                ang = fod.getInverseKinematics('xyz', [x,y,z], 'so3', rotationMatrix);
            end
            
            pos = fod.getForwardKinematics('EndEffector',ang);
            tempX = pos(1,4);
            tempY = pos(2,4);
            tempZ = pos(3,4);
            
            newRotMat = pos(1:3, 1:3) ;
            rotAngle = angleBetween(newRotMat,rotationMatrix);
            isTransalation = ((abs(x-tempX)<=TransalationTol) && ...
                              (abs(y-tempY)<=TransalationTol) && ...
                              (abs(z-tempZ)<=TransalationTol));
            isRotation = (abs(rotAngle)<=angleTol);
            
            prevValid = false;
            if (isTransalation && isRotation)
                %newRotMat, rotAngle
                prevValid = true;
                
                if (found >=0)
                    found = found-1;
                    plotAng = ang;
                    XYZ = [x,y,z];
                end
                plotX = [plotX;tempX];
                plotY = [plotY;tempY];
                plotZ = [plotZ;tempZ];
                
            end
                
        end
    end
end

close all;
p = HebiPlotter;
plotAng =  fod.getInverseKinematics( 'xyz', XYZ, 'so3', rotationMatrix);

Xangles = plotAng(1:2);
xEndFrame = temp.xKin.getFK('EndEffector',Xangles);
p.setBaseFrame(xEndFrame);


p.plot(plotAng(3:8));
hold on;
% [sx, sy, sz] = sphere;
% r = 0.05
% for i=1:size(plotX,1)
%     surf(r*sx+plotX(i), r*sy + plotY(i), r*sz + plotZ(i));
% end
scatter3(plotX, plotY, plotZ);
fod.getFK('EndEffector',plotAng)

end