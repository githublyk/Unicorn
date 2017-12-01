close all;
clear all
addpath('HebiCam');

% snake head group
g = HebiLookup.newGroupFromNames('*', 'S_CAM_1');

%% camera window
addpath(genpath(pwd));
% javaaddpath('hebicam-1.1-all-x86_64.jar');
% Replace url with your device specific stream
% url = 'http://<ip address>/mjpg/video.mjpg?resolution=640x480';
url = 'rtsp://admin:biorobotics15213@10.10.10.117';
cam = HebiCam(url);
camfig = figure();
img = getsnapshot(cam);
% low pass filter on rot angle
alphaFilter0 = .1;
alphaFilter = alphaFilter0;

%% feedback
try
    fbk = g.getNextFeedback;
catch err
    disp(err.message);
end

%% USE fbk to get a new rotImageAngle %%%
gravity = -[fbk.accelX; fbk.accelY; fbk.accelZ];
%  v = gravity - dot(gravity, [0;0;1]);
rotImageAngle = atan2(gravity(1), -gravity(2));
imageWidth = max(size(img));

fig = imshow(zeros(imageWidth, imageWidth, 3)); % this ensures the figure is big enough to take all image sizes
set(fig, 'CData', imrotate(img, rad2deg(rotImageAngle)) );


% snake head group
g = HebiLookup.newGroupFromNames('*', 'S_CAM_1'); % this needs to be started after the camera, it seems.

tic; tocLast = 0;

while true
    
    %% feedback
    try
        fbk = g.getNextFeedback;
    catch err
        disp(err.message);
    end
    
    %%USE fbk to get a new rotImageAngle %%%
    gravity = -[fbk.accelX; fbk.accelY; fbk.accelZ];
    %  v = gravity - dot(gravity, [0;0;1]);
    rotImageAngleNow = atan2(gravity(1), -gravity(2));
    
   %% PROBLEM: When the image angle goes from around the boundary pi to -pi or back, 
   % then the filter fails. It should be a filter on a S1 element, not a R1
   % element!
    
    %% scale filter by how big close we are to straight up
    alphaFilter  = alphaFilter0 *(10 - abs(gravity(3)))/10;
    if abs(gravity(3))<8 % HACK: don't bother updating rot angle when camera is up or down pointing.
    rotImageAngle = alphaFilter*rotImageAngleNow +(1-alphaFilter)*rotImageAngle; % filter with some delay
    end   

    %% camera
    try
        img = getsnapshot(cam);
    catch err
        disp([err.message ' Restarting cam...'] );
        %         disp('skip img');
        cam = HebiCam(url);
        
    end
    
    set(fig, 'CData', imrotate(img, rad2deg(rotImageAngle)) );
    drawnow;
    
    
    tocNow =toc;
    disp(['Time: ' num2str(tocNow - tocLast), ...
        ' AngleNow: ' num2str(rotImageAngleNow), ...
        ' AngleFiltered: ' num2str(rotImageAngle)])
    tocLast = tocNow;
end
