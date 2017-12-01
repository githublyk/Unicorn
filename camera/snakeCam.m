close all;
clear all
addpath('HebiCam');

% snake head group
% g = HebiLookup.newGroupFromNames('*', 'S_CAM_1');

%% camera window
addpath(genpath(pwd));
% javaaddpath('hebicam-1.1-all-x86_64.jar');
% Replace url with your device specific stream
% url = 'http://<ip address>/mjpg/video.mjpg?resolution=640x480';
url = 'rtsp://admin:biorobotics15213@10.10.10.117';
cam = HebiCam(url);
camfig = figure();
        img = getsnapshot(cam);
rotImageAngle = 0;
fig = imshow( imrotate(img, rad2deg(rotImageAngle)));


tic; tocLast = 0;

while true
    
%     %% feedback
%     try
%         fbk = g.getNextFeedback;
%     catch err
%         disp(err.message);
%     end
   
%% TO DO: USE fbk to get a new rotImageAngle %%%
    
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
    disp(tocNow - tocLast)
    tocLast = tocNow;
end
