close all;
clear all
addpath('HebiCam');



%% camera window
addpath(genpath(pwd));
% Replace url with your device specific stream
% url = 'http://<ip address>/mjpg/video.mjpg?resolution=640x480';
url = 'rtsp://admin:biorobotics15213@10.10.10.117';
cam = HebiCam(url);
camfig = figure();
        img = getsnapshot(cam);
fig = imshow( imrotate(img, rad2deg(rotImageAngle)));


tic; tocLast = 0;

while true
    
 
    %% camera
    try
        img = getsnapshot(cam);
    catch err
        disp([err.message ' Restarting cam...'] );
        %         disp('skip img');
        cam = HebiCam(url);
        
    end
    
    set(fig, 'CData', img) ;
    drawnow;
    
    
    tocNow =toc;
    disp(['Time: ' num2str(tocNow - tocLast)])
    tocLast = tocNow;
end
