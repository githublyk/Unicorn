function rosTesting
% rosinit
    % setenv
    setenv('ROS_IP','10.10.10.37');
    setenv('ROS_MASTER_URI', 'http://10.10.11.216:11311')
    rosinit('10.10.11.216')
    getenv('ROS_IP')

    cleanupObj = onCleanup(@destructor); 
    
    
    pause(2);
    
    pCloudSub = rossubscriber('/camera_1/PointCloud', @displayPointCloud)
    % pCloudSub.NewMessageFcn = {@displayPointCloud}
    % pCloudSub.setOnNewMessageListeners({@displayPointCloud});
    while(true)
        % receive(pCloudSub)
        pause(0.1)
    end    
    pause(3);
    tftree.AvailableFrames
    
end

function displayPointCloud(src,pCloud)
    disp('called')
    pCloud
end

function destructor()
    rosshutdown
end
