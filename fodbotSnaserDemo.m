% cam = HebiCam(['rtsp://192.168.1.20:554/user=admin&password=&' ...
%                'channel=1&stream=0.sdp?'])
% f = Fodbot()



figure()

while true
    
    points = getSnaserFbk(cam.getsnapshot)
    fk = fodbot.getFK();
    points = [points, ones(size(points,1),1)];
    points = (fk * points')';
    
    hold on
    scatter3(points(:,1), points(:,2), points(:,3), '.');
end


