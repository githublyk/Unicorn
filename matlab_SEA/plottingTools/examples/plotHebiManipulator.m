% To do:
% for arm link of X-module measure out length of each clamp and compensate 
% for them in the stl patching so that the when user enters length of 
%'ext' the right value pops up.



function plotHebiManipulator()
%Plots a simple HEBI linkage with elbow modules and strait modules
    close all;
    
    links = {{'X5-9'},...
            {'GenericLink',...
                'com',[.01,.01,.01]','out',...
                ((rotx(pi()/2)+ [zeros(4,3),[0 .025 .055 0]'])*rotz(pi/2)),...
                'mass',.1},...
            {'X5-9'},...
            {'X5Link','ext',.28,'twist',pi},...
            {'X5-9'},...
            {'X5Link','ext',.285,'twist',pi},...
            {'X5-9'},...;
            {'X5WristBracket',...
                'com',[.01,.01,.01]','out',...
                (roty(pi()/2)+ [zeros(4,3),[.043 0 .04 0]'])*rotz(0),...
                'mass',.1},...
            {'X5-9'}};

      plt = HebiPlotter('JointTypes', links,'resolution','low');
    
   angle1 = 0;
   angle2 = 0;
   angle3 = 0;
   angle4 = 0;
   angle5 = 0;
   angle6 = 0;
   num_samples = 1000;
   %angles = [];

  %  for i=0::
  %     angles = [angles, linspace(-pi/2, pi/2, num_samples)'];
  %  end
    
%     for i= 0:.01:1
%         plt.plot(theta2*i);
%         pause(.01);
%     end
        
    for i= 1:100
        plt.plot([angle1,-i/90,0,0,0]);
        pause(.01);
        angle2 = -i/90;
    end
    
    for i= 1:100
        plt.plot([angle1,angle2,i/90,0,0]);
        pause(.01);
        angle3 = i/90;
    end
    for i= 1:100
        plt.plot([angle1,angle2+i/90,angle3,-i/90,0]);
        pause(.01);
        angle4 = -i/90;
        angle5 = angle2+i/90;
    end
    
    for i = 1:200
        plt.plot([angle1+i/90,angle5,angle3,angle4,0]);
        pause(0.01);
        angle6 = angle1+i/90;
    end
    
    for i = 1:50
        plt.plot([angle6,angle5-i/90,angle3,angle4,0]);
        pause(0.01);
    end
     
    

        
end
