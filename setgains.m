% SET GAINS
gains = GainStruct()

% Initialize X Module Gains
ones_n                          = ones(1,2);
XcontrolStrategy           = ones_n*4;
XpositionKp                = ones_n*4;
XpositionKi                = ones_n*.01;
XpositionKd                = ones_n*1;
% gains.torqueKp                  = ones_n*1;
% gains.torqueKi                  = ones_n*0;
% gains.torqueKd                  = ones_n*.1;
% gains.torqueMaxOutput           = ones_n*2.25;
% gains.torqueMinOutput           = -gains.torqueMaxOutput;
XpositionIClamp            = ones_n*1;
XvelocityKp                = ones_n*1;
XpositionMaxOutput         = ones_n*10;
XpositionMinOutput         = ones_n*-10;
% gains.torqueOutputLowpassGain   = ones_n*.5;
% gains.torqueFF                  = ones_n*0.15;

% Initialize S Module Gains
ones_n                          = ones(1,sGroup.getNumModules);
ScontrolStrategy           = ones_n*4;
SpositionKp                = ones_n*10
SpositionKi                = ones_n*.01;
SpositionKd                = ones_n*1;
% gains.torqueKp                  = ones_n*1;
% gains.torqueKi                  = ones_n*0;
% gains.torqueKd                  = ones_n*.1;
% gains.torqueMaxOutput           = ones_n*2.25;
% gains.torqueMinOutput           = -gains.torqueMaxOutput;
SpositionIClamp            = ones_n*1;
SvelocityKp                = ones_n*1;
SpositionMaxOutput         = ones_n*10;
SpositionMinOutput         = ones_n*-10;
% gains.torqueOutputLowpassGain   = ones_n*.5;
% gains.torqueFF                  = ones_n*0.15;

% ALL GAINS
gains.controlStrategy           = [XcontrolStrategy ScontrolStrategy];
gains.positionKp                = [XpositionKp SpositionKp];
gains.positionKi                = [XpositionKi SpositionKi];
gains.positionKd                = [XpositionKd SpositionKd];
% gains.torqueKp                  = ones_n*1;
% gains.torqueKi                  = ones_n*0;
% gains.torqueKd                  = ones_n*.1;
% gains.torqueMaxOutput           = ones_n*2.25;
% gains.torqueMinOutput           = -gains.torqueMaxOutput;
gains.positionIClamp            = [XpositionIClamp SpositionIClamp];
gains.velocityKp                = [XvelocityKp SvelocityKp];
gains.positionMaxOutput         = [XpositionMaxOutput SpositionMaxOutput];
gains.positionMinOutput         = [XpositionMinOutput SpositionMinOutput];
% gains.torqueOutputLowpassGain   = ones_n*.5;
% gains.torqueFF                  = ones_n*0.15;

this.fullGroup.send('gains', gains);
%this.fullKin.set('gains', gains);
