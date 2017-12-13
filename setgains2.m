% SET GAINS
gains = GainStruct();

% Initialize X Module Gains
ones_n                          = ones(1,2);
XcontrolStrategy           = ones_n*4;

XpositionKp                           = [10 15];
XpositionKi                            = ones_n*0;
XpositionKd                           = ones_n*0;
XpositionFF                           = ones_n*0;
XpositionDeadZone                = ones_n*0;
XpositionIClamp                     = ones_n*1;
XpositionPunch                      = ones_n*0;
XpositionMinTarget                = -[1.57 inf];
XpositionMaxTarget               = [1.57 inf];
XpositionTargetLowpassGain  = ones_n*1;
XpositionMinOutput               = -[10 20];
XpositionMaxOutput              = [10 20];
XpositionOutputLowpassGain  = ones_n*1;
XpositionDOnError                    = ones_n*1;

XvelocityKp                           = ones_n*0.05;
XvelocityKi                            = ones_n*0;
XvelocityKd                           = ones_n*0;
XvelocityFF                           = ones_n*1;
XvelocityDeadZone                = ones_n*0;
XvelocityIClamp                     = ones_n*0.25;
XvelocityPunch                      = ones_n*0;
XvelocityMinTarget                = -[3.434687 1.502675];
XvelocityMaxTarget               = [3.434687 1.502675];
XvelocityTargetLowpassGain  = ones_n*1;
XvelocityMinOutput               = ones_n*-1;
XvelocityMaxOutput              = ones_n*1;
XvelocityOutputLowpassGain  = ones_n*0.75;
XvelocityDOnError                  = ones_n*1;

XeffortKp                           = ones_n*0.25;
XeffortKi                            = ones_n*0;
XeffortKd                           = ones_n*0.001;
XeffortFF                           = ones_n*1;
XeffortDeadZone                = ones_n*0;
XeffortIClamp                     = ones_n*0.25;
XeffortPunch                      = ones_n*0;
XeffortMinTarget                = -[10 20];
XeffortMaxTarget               = [10 20];
XeffortTargetLowpassGain  = ones_n*1;
XeffortMinOutput               = -[4 9]; %ones_n*-1;
XeffortMaxOutput              = [4 9]; %ones_n*1;
XeffortOutputLowpassGain  = ones_n*0.9;
XeffortDOnError                  = ones_n*0;

% Initialize S Module Gains
ones_n                        = ones(1,sGroup.getNumModules);
ScontrolStrategy           = ones_n*4;

SpositionKp                           = ones_n*12;
SpositionKi                            = ones_n*.01;%*1000;
SpositionKd                           = (ones_n(1:end))*4;%*.001;
SpositionFF                           = ones_n*0;
SpositionDeadZone                = ones_n*0;
SpositionIClamp                     = ones_n*1;
SpositionPunch                      = ones_n*0;
SpositionMinTarget                = ones_n*-1.77;
SpositionMaxTarget               = ones_n*1.77;
SpositionTargetLowpassGain  = ones_n*1;
SpositionMinOutput               = ones_n*-10;
SpositionMaxOutput              = ones_n*10;
SpositionOutputLowpassGain  = ones_n*1;
SpositionDOnError                    = ones_n*1;

SvelocityKp                           = ones_n*1;
SvelocityKi                            = ones_n*0*1000;
SvelocityKd                           = ones_n*0*.001;
SvelocityFF                           = ones_n*0;
SvelocityDeadZone                = ones_n*0.01;
SvelocityIClamp                     = ones_n*1;
SvelocityPunch                      = ones_n*0;
SvelocityMinTarget                = ones_n*-4;
SvelocityMaxTarget               = ones_n*4;
SvelocityTargetLowpassGain  = ones_n*1;
SvelocityMinOutput               = ones_n*-12;
SvelocityMaxOutput              = ones_n*12;
SvelocityOutputLowpassGain  = ones_n*1;
SvelocityDOnError                  = ones_n*1;

SeffortKp                           = ones_n*.75;
SeffortKi                            = ones_n*0*1000;
SeffortKd                           = ones_n*1;%*.001;
SeffortFF                           = ones_n*.15;
SeffortDeadZone                = ones_n*.01;
SeffortIClamp                     = ones_n*0;
SeffortPunch                      = ones_n*0;
SeffortMinTarget                = ones_n*-12;
SeffortMaxTarget               = ones_n*12;
SeffortTargetLowpassGain  = ones_n*1;
SeffortMinOutput               = ones_n*-5;
SeffortMaxOutput              = ones_n*5;
SeffortOutputLowpassGain  = ones_n*.15;
SeffortDOnError                  = ones_n*0;

% ALL GAINS
gains.controlStrategy       = [XcontrolStrategy ScontrolStrategy];

gains.positionKp             = [XpositionKp SpositionKp];
gains.positionKi              = [XpositionKi SpositionKi];
gains.positionKd             = [XpositionKd SpositionKd];
gains.positionFF              = [XpositionFF SpositionFF];
gains.positionDeadZone   = [XpositionDeadZone SpositionDeadZone];
gains.positionIClamp       = [XpositionIClamp SpositionIClamp];
gains.positionPunch         = [XpositionPunch SpositionPunch];
gains.positionMinTarget   = [XpositionMinTarget SpositionMinTarget];
gains.positionMaxTarget  = [XpositionMaxTarget SpositionMaxTarget];
gains.positionTargetLowpassGain = [XpositionTargetLowpassGain SpositionTargetLowpassGain];
gains.positionMinOutput  = [XpositionMinOutput SpositionMinOutput];
gains.positionMaxOutput = [XpositionMaxOutput SpositionMaxOutput];
gains.positionOutputLowpassGain = [XpositionOutputLowpassGain SpositionOutputLowpassGain];
gains.positionDOnError    = [XpositionDOnError SpositionDOnError];

gains.velocityKp             = [XvelocityKp SvelocityKp];
gains.velocityKi              = [XvelocityKi SvelocityKi];
gains.velocityKd             = [XvelocityKd SvelocityKd];
gains.velocityFF              = [XvelocityFF SvelocityFF];
gains.velocityDeadZone   = [XvelocityDeadZone SvelocityDeadZone];
gains.velocityIClamp       = [XvelocityIClamp SvelocityIClamp];
gains.velocityPunch         = [XvelocityPunch SvelocityPunch];
gains.velocityMinTarget   = [XvelocityMinTarget SvelocityMinTarget];
gains.velocityMaxTarget  = [XvelocityMaxTarget SvelocityMaxTarget];
gains.velocityTargetLowpassGain = [XvelocityTargetLowpassGain SvelocityTargetLowpassGain];
gains.velocityMinOutput  = [XvelocityMinOutput SvelocityMinOutput];
gains.velocityMaxOutput = [XvelocityMaxOutput SvelocityMaxOutput];
gains.velocityOutputLowpassGain = [XvelocityOutputLowpassGain SvelocityOutputLowpassGain];
gains.velocityDOnError    = [XvelocityDOnError SvelocityDOnError];

gains.effortKp             = [XeffortKp SeffortKp];
gains.effortKi              = [XeffortKi SeffortKi];
gains.effortKd             = [XeffortKd SeffortKd];
gains.effortFF              = [XeffortFF SeffortFF];
gains.effortDeadZone   = [XeffortDeadZone SeffortDeadZone];
gains.effortIClamp       = [XeffortIClamp SeffortIClamp];
gains.effortPunch         = [XeffortPunch SeffortPunch];
gains.effortMinTarget   = [XeffortMinTarget SeffortMinTarget];
gains.effortMaxTarget  = [XeffortMaxTarget SeffortMaxTarget];
gains.effortTargetLowpassGain = [XeffortTargetLowpassGain SeffortTargetLowpassGain];
gains.effortMinOutput  = [XeffortMinOutput SeffortMinOutput];
gains.effortMaxOutput = [XeffortMaxOutput SeffortMaxOutput];
gains.effortOutputLowpassGain = [XeffortOutputLowpassGain SeffortOutputLowpassGain];
gains.effortDOnError    = [XeffortDOnError SeffortDOnError];

this.fullGroup.send('gains', gains);