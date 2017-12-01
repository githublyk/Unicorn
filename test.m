
close all; clear; clc;

startup;
HebiLookup

sGroup = HebiLookup.newGroupFromNames('*', ...
          {'SA065', 'SA068', 'SA069', ...
           'SA067', 'SA064', 'SA061', ...,
            'SA066', 'SA060'...
                   }); 

% this@FodbotOffline(2 + sGroup.getNumModules);

this.xGroup = HebiLookup.newGroupFromNames('*', ...
                   {'X-00036', ...
                    'X-00031'});
this.sGroup = sGroup; 

this.fullGroup = HebiLookup.newGroupFromNames('*',...
    [this.xGroup.getInfo.name(1:end);
     this.sGroup.getInfo.name(1:end)]);
 
 this.fullGroup
 
fbk = this.fullGroup.getNextFeedback();
pos_meas0 = fbk.position;
vel_meas0 = fbk.velocity;
tau_meas0 = fbk.torque;
cmd = CommandStruct();

group = this.fullGroup

kin = HebiKinematics();
kin.addBody('X5-4'); % 1st Module, 1st X-Module
kin.addBody('X5Bracket');
kin.addBody('GenericLink', 'CoM', ones(3,1), ...
                              'Output', rotz(pi), ... 
                              'mass', 0); 
kin.addBody('X5-9'); % 2nd Module, 2nd X-Module
kin.addBody('X5Link','Extension',.05,'twist',0); 
kin.addBody('FieldableElbowJoint'); % 3rd Module, 1st S-Module
kin.addBody('FieldableElbowJoint'); % 4th Module, 2nd S-Module
kin.addBody('FieldableElbowJoint'); % 5th Module, 3rd S-Module
kin.addBody('FieldableElbowJoint'); % 6th Module, 4th S-Module
kin.addBody('FieldableElbowJoint'); % 7th Module, 5th S-Module
 
group.send('led', 'red');

 