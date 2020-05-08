%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implements the motion control of the 7-DoF Barrett WAM^TM robotic arm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;

% load the joint data
jtData = dlmread("JointData.txt"); 

% set up the init gst
gst0 = [0 -1 0 0.61;
        1  0 0 0.72;
        0  0 1 2.376;
        0  0 0 1    ];
        
% define the 7-DoF twists
twJ1 = getTwist([0 0 1]', [0.61 0.72 0]');
twJ2 = getTwist([-1 0 0]', [0 0.72 1.346]');
twJ3 = getTwist([0 0 1]', [0.61 0.72 0]');
twJ4 = getTwist([-1 0 0]', [0 0.765 1.896]');
twJ5 = getTwist([0 0 1]', [0.61 0.72 0]');
twJ6 = getTwist([-1 0 0]', [0 0.72 2.196]');
twJ7 = getTwist([0 0 1]', [0.61 0.72 0]');

tws = [twJ1, twJ2, twJ3, twJ4, twJ5, twJ6, twJ7];

% Run through the FK
eePoses = BarrettWAM_FK(jtData, tws, gst0, 0);

BarrettWAM_EOM(tws)
