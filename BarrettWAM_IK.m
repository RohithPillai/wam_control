%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implements the motion of the 7-DoF Barrett WAM^TM robotic arm
% Calculate Inverse kinematics using pseudo-inverse method
% NOTE: Much of the code structure follows from the 
% example from recitation 2.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
% load the joint data
jt_s = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7];
x_s = [0.44543, 1.12320, 2.22653, -0.29883, 0.44566, 0.84122, -0.06664];

x_des1 = [0.46320, 1.16402, 2.22058, -0.29301, 0.41901, 0.84979, 0.12817];
x_des2 = [ 0.49796, 0.98500, 2.34041, -0.11698, 0.07755, 0.82524, 0.54706];


% g_s = [-0.81252 -0.15424 -0.56216 0.44543;
%        -0.37847 -0.59390  0.70996 1.12320;
%        -0.44337  0.78962  0.42418 2.22653;
%         0.00000  0.00000  0.00000 1.00000];
    
    
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

%% Q3.1 

J_s = calcJ_s(tws, jt_s, 0);
fprintf("J_s:\n");
disp(J_s);

%% Q3.2 & Q3.3
des_pos = [x_des1; x_des2];

% pseudo Inv method for IK
for k=1:2
    fprintf("\nRunning IK for x_des%i ...\n", k);
    [iter, jt_new, V, err] = calcIK_pseudoInv(tws, jt_s, gst0, des_pos(k,:), k);
    gab_final = calcG(tws, jt_new', gst0);
    quat_ab_final = quaternion(gab_final(1:3,1:3), 'rotmat', 'point');
    quat_ab_final = quat_ab_final.compact';
    pos_final = [gab_final(1:3,4); quat_ab_final(2:4); quat_ab_final(1)];

    fprintf("V_s:\n");
    disp(V);

    fprintf("error:%f\n",err);
end

%% Q3.4

% Damped LS method of IK
for k=1:2
    fprintf("\nRunning IK for x_des%i ...\n", k);
    [iter, jt_new, V, err] = calcIK_dampedLS(tws, jt_s, gst0, des_pos(k,:), k);
    gab_final = calcG(tws, jt_new', gst0);
    quat_ab_final = quaternion(gab_final(1:3,1:3), 'rotmat', 'point');
    quat_ab_final = quat_ab_final.compact';
    pos_final = [gab_final(1:3,4); quat_ab_final(2:4); quat_ab_final(1)];

    fprintf("V_s:\n");
    disp(V);

    fprintf("error:%f\n",err);
end





