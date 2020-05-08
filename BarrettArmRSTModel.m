% Om namo narayanaya
% Om gum ganapathaye namo namaha
clear all;
clc;

%% Setup the Robotics Systems Toolbox model for the Barrett arm model
m0 = 10.17;
m = [ 10.23, 4.06, 1.7, 2.5, 0.12, 0.5, 0.08];

r0 = [-0.015, -0.271, -0.145];
r = [-0.007, 0.127, 0;
     -0.002, 0.031, 0.015;
     -0.042, 0.210, 0;
     0.003, 0, 0.138;
     0, 0.007, 0.002;
     0, -0.024, 0.028;
     -1.8e-4, 2.6e-4, 35.3e-4];
 
I0 = [0.089, 0.142, 0.187];
I = [0.085, 0.107, 0.128;
     0.013, 0.018, 0.021;
     0.003, 0.058, 0.058;
     0.004, 0.016, 0.016;
     4.5e-5, 5.4e-5, 6.9e-5;
     2.7e-4, 5.75e-4, 6.8e-4;
     4.2e-5, 4.21e-5, 8.45e-5];
 
Q0 = [0.763 0.64 -0.089; 
      0.062 0.065 0.996;
      0.643 -0.765 0.01];
Q1 = [-0.004 0.097 0.995;
       0.03 -0.995 0.097; 
       0.999 0.031 0.001];
Q2 = [0.025 0.035 0.999; 
      0.85 0.525 -0.039; 
      -0.525 0.85 -0.016];
Q3 = [-0.044 -0.99 -0.132;
       0.999 -0.044 -0.005;
       0 -0.132 0.991];
Q4 = [-0.123 -0.053 0.991; 
      -0.002 -0.998 -0.053;
       0.992 -0.01 0.123];
Q5 = [0.999 -0.01 -0.01;
      0.01 0.247 0.969; 
      0 -0.969 0.248];
Q6 = [0.002 -0.006 0.999;
      0.979 0.204 0;
     -0.205 0.979 0.007];
Q7 = [0.913 0.408 0;
     -0.408 0.913 0;
      0 0 0.999];
Qs = [Q1,Q2,Q3,Q4,Q5,Q6,Q7];

gravityVec = [0 0 -9.80665];

% import and add the given params
wam_model = importrobot('wam_model.urdf');
wam_model.DataFormat = 'column';
wam_model.Gravity = gravityVec;

% base body init.
% wam_model.Base.Mass = m0;
% wam_model.Base.CenterOfMass = r0;
% wam_model.Base.Inertia = getInertiaVector(Q0,I0);

% link bodies init.
for li = 1:7
    wam_model.Bodies{1,li}.Mass = m(li);
    wam_model.Bodies{1,li}.CenterOfMass = r(li,:);
    wam_model.Bodies{1,li}.Inertia = getInertiaVector(Qs(li),I(li,:));
end

tool = robotics.RigidBody('tool');
tform = trvec2tform([0, 0, 0.12]); % User defined
setFixedTransform(tool.Joint,tform);
wam_model.addBody(tool, wam_model.BodyNames{1,7});

clear m0 m I0 I Qs Q0 Q1 Q2 Q3 Q4 Q5 Q6 Q7 r r0 li tool tform

showdetails(wam_model);
show(wam_model);
%% Setup the Robotics Systems Toolbox model for the Barrett arm simulation

m0 = 9.97;
m = [10.77, 3.87, 1.8, 2.4, 0.123, 0.42, 0.069];

r0 = [-0.02, -0.266, -0.14];
r = [-0.004, 0.122, 0;
     -0.002, 0.031, 0.015;
     -0.038, 0.207, 0;
     0.005, 0, 0.133;
     0, 0.005, 0.004;
     0, -0.02, 0.025;
     -0.8e-4, 1.6e-4, 32.3e-4];
 
I0 = [0, -0.02, 0.025];
I = [0.09, 0.113, 0.135;
     0.012, 0.017, 0.02;
     0.003, 0.059, 0.059;
     0.003, 0.015, 0.015;
     5e-5, 6e-5, 7.7e-5;
     2.3e-4, 4.6e-4, 5.5e-4;
     3.7e-5, 3.8e-5, 7.4e-5];
 
Q0 = [0.763 0.64 -0.089; 
      0.062 0.065 0.996;
      0.643 -0.765 0.01];
Q1 = [-0.004 0.097 0.995;
       0.03 -0.995 0.097; 
       0.999 0.031 0.001];
Q2 = [0.025 0.035 0.999; 
      0.85 0.525 -0.039; 
      -0.525 0.85 -0.016];
Q3 = [-0.044 -0.99 -0.132;
       0.999 -0.044 -0.005;
       0 -0.132 0.991];
Q4 = [-0.123 -0.053 0.991; 
      -0.002 -0.998 -0.053;
       0.992 -0.01 0.123];
Q5 = [0.999 -0.01 -0.01;
      0.01 0.247 0.969; 
      0 -0.969 0.248];
Q6 = [0.002 -0.006 0.999;
      0.979 0.204 0;
     -0.205 0.979 0.007];
Q7 = [0.913 0.408 0;
     -0.408 0.913 0;
     0  0 0.999];
 
Qs = [Q1,Q2,Q3,Q4,Q5,Q6,Q7];

% import and add the given params
wam_sim = importrobot('wam_model.urdf');
wam_sim.DataFormat = 'column';
wam_sim.Gravity = gravityVec;

% base body init.
% wam_sim.Base.Mass = m0;
% wam_sim.Base.CenterOfMass = r0;
% wam_sim.Base.Inertia = getInertiaVector(Q0,I0);

% link bodies init.
for li = 1:7
    wam_sim.Bodies{1,li}.Mass = m(li);
    wam_sim.Bodies{1,li}.CenterOfMass = r(li,:);
    wam_sim.Bodies{1,li}.Inertia = getInertiaVector(Qs(li),I(li,:));
end

tool = robotics.RigidBody('tool');
tform = trvec2tform([0, 0, 0.12]); % User defined
setFixedTransform(tool.Joint,tform);
wam_sim.addBody(tool, wam_sim.BodyNames{1,7});

clear m0 m I0 I Qs Q0 Q1 Q2 Q3 Q4 Q5 Q6 Q7 r r0 li tool tform gravityVec

showdetails(wam_sim);

%% Run the IK through

baseTrans = [0.75,0.5,1];
intrvl = 80;
traj = dlmread('tooltip_xyz.txt')-baseTrans;
traj(:,1:2) = -traj(:,1:2);
traj = (rotz(45)*traj')';
traj(:,2) = traj(:,2) + 0.275;
waypts = traj(1:intrvl:end,:);

plot3(waypts(:,1), waypts(:,2), waypts(:,3), '*k');
hold on;
show(wam_model);
axis square;
%%
t = (0:0.2:10)'; % Time
count = length(t);

q0 = homeConfiguration(wam_model);
ndof = length(q0);
qs = zeros(count, ndof);

ik = robotics.InverseKinematics('RigidBodyTree', wam_model);
weights = [1, 1, 1, 1, 1, 1];
endEffector = 'tool';
eePose = zeros(count, 3);

qInitial = q0;
% Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position 
    point = waypts(i,:);    
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration    
    qs(i,:) = qSol;
    
    eePose(i,:) = tform2trvec(getTransform(wam_model,qSol,endEffector));
    % Start from prior solution    
    qInitial = qSol;
    
end
fprintf("IK done!");

%%
ttraj = cscvn(waypts');
%plot ttraj splines

plot3(eePose(:,1), eePose(:,2), eePose(:,3), '^r');
hold on;
plot3(waypts(1:51,1), waypts(1:51,2), waypts(1:51,3), '*k');
hold on;
fnplt(ttraj, 'g',2);

hold on;
show(wam_model);
axis square;