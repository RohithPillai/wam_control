
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

gravityVec = [0 0 -9.81];
ndof = 7;

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

clear m0 m I0 I Qs Q0 Q1 Q2 Q3 Q4 Q5 Q6 Q7 r r0 li tool tform 

showdetails(wam_sim);

%% Set up the trajectory 

%
baseTrans = [0.75,0.5,1];
% intrvl = 80;
traj = dlmread('tooltip_xyz.txt')-baseTrans;
traj(:,1:2) = -traj(:,1:2);
traj = (rotz(45)*traj')';
traj(:,2) = traj(:,2) + 0.275;
% waypts = traj(1:intrvl:end,:);

T = size(traj,1); % Time

plot3(traj(:,1), traj(:,2), traj(:,3), '*k');
hold on;
show(wam_model);
axis square;


%% Run the IK through

q0 = homeConfiguration(wam_model);
ndof = length(q0);
qs = zeros(T, ndof);

ik = robotics.InverseKinematics('RigidBodyTree', wam_model);
weights = [1, 1, 1, 1, 1, 1];
endEffector = 'tool';
eePose = zeros(T, 3);

qInitial = q0;
% Use home configuration as the initial guess
for i = 1:T
    % Solve for the configuration satisfying the desired end effector
    % position 
    point = traj(i,:);    
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration    
    qs(i,:) = qSol;
%     eePose(i,:) = tform2trvec(getTransform(wam_model,qSol,endEffector));
    % Start from prior solution    
    qInitial = qSol;
    
end
fprintf("IK done!\n");
dlmwrite('wam_modelIKThetas.txt',qs);
dlmwrite('wam_modeleePose.txt',eePose);
%% Visualize robot configurations

qs = dlmread('wam_modelIKThetas.txt');
eePose_mod = dlmread('wam_modeleePose.txt');

dur = 10;
trajTimes = linspace(0,dur,T);
ts = trajTimes(1,2); % indv. timestep
shwEvry = 70;
viz = 0;

if viz 
    figure;
    set(gcf,'Visible','on');
    axis equal;
    plot3(traj(:,1), traj(:,2), traj(:,3), 'k-');
    hold on;
    title('Robot waypoint tracking visualization')
    st_t = cputime;
    for idx = 1:shwEvry:T % show only 100 points for efficiency

        show(wam_model, qs(idx,:)', 'PreservePlot',false);
        hold on;

        plot3(eePose_mod(idx,1), eePose_mod(idx,2), eePose_mod(idx,3), 'r-','LineWidth',2);
        hold on;
        pause(ts*shwEvry);    
    end
    fprintf("Time s:%.2f\n",cputime -st_t);
    hold off
end
%% Dynamics for simulation

clc;
% get the joint vel. and acc. for the inv. dynamics
q_mod = [qs; qs(end,:); qs(end,:)];
qd_mod = diff(qs,1)/ts;
qdd_mod = diff(qs,2)/ts;

% set up sim vars.
q_sim = zeros(size(q_mod));
qd_sim = zeros(size(qd_mod));
qdd_sim = zeros(size(qdd_mod));
eePose_sim = zeros(size(eePose_mod));

% sim and model begin at same config. and vel.
q_sim(1,:) = q_mod(1,:);
qd_sim(1,:) = qd_mod(1,:);

endEffector = 'tool';

figure;
set(gcf,'Visible','on');
plot3(traj(:,1), traj(:,2), traj(:,3), 'k*');
hold on;
    
% each time step
for t=1:T
    
    % get the tau from the inv dyn.
    tau_mod = inverseDynamics(wam_model,q_mod(t,:)',qd_mod(t,:)',qd_mod(t,:)');
    
    % get the actual motion from the sim using it's for dyn.
    qdd_sim(t,:) = forwardDynamics(wam_sim,q_sim(t,:)',qd_sim(t,:)',tau_mod);
    
    % update our actual sim joint config. and vel. values using Euler method
    qd_sim(t+1,:) =  qd_sim(t,:) + (ts*qdd_sim(t,:));
    q_sim(t+1,:) =  q_sim(t,:) + (ts*qd_sim(t,:));
    
    % find the actual eePose from the sim
    eePose_sim(t,:) = tform2trvec(getTransform(wam_sim,q_sim(t,:)',endEffector));
    
    %plot it 
    plot3(eePose_sim(t,1), eePose_sim(t,2), eePose_sim(t,3), 'b*');
    hold on;
    
end



