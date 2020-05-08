%% Setup all the symbolic variables and initialize the robot's twists etc.
clear all;
clc;

% Init the variables and constants
syms l0 l1 l2 r0 r1 r2 m1 m2 m3 Ix1 Ix2 Ix3 Iy3 Iy1 Iy2 Iz3 Iz1 Iz2 th1 th2 th3 g real
th = [th1, th2, th3];

% Define the joint twists of elbow manipulator
jt1 = [0,0,0,0,0,1]';
jt2 = [0,-l0,0,-1,0,0]';
jt3 = [0,-l0, l1,-1,0,0]';

% Transforms till the ith link
g_sl1_init = [eye(3) [0,0,r0]'; 0, 0, 0, 1];
g_sl2_init = [eye(3) [0,r1,l0]'; 0, 0, 0, 1];
g_sl3_init = [eye(3) [0,l1+r2,l0]'; 0, 0, 0, 1];

% Define the principal axis aligned Inertia matrices for the links
I1 = diag([Ix1, Iy1, Iz1]);
I2 = diag([Ix2, Iy2, Iz2]);
I3 = diag([Ix3, Iy3, Iz3]);

% Define the Mu matrices for the 3 links
Mu1 = [m1*eye(3) 0*eye(3); 0*eye(3) I1];
Mu2 = [m2*eye(3) 0*eye(3); 0*eye(3) I2];
Mu3 = [m3*eye(3) 0*eye(3); 0*eye(3) I3];

% Define the Jacobians for the 3 links
J1 = zeros(6,3);
J1(6,1) = 1;

J2 = zeros(6,3);
J2 = sym(J2);
J2(1,1) = -r1*cos(th2);
J2(5,1) = -sin(th2);
J2(6,1) = cos(th2);
J2(3,2) = -r1;
J2(4,2) = -1;

J3 = zeros(6,3);
J3 = sym(J3);
J3(1,1) = (-l1*cos(th2)) - (r2*cos(th2+th3));
J3(5,1) = -sin(th2+th3);
J3(6,1) = cos(th2+th3);
J3(2,2) = l1*sin(th3);
J3(3,2) = -r2-(l1*cos(th3));
J3(4,2) = -1;
J3(3,3) = -r2;
J3(4,3) = -1;

%% Q1.1
% components of the resulting manipulator inertia matrix M(theta)
% given by on MLS p.173 
M = (J1'*Mu1*J1) + (J2'*Mu2*J2) + (J3'*Mu3*J3);

fprintf("M:\n");
disp(M);
fprintf("M(1,1):\n");
disp(M(1,1));

%% Q1.2
% components of the vector C
% given by on MLS p.173 
C = zeros(3,3);
C = sym(C);
for i=1:3
    for j=1:3
        c = 0;
        for k=1:3
            c = c + diff(M(i,j),th(k)) + diff(M(i,k),th(j)) - diff(M(k,j),th(i));
        end
        C(i,j) = 0.5*c;
    end
end

fprintf("C:\n");
disp(C);
fprintf("C(2,1):\n");
disp(C(2,1));

%% Q1.3
% components of the vector N
% given by on MLS p.175, eqn.4.25 
% The heights of the 3 links
h = [r0, l0-(r1*sin(th2)), l0-(l1*sin(th2))-(r2*sin(th2+th3))];
V = (m1*g*h(1)) + (m2*g*h(2)) + (m3*g*h(3));
N = zeros(3,1);
N = sym(N);
N(1) = diff(V, th(1));
N(2) = diff(V, th(2));
N(3) = diff(V, th(3));

% N = [0; -(cos(th2)*((m2*g*r1)+(m3*g*l1))) - (m3*r2*cos(th2+th3)); -m3*r2*cos(th2+th3)];
fprintf("N:\n");
disp(N);
fprintf("N(3):\n");
disp(N(3));
