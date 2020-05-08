%Setup all the symbolic variables and initialize the robot's twists etc.

function [M, C, N] = BarrettWAM_EOM(tws)
    % Init the variables and constants
    syms jt1 jt2 jt3 jt4 jt5 jt6 jt7 real
    
    jt = [jt1, jt2, jt3, jt4, jt5, jt6, jt7];
    
    % Define the joint twists => tws
    m = [ 10.23, 4.06, 1.7, 2.5, 0.12, 0.5, 0.08];%10.17,
    
    r = [-0.007, 0.127, 0;
         -0.002, 0.031, 0.015;
         -0.042, 0.210, 0;
         0.003, 0, 0.138;
         0, 0.007, 0.002;
         0, -0.024, 0.028;
         -1.8e-4, 2.6e-4, 35.3e-4];%-0.015, -0.271, -0.145;
     
    I = [0.085, 0.107, 0.128;
     0.013, 0.018, 0.021;
     0.003, 0.058, 0.058;
     0.004, 0.016, 0.016;
     4.5e-5, 5.4e-5, 6.9e-5;
     2.7e-4, 5.75e-4, 6.8e-4;
     4.2e-5, 4.21e-5, 8.45e-5];%0.089, 0.142, 0.187;

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
      
%     Qs = [Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7]; 

    % Transforms till the ith link
    g_sl1_init = [eye(3) r(1,:)'; 0, 0, 0, 1];
    g_sl2_init = [eye(3) r(2,:)'; 0, 0, 0, 1];
    g_sl3_init = [eye(3) r(3,:)'; 0, 0, 0, 1];
    g_sl4_init = [eye(3) r(4,:)'; 0, 0, 0, 1];
    g_sl5_init = [eye(3) r(5,:)'; 0, 0, 0, 1];
    g_sl6_init = [eye(3) r(6,:)'; 0, 0, 0, 1];
    g_sl7_init = [eye(3) r(7,:)'; 0, 0, 0, 1];

    % Define the principal axis aligned Inertia matrices for the links
    
    % Define the Mu matrices for the 3 links
    Mu1 = [m(1)*eye(3) 0*eye(3); 0*eye(3) Q1*diag(I(1,:))*Q1'];
    Mu2 = [m(2)*eye(3) 0*eye(3); 0*eye(3) Q2*diag(I(2,:))*Q2'];
    Mu3 = [m(3)*eye(3) 0*eye(3); 0*eye(3) Q3*diag(I(3,:))*Q3'];
    Mu4 = [m(4)*eye(3) 0*eye(3); 0*eye(3) Q4*diag(I(4,:))*Q4'];
    Mu5 = [m(5)*eye(3) 0*eye(3); 0*eye(3) Q5*diag(I(5,:))*Q5'];
    Mu6 = [m(6)*eye(3) 0*eye(3); 0*eye(3) Q6*diag(I(6,:))*Q6'];
    Mu7 = [m(7)*eye(3) 0*eye(3); 0*eye(3) Q7*diag(I(7,:))*Q7'];

    % Define the Jacobians for the 3 links
    J1 = calcAD_g(inv(g_sl1_init))*calcJ_si(tws, jt, 1);
    J2 = calcAD_g(inv(g_sl2_init))*calcJ_si(tws, jt, 2);
    J3 = calcAD_g(inv(g_sl3_init))*calcJ_si(tws, jt, 3);
    J4 = calcAD_g(inv(g_sl4_init))*calcJ_si(tws, jt, 4);
    J5 = calcAD_g(inv(g_sl5_init))*calcJ_si(tws, jt, 5);
    J6 = calcAD_g(inv(g_sl6_init))*calcJ_si(tws, jt, 6);
    J7 = calcAD_g(inv(g_sl7_init))*calcJ_si(tws, jt, 7);
    
    % components of the resulting manipulator inertia matrix M(theta)
    % given by on MLS p.173 
    M = (J1'*Mu1*J1) + (J2'*Mu2*J2) + (J3'*Mu3*J3) + (J4'*Mu4*J4) + (J5'*Mu5*J5) +(J6'*Mu6*J6) + (J7'*Mu7*J7);

    fprintf("M:\n");
    disp(M);


    % components of the vector C
    % given by on MLS p.173 
    C = zeros(3,3);
    C = sym(C);
    for i=1:7
        for j=1:7
            c = 0;
            for k=1:7
                C(i,j) = C(i,j) + (diff(M(i,j),jt(k)) + diff(M(i,k),jt(j)) - diff(M(k,j),jt(i)))/2;
            end
        end
    end

    fprintf("C:\n");
    disp(C);


   
    % components of the vector N
    % given by on MLS p.175, eqn.4.25 
    % The heights of the 3 links
    h1 = calcGi(tws, jt, g_sl1_init, 1);
    h2 = calcGi(tws, jt, g_sl2_init, 2);
    h3 = calcGi(tws, jt, g_sl3_init, 3);
    h4 = calcGi(tws, jt, g_sl4_init, 4);
    h5 = calcGi(tws, jt, g_sl5_init, 5);
    h6 = calcGi(tws, jt, g_sl6_init, 6);
    h7 = calcGi(tws, jt, g_sl7_init, 7);
    
    h = [h1(3,4), h2(3,4), h3(3,4), h4(3,4), h5(3,4), h6(3,4), h7(3,4)];
    V = (m(1)*g*h(1)) + (m(2)*g*h(2)) + (m(3)*g*h(3))+ (m(4)*g*h(4))+ (m(5)*g*h(5))+ (m(6)*g*h(6))+ (m(7)*g*h(7));
    N = zeros(3,1);
    N = sym(N);
    N(1) = diff(V, jt(1));
    N(2) = diff(V, jt(2));
    N(3) = diff(V, jt(3));
    N(4) = diff(V, jt(4));
    N(5) = diff(V, jt(5));
    N(6) = diff(V, jt(6));
    N(7) = diff(V, jt(7));
    
    
    fprintf("N:\n");
    disp(N);

end
