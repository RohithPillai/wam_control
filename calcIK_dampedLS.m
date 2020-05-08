function [iter, jt_new, V, err_norm] = calcIK_dampedLS(tws, jt_init, g_init, pos, k)
    
    pos_des = pos(1:3)';
    quat_des = quaternion(pos(7), pos(4), pos(5), pos(6));
    
    jt_curr = jt_init;
    err_norm = 1;
    iter = 0;
    kp = 1;
    ko = 10;
    t = 0.002;
    lamda = 0.01;
    
    jt_traj = [jt_init];
    
    while((err_norm > 1e-3) && (iter<5000))
       
        iter = iter + 1;
        g = calcG(tws, jt_curr, g_init);
        pos_curr = g(1:3,4);
        quat_curr = quaternion(g(1:3, 1:3), 'rotmat', 'point');
        
        pos_err = pos_des - pos_curr;
        quat_err = quat_des*(quat_curr.conj);
        quat_err_compact = quat_err.compact';
        
        err_norm = norm([pos_err; quat_err_compact(1)*quat_err_compact(2:4)]);
        
        V = [kp*pos_err; 
            ko*quat_err_compact(1)*quat_err_compact(2:4)];
        J = calcJ_s(tws, jt_curr, 0);
        dmpdJ = J'*inv((J*J')+lamda^2*eye(size(J,1)));
        jt_dot = dmpdJ*V;
        jt_curr = jt_curr + jt_dot*t;
        jt_traj = [jt_traj; jt_curr];
    end
    
    jt_new = jt_curr;
    
    % save joint trajectories to file
    dlmwrite(['jt_trajectory_dampedLS_xdes' num2str(k) '.txt'], jt_traj, ' ');
    
end
