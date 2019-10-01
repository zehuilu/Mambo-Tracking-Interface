function [traj_ped_1,time_ped_1,traj_ped_2,time_ped_2] = add_ped_obstacle()
% in simulator frame!!!
    position_ini_1 = [0.0; 1.8; 1.0] ;
    vy_1 = -0.50 ;
    traj_py_1 = position_ini_1(2) : vy_1 : -5.00 ;
    traj_px_1 = position_ini_1(1) .* ones(size(traj_py_1)) ;
    traj_pz_1 = position_ini_1(3) .* ones(size(traj_py_1)) ;
    traj_ped_1 = [traj_px_1; traj_py_1; traj_pz_1] ;
    time_ped_1 = 0 : 1 : size(traj_ped_1, 2) - 1;

    
    position_ini_2 = [-2.42; -1.5; +1.0] ;
    position_end_2 = [+3.28; +1.99; +1.0] ;
    vec_2 = position_end_2 - position_ini_2 ;
    vxy_2 = 0.50 ;
    ratio_2 = vec_2(1) / vec_2(2) ;
    vx_2 = ratio_2 * vxy_2 / (sqrt(ratio_2^2 + 1)) ;
    vy_2 = vxy_2 / (sqrt(ratio_2^2 + 1)) ;
    
    traj_px_2 = position_ini_2(1) : vx_2 : +10.0 ;
    num_second_2 = size(traj_px_2, 2) - 1 ;
    traj_py_2 = position_ini_2(2) : vy_2 : position_ini_2(2)+vy_2*num_second_2 ;
    traj_pz_2 = position_ini_2(3) .* ones(size(traj_px_2)) ;
    traj_ped_2 = [traj_px_2; traj_py_2; traj_pz_2] ;
    time_ped_2 = 0 : 1 : num_second_2;