clear
clc

%%
add_rover_flag = false ;
create_obs_manually_flag = true ;
add_ped_flag = false ;

% true ==> 1.5 m/s, false ==> 1.0 m/s
speed_flag = false ;

% one second delay for corner ped

% for No.2 rover, start matlab first, after it starts computing, hit the Enter for rover
% and remember to release the kill switch after the rover arrives at the desired position

% dynamic rover obstacles
% No.1, eight shape
% No.2, circle
name_rover = 2 ;

% No. x static obstacles
% DOn't use No.2 here!!!
name_obstacles = 4 ;

% rover size
l_rover = 0.6 ;
w_rover = 0.6 ;
h_rover = 2.0 ;

% pedestrian size
l_ped_1 = 0.8 ;
w_ped_1 = 0.8 ;
h_ped_1 = 2.0 ;

%path for dynamic obstacle, in simulator frame
if add_rover_flag == true
    if name_rover == 1
        position_ini = [-1.90; -1.22; 0.0];
    else
        position_ini = [-0.74; -1.22; 0.0];
    end
    [position_rover,speed_or_time_rover] = add_rover_obstacle(name_rover, position_ini, h_rover) ;
end

% path for pedestrian, in simulator frame
if add_ped_flag == true
    [traj_ped_1,time_ped_1,traj_ped_2,time_ped_2] = add_ped_obstacle() ;
end

% define start and goal IN PHASESPACE FRAME
if ~create_obs_manually_flag
    if add_ped_flag
        % start from static No.3
        start = [-1.83 0.6 -0.772]' ;
        goal = [2.1 1.0 +0.5]' ;
        
        obs_x_lo = [+1.22] ;
        obs_x_hi = [+1.53] ;
        obs_y_lo = [-0.60] ;
        obs_y_hi = [-0.92] ;
        obs_z_lo = [0.0] ;
        obs_z_hi = [+1.35] ;
    % rover No.1
    elseif name_rover == 1
        % No. D3
        start = [-1.82 0.6 0.30]' ;
        goal = [2.1 1.0 +0.2]' ;
        
    % rover No.2
    else
        % start from static No.1
        start = [-1.82 0.6 0.60]' ;
        goal = [2.2 1.0 +0.30]' ;

        obs_x_lo = [+0.30] ;
        obs_x_hi = [+0.63] ;
        obs_y_lo = [-0.63] ;
        obs_y_hi = [-0.30] ;
        obs_z_lo = [0.0] ;
        obs_z_hi = [+2.00] ;
    end
    
else
    % create static obstacles manually
    [start,goal,obs_x_lo,obs_x_hi,obs_y_lo,obs_y_hi,obs_z_lo,obs_z_hi] ...
        = load_static_obstacles(name_obstacles) ;
end