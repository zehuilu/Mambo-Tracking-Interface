clear ; clc ; close all ;

%% user parameters

add_rover_flag = false ;
create_obs_manually_flag = true ;
add_ped_flag = false ;

% one second delay for corner ped

% No. x dynamic rover
% No.1, eight shape
% No.2, circle
name_rover = 2 ;

% for No.2, start matlab first, after it starts computing, hit the Enter for rover
% and remember to release the kill switch after the rover arrives at the desired position

% No. x static obstacles
% DOn't use No.2 here!!!
name_obstacles = 3 ;


% add rover as obstacle
l_rover = 0.6 ;
w_rover = 0.6 ;
h_rover = 2.0 ;
%path for dynamic obstacle, in simulator frame
if add_rover_flag == true
    if name_rover == 1
        position_ini = [-1.90; -1.22; 0.0];
    else
        position_ini = [-0.74; -1.22; 0.0];
    end
    [position_rover,speed_or_time_rover] = add_rover_obstacle(name_rover, position_ini, h_rover) ;
end

% add pedestrian as obstacles
l_ped_1 = 0.8 ;
w_ped_1 = 0.8 ;
h_ped_1 = 2.0 ;
if add_ped_flag == true
    [traj_ped_1,time_ped_1,traj_ped_2,time_ped_2] = add_ped_obstacle() ;
end

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
end

% general params
traj_dir = '/home/roahmlab/Mambo_scripts/low_level_controller/traj_lib/' ;
all_data_dir = '/home/roahmlab/Mambo_scripts/low_level_controller/sysid_data/' ;

tracking_error_amount = 0.2 ;

high_level_planner_lookahead_dist = 0.5 ; % meters
save_csv_flag = true ;
save_data_flag = true ;
if create_obs_manually_flag
    [start,goal,obs_x_lo,obs_x_hi,obs_y_lo,obs_y_hi,obs_z_lo,obs_z_hi] ...
        = load_static_obstacles(name_obstacles) ;
end
% convert to simulator frame
R_p2s = [1 0 0; 0 0 -1; 0 1 0];
start = R_p2s * start ;
goal = R_p2s * goal ;

plot_flag = true ;
verbose_level = 0 ;
t_overrun = 0.004 ; % the loop always takes approx 10ms extra

% quadrotor
v_max = 1.0 ;
sensor_radius = 100 ;

% planner
N_sample = 41 ;
use_fmincon_online_flag = false ; 
t_plan = 0.5 ;
tracking_error_type = 'constant' ; % 'none' or 'table' or 'constant'

% frs_filename = 'mambo_FRS.mat' ;
frs_filename = 'mambo_zehui_FRS.mat' ;

add_noise_to_waypoint = false ;

% CSV output
t_sample = 0.05 ;

% world
bounds = [-2.6 3.0 -1.2 1.5 0.0 2.2] ; % [xlo, xhi, ylo, yhi, zlo, zhi] in NWU coords
goal_radius = 0.25 ;
N_tall = 0 ;
N_wide = 0 ;
N_long = 0 ;
N_boxy = 0 ;
N_dyn = 0 ;
tall_dims = [0.1 0.2] ;
wide_dims = [0.1 0.2] ;
long_dime = [0.1 0.2] ;
boxy_dims = [0.1 0.2 0.1 0.2] ;
dyn_dims = [0.1 0.2 0.2 0.1] ;
obs_color = [1 0.7 0.7] ;
obs_opacity = 0.5 ;

% manually add obstacles
% in simulator frame
% obs_length = 0.1 ;
% obs_width = 0.2 ;
% obs_height = 0.5 ;
% obs_center = [0 ; 0 ; 1] ;

% hardware loop
max_iterations = 80 ;
max_time = 60 ;

%% automated from here
% load FRS
FRS = load(frs_filename) ;

% create world
W = zonotope_mambo_world('verbose',verbose_level,'N_tall',N_tall,...
    'N_wide',N_wide,'N_boxy',N_boxy,'N_long',N_long,'N_dyn',N_dyn,...
    'obs_color',obs_color,...
    'obs_opacity',obs_opacity,...
    'goal_radius',goal_radius,...
    'obs_tall_size_range',tall_dims,...
    'obs_wide_size_range',wide_dims,...
    'obs_boxy_size_range',boxy_dims,...
    'obs_dyn_size_range',dyn_dims,...
    'bounds',bounds,...
    'buffer_start',0.5,...
    'use_wall_obstacles',true,...
    'start',start,'goal',goal) ;

%% add obstacle to world
if create_obs_manually_flag
    obs_bounds = [obs_x_lo(:) obs_x_hi(:) obs_y_lo(:) obs_y_hi(:) obs_z_lo(:) obs_z_hi(:)] ;
    for idx = 1:size(obs_bounds,1) % 1:size(obs_length,2)
        % W.add_obstacle(obs_length(idx),obs_width(idx),obs_height(idx),obs_center(:,idx))
        B_idx = obs_bounds(idx,:) ;
        [l,w,h,c] = bounds_to_box(B_idx) ;
        W.add_obstacle(l,w,h,c)
    end
end

if add_rover_flag
% add dynamic obstacles
    W.add_obstacle(l_rover,w_rover,h_rover,position_rover,speed_or_time_rover) ;
    % W.add_obstacle(l_rover,w_rover,h_rover,position(:, 45)) ;
    
    if name_rover == 2
        obs_bounds = [obs_x_lo(:) obs_x_hi(:) obs_y_lo(:) obs_y_hi(:) obs_z_lo(:) obs_z_hi(:)] ;
        for idx = 1:size(obs_bounds,1) % 1:size(obs_length,2)
        % W.add_obstacle(obs_length(idx),obs_width(idx),obs_height(idx),obs_center(:,idx))
            B_idx = obs_bounds(idx,:) ;
            [l,w,h,c] = bounds_to_box(B_idx) ;
            W.add_obstacle(l,w,h,c)
        end
    end
end

if add_ped_flag
    W.add_obstacle(l_ped_1,w_ped_1,h_ped_1,traj_ped_1,time_ped_1) ;
    W.add_obstacle(l_ped_1,w_ped_1,h_ped_1,traj_ped_2,time_ped_2) ;
    
    obs_bounds = [obs_x_lo(:) obs_x_hi(:) obs_y_lo(:) obs_y_hi(:) obs_z_lo(:) obs_z_hi(:)] ;
    for idx = 1:size(obs_bounds,1) % 1:size(obs_length,2)
        % W.add_obstacle(obs_length(idx),obs_width(idx),obs_height(idx),obs_center(:,idx))
        B_idx = obs_bounds(idx,:) ;
        [l,w,h,c] = bounds_to_box(B_idx) ;
        W.add_obstacle(l,w,h,c)
    end
end

%% add any obstacles before we call high-level planner
% create high-level planner
HLP = mambo_dijkstra_HLP('default_lookahead_distance',high_level_planner_lookahead_dist) ;
HLP.get_path(W) ;

% create planner
P = quadrotor_zono_RTD_planner(FRS,[],'t_move',t_plan,'verbose',verbose_level,...
    'v_max',v_max,'timeout',t_plan,...
    't_plan',t_plan,'t_move',t_plan,'t_sample',t_sample,...
    'tracking_error_type','constant',...
    'use_fmincon_online_flag',use_fmincon_online_flag,...
    'plot_zonotopes',false,'zono_plot_style','tube',...
    'add_noise_to_waypoint',add_noise_to_waypoint,...
    'use_agent_for_initial_condition',false,...
    'N_sample',N_sample,...
    'tracking_error_constant_value',tracking_error_amount,...
    'HLP',HLP) ;

%% set up for planning loop
agent_info.time = 0 ;
agent_info.position = mean(reshape(W.bounds(:),2,3),1)' ;
agent_info.sensor_radius = sensor_radius ;
world_info = W.get_world_info(agent_info) ;
P.setup(agent_info,world_info) ;
T_save = [] ;
Z_save = [] ;

%% set up for CSV publishing
% create traj dir if it doesn't exist
mkdir(traj_dir)

% delete all CSV files in folder
traj_files = dir(traj_dir) ;
for idx = 1:length(traj_files)
    f_name = traj_files(idx).name ;
    try
        if strcmp(f_name(end-3:end),'.csv')
            f_name = [traj_dir, f_name] ;
            disp(['Deleting ',f_name])
            delete(f_name)
        end
    catch
    end
end

%% run planning loop
start_tic = tic ;
iter_cur = 1 ;
t_cur = toc(start_tic) ;
d_to_goal = inf ;

if plot_flag
    figure(1) ; clf ; hold on ; axis equal ;
    plot(W) ; view(3) ;
end

% NWU to NUE coordinate rotation
R_fix = [1 0 0 ;
         0 0 1 ;
         0 -1 0] ;

R_fix = blkdiag(R_fix,R_fix,R_fix) ;

while iter_cur < max_iterations && t_cur < max_time && d_to_goal > W.goal_radius
    % start in-loop timer
    loop_start_tic = tic ;
    
    % set agent_info time
    t_agent = iter_cur*t_plan ;
    agent_info.time = 0:t_sample:t_agent ;
    if iter_cur > 1
        agent_info.position = match_trajectories(t_agent,T_save,Z_save(1:3,:)) ;
    end
    
    % update world
    W.update_obstacle_times(t_agent) ;

    % call planner
    [T_new,~,Z_new] = P.replan(agent_info,world_info) ;
    Z_new_fix = R_fix*Z_new(1:9,:) ;
    
    if iter_cur > 1
        % interpolate old trajectory until t_plan
        T_old = 0:t_sample:(iter_cur*t_plan) ;
        Z_old = match_trajectories(T_old,T_save,Z_save) ;
        
        % add on new trajectory
        T_new = T_new + T_old(end) ;
        T_save = [T_old, T_new(2:end)] ;
        Z_save = [Z_old, Z_new_fix(:,2:end)] ;
    else
        T_save = T_new ;
        Z_save = Z_new_fix ;
    end
    
    % get north-west-up frame for plotting and goal check
    Z_NWU = R_fix'*Z_save ;
    
    % save csv
    f_traj = [traj_dir, 'traj_',num2str(iter_cur,'%03.f'),'.csv'] ;
    if save_csv_flag
        disp(['Writing ',f_traj])
        writematrix([T_save ; Z_save(1:6, :)], f_traj)
    end
    
    % plot
    if plot_flag
        plot(W) ;
        
        if exist('traj_plot_data','var')
            traj_plot_data.XData = Z_NWU(1,:) ;
            traj_plot_data.YData = Z_NWU(2,:) ;
            traj_plot_data.ZData = Z_NWU(3,:) ;
        else
            traj_plot_data = plot3(Z_NWU(1,:),Z_NWU(2,:),Z_NWU(3,:)) ;
        end
        
        %campos([0.2000    -0.1500   34.0735]) ;
    end
    
    % check distance to goal
    d_to_goal = norm(W.goal - Z_NWU(1:3,end)) ;
    
    % iterate
    t_cur = toc(start_tic) ;
    iter_cur = iter_cur + 1 ;
    
    % pause for next iteration
    t_in_loop = toc(loop_start_tic) ;
    pause(t_plan - t_in_loop - t_overrun)
    toc(loop_start_tic) ;
end

%% wrapup
disp(['Total number of iterations: ',num2str(iter_cur)])
disp(['Total time: ',num2str(t_cur)])
disp(['Final distance to goal: ',num2str(d_to_goal)]) ;
disp("Change the total time in LLC.py!!!!!!!")
disp("Print the final time")
disp(T_save(end-3:end))

%% save all the variables
if save_data_flag
    error_file_name = strcat(all_data_dir, 'matlab_data_', datestr(now, 'yyyymmddTHHMMSS')) ;
    save(error_file_name)
end
