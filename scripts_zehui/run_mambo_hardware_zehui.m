%% load obstacles
% don't use clear clc here, load_obstacles.m has those commands
% specify all the obstacles in load_obstacles.m
% and directly run this script

% if you only want random obstacles, make this flag true
random_obstacles_only_flag = false ;
if ~random_obstacles_only_flag
    run load_obstacles.m
end

%% user parameters
% general params
traj_dir = '/home/roahmlab/Mambo_scripts/low_level_controller/traj_lib/' ;
% traj_dir = '~/MATLAB/drone_traj/' ;
save_csv_flag = true ;
t_sample = 0.05 ;

% quadrotor
if speed_flag
    v_max = 1.5 ;
    frs_filename = 'mambo_zehui_FRS_1_5.mat' ;
else
    v_max = 1.0 ;
    frs_filename = 'mambo_zehui_FRS.mat' ;
end

sensor_radius = 100 ;

% planner
N_sample = 41 ;
use_fmincon_online_flag = false ;
t_plan = 0.5 ;
tracking_error_type = 'none' ; % 'none' or 'table' or 'constant'
tracking_error_value = 0.0 ; % use 0 for flights with virtual obstacles
add_noise_to_waypoint = false ;
high_level_planner_lookahead_dist = 0.5 ; % meters
t_overrun = 0.004 ; % the loop always takes approx 10ms extra

% world
bounds = [-2.4 2.9 -1.0 1.2 0.4 2.0] ; % [xlo, xhi, ylo, yhi, zlo, zhi] in NWU coords
goal_radius = 0.25 ;
min_distance_of_obs_from_start = 0.5 ; % m
min_distance_of_obs_from_goal = 0.3 ; % m
N_tall = 0 ;
N_wide = 0 ;
N_long = 0 ;
N_boxy = 0 ;
N_dyn = 0 ;
tall_dims = [0.2 0.5] ;
wide_dims = [0.1 0.2] ;
long_dime = [0.1 0.2] ;
boxy_dims = [0.2 0.4 0.3 0.1] ;
dyn_dims = [0.3 0.6 0.5 0.1] ;

% params for hardware trial
max_iterations = 40 ;
max_time = 60 ;
plot_flag = true ;
verbose_level = 0 ;

%% automated from here
% load FRS
FRS = load(frs_filename) ;

% convert start and goal to simulator frame
R_p2s = [1 0 0; 0 0 -1; 0 1 0];
start = R_p2s * start ;
goal = R_p2s * goal ;

% create world
W = zonotope_mambo_world('verbose',verbose_level,'N_tall',N_tall,...
    'N_wide',N_wide,'N_boxy',N_boxy,'N_long',N_long,'N_dyn',N_dyn,...
    'goal_radius',goal_radius,...
    'obs_tall_size_range',tall_dims,...
    'obs_wide_size_range',wide_dims,...
    'obs_boxy_size_range',boxy_dims,...
    'obs_dyn_size_range',dyn_dims,...
    'bounds',bounds,...
    'buffer_start',min_distance_of_obs_from_start,...
    'buffer_goal',min_distance_of_obs_from_goal,...
    'use_wall_obstacles_flag',true,...
    'start',start,'goal',goal) ;

%% add obstacle to world, added by Zehui Lu, Nov. 21 2019

if ~random_obstacles_only_flag
    run create_obstacles_in_world.m
end

%%
% create high-level planner
HLP = mambo_dijkstra_HLP('default_lookahead_distance',high_level_planner_lookahead_dist) ;
HLP.get_path(W) ;

if isempty(HLP.best_path_points)
    error('Unable to find collision-free path through world!')
end

% create planner
P = quadrotor_zono_RTD_planner(FRS,[],'t_move',t_plan,'verbose',verbose_level,...
    'v_max',v_max,'timeout',t_plan,...
    't_plan',t_plan,'t_move',t_plan,'t_sample',t_sample,...
    'tracking_error_type','constant',...
    'tracking_error_constant_value',tracking_error_value,...
    'use_fmincon_online_flag',use_fmincon_online_flag,...
    'plot_zonotope_reach_set_flag',false,'plot_zono_style','tube',...
    'add_noise_to_waypoint',add_noise_to_waypoint,...
    'use_agent_for_initial_condition',false,...
    'N_sample',N_sample,...
    'HLP',HLP) ;

%% add obstacle to world
if exist('obs_x_lo','var')
    obs_bounds = [obs_x_lo(:) obs_x_hi(:) obs_y_lo(:) obs_y_hi(:) obs_z_lo(:) obs_z_hi(:)] ;
    for idx = 1:size(obs_bounds,1) % 1:size(obs_length,2)
        % W.add_obstacle(obs_length(idx),obs_width(idx),obs_height(idx),obs_center(:,idx))
        B_idx = obs_bounds(idx,:) ;
        [l,w,h,c] = bounds_to_box(B_idx) ;
        W.add_obstacle(l,w,h,c)
    end
end

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
t_agent = 0 ;
t_cur = toc(start_tic) ;
d_to_goal = inf ;

if plot_flag
    figure(1) ; clf ; hold on ; axis equal ;
    plot(W) ; view(3) ;
    X = P.HLP.best_path_points ;
    plot_path(X,'--','LineWidth',1.5)
end

% NWU to NUE coordinate rotation
R_fix = [1 0 0 ;
    0 0 1 ;
    0 -1 0] ;

R_fix = blkdiag(R_fix,R_fix,R_fix) ;

while iter_cur < max_iterations && t_cur < max_time && d_to_goal > W.goal_radius
    % start in-loop timer
    loop_start_tic = tic ;
    
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
    
    % update agent_info time and position
    t_agent = iter_cur*t_plan ;
    agent_info.time = 0:t_sample:t_agent ;
    if iter_cur > 1
        agent_info.position = match_trajectories(t_agent,T_save,Z_save(1:3,:)) ;
    end
    
    % get north-west-up frame for plotting and goal check
    Z_NWU = R_fix'*Z_save ;
    
    % save csv
    f_traj = [traj_dir, 'traj_',num2str(iter_cur,'%03.f'),'.csv'] ;
    if save_csv_flag
        disp(['Writing ',f_traj])
        writematrix([T_save ; Z_save], f_traj)
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