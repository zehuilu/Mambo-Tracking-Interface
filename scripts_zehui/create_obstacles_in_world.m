%% add obstacle to world, added by Zehui Lu, Nov. 21 2019
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