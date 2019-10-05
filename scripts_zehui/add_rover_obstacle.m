function [posi_global,time_traj] = add_rover_obstacle(name_rover, position_ini, h)
    % position_ini is in simulator frame, 3 by 1
    
    Directory_1 = '/home/roahmlab/Desktop/rover/offline_lqr/ref_traj_eight_shape_zehui.mat';
    Directory_2 = '/home/roahmlab/Desktop/rover/offline_lqr/ref_traj_circle_zehui.mat';

    % No.1, eight shape
    % No.2, circle
    switch name_rover
        case 1
            load(Directory_1);
            num = size(ref_traj(:, 1));
            posi_local = [ref_traj(:, 1), ref_traj(:, 2), h/2.*ones(num)];
    
            % when rover points zhenyu's desk
            R = [1 0 0; 0 -1 0; 0 0 1];
%             R = [0 -1 0; 1 0 0; 0 0 1];
    
            posi_global = R * posi_local';
            posi_global = posi_global + repmat(position_ini, 1, num(1));
    
            time_traj = 0 : 0.05 : 0.05*(num(1)-1);
                   
        case 2
            load(Directory_2);
            
            num_idx_delay = 50 ;
            ref_traj = [repmat(ref_traj(1, :), num_idx_delay, 1); ref_traj] ;
            
            stop_idx = 115 + num_idx_delay;
            
            num = size(ref_traj(1:stop_idx, 1));
            posi_local = [ref_traj(1:stop_idx, 1), ref_traj(1:stop_idx, 2), h/2.*ones(num)];
            num = size(posi_local);
            
            % when rover points zhenyu's desk
            R = [1 0 0; 0 -1 0; 0 0 1];
    
            posi_global = R * posi_local';
            posi_global = posi_global + repmat(position_ini, 1, num(1));
            
            num_stop_duration_after_idx = 1000 ;
            
            posi_global = [posi_global, repmat(posi_global(:, end), 1, num_stop_duration_after_idx)] ;
            
            num = size(posi_global);
            time_traj = 0 : 0.05 : 0.05*(num(2)-1);
            
        case 3
            load(Directory_2);
            
            num = size(ref_traj(:, 1));
            posi_local = [ref_traj(:, 1), ref_traj(:, 2), h/2.*ones(num)];

            
            % when rover points zhenyu's desk
            R = [1 0 0; 0 -1 0; 0 0 1];
    
            posi_global = R * posi_local';
            posi_global = posi_global + repmat(position_ini, 1, num(1));
            
%             num_idx_delay = 100 ;
            num_idx_delay = 100 ;
            posi_global = [repmat(posi_global(:, 1), 1, num_idx_delay), posi_global] ;
            num = size(posi_global);
            time_traj = 0 : 0.05 : 0.05*(num(2)-1);
    end

