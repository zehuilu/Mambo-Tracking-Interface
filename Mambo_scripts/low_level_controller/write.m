clear
clc
close all

Directory = '/home/roahmlab/Mambo_scripts/combo_old/traj_lib' ;
BaseName = 'traj_' ;

idx = 1 ;

p_0 = [-1.62361; 0.6; -0.64318] ;


v_0 = [0; 0; 0] ;
a_0 = [0; 0; 0] ;
v_peak = [0.7; 0.3; 0.2] ;
t_peak = 2.5 ;
t_total = 3.0 ;
dt = 0.1 ;

% t_plan won't use in this demo
t_plan = 3.0 ;

[T,Z] = generate_spline_peak_speed(p_0, v_0, a_0, v_peak, t_plan, t_peak, t_total, dt) ;

flag_change_traj = [1, 0, 0, 0, 0, 0] ;
idx_flag_change_traj = 1 ;
idx_current_traj = 1 ;
traj_ref = [] ;

while idx < 6 
    FileName = [BaseName, num2str(idx_flag_change_traj - 1), '.csv'] ;
    FileDest = fullfile(Directory,FileName) ;
    
    if idx > 1
        if flag_change_traj(idx_flag_change_traj) == 1
            [T,Z] = generate_spline_peak_speed(Z(1:3, idx_current_traj*5+1), Z(4:6, idx_current_traj*5+1), Z(7:9, idx_current_traj*5+1), v_peak, t_plan, t_peak, t_total, dt) ;
            traj = Z(1:6, :) ;
            idx_flag_change_traj = idx_flag_change_traj + 1 ;
            idx_current_traj = 1 ;
            csvwrite(FileDest, [T; traj])
        else
            idx_current_traj = idx_current_traj + 1 ;
        end
    else
        traj = Z(1:6, :) ;
        idx_flag_change_traj = idx_flag_change_traj + 1 ;
        csvwrite(FileDest, [T; traj])
    end
    
    % traj append
    traj_ref = [traj_ref; traj];
    
    disp("The current idx is :")
    disp(idx)
    
    idx = idx + 1 ;
    pause(0.5)

end

size_traj = size(traj_ref) ;

time = [0:1:size_traj(2)-1] * dt ;

% plot
figure(1)
axis equal
plot3(traj_ref(3, :), traj_ref(1, :), traj_ref(2, :))

figure(2)
subplot(3,1,1)
plot(time, traj_ref(1, :))
legend("px")

subplot(3,1,2)
plot(time, traj_ref(2, :))
legend("py")

subplot(3,1,3)
plot(time, traj_ref(3, :))
legend("pz")






