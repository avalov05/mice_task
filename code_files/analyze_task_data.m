%% Task Data Analysis with Correct Reward Zone Boundaries
% This script properly handles the coordinate system conversion between
% servo angles and lever angles for reward zone visualization

%==========Parameters==========
folder = 'D140m45';  % Change this to your data folder
min_letgo_time = 2; % seconds
min_hold_time = 2; % seconds
min_lever_angle_offset = 10; % degrees for movement
lever_angle_sampling_time = 10; % seconds
%==============================

%============Setup=============
files = dir(fullfile('../', folder, '*.txt'));
file_count = 0;
hold_count = 0;
general_count = 0;
extracted_data = cell(length(files), 8);
%==============================

for f = 1:length(files)
    filepath = fullfile(files(f).folder, files(f).name);
    
    % Skip parameter files
    if contains(files(f).name, '_params')
        continue;
    end
    
    disp(filepath);
    data = readtable(filepath);
    file_count = file_count + 1;
    
    time = data{:,1};
    lever = data{:,2};  % Now correctly lever angle
    servo = data{:,3};  % Now correctly servo angle
    reward = data{:,5};
    
    % Load parameters for this session
    params_file = strrep(filepath, '.txt', '_params.txt');
    if exist(params_file, 'file')
        params_data = readtable(params_file);
        reward_zone_lever_min = params_data{strcmp(params_data{:,1}, 'REWARD_ZONE_LEVER_MIN'), 2};
        reward_zone_lever_max = params_data{strcmp(params_data{:,1}, 'REWARD_ZONE_LEVER_MAX'), 2};
        reward_zone_size = params_data{strcmp(params_data{:,1}, 'REWARD_ZONE_SIZE'), 2};
    else
        % Fallback to default values if params file doesn't exist
        reward_zone_lever_min = 70;  % Default lever angle min
        reward_zone_lever_max = 110; % Default lever angle max
        reward_zone_size = 40;
        warning('Parameter file not found, using default reward zone values');
    end
    
    % Preallocate arrays
    hold_durations = zeros(1, 1000); 
    hold_full_start = zeros(1, 1000); 
    hold_full_stop  = zeros(1, 1000); 
    hold_attempt_start = zeros(1, 1000);
    hold_attempt_stop  = zeros(1, 1000);
    reward_times = zeros(1, 1000);
    dur_idx = 1; full_idx_b = 1; full_idx_e = 1; att_idx_b = 1; att_idx_e = 1; rew_ind = 1;
    
    in_hold = false;
    in_attempt = false;
    current_hold_start = 0;
    last_reward_time = 0;
    last_lever_time = 0;

    min_lever_angle = min_lever_angle_offset + lever(100);
    
    for i = 1:length(reward)
        % --- in reward zone detection ---
        if reward(i) == 1
            reward_times(rew_ind) = time(i);
            rew_ind = rew_ind + 1;
            last_reward_time = time(i);
            if ~in_hold
                in_hold = true;
                hold_full_start(full_idx_b) = time(i);
                full_idx_b = full_idx_b + 1;
                current_hold_start = time(i);
            end
        else
            if in_hold
                break_duration = time(i) - last_reward_time;
                if break_duration >= min_letgo_time
                    hold_duration = time(i) - current_hold_start;
                    if hold_duration >= min_hold_time
                        hold_durations(dur_idx) = hold_duration;
                        dur_idx = dur_idx + 1;
                        hold_full_stop(full_idx_e) = time(i);
                        full_idx_e = full_idx_e + 1;
                    end
                    in_hold = false;
                end
            end
        end

        % --- attempt detection ---
        if lever(i) >= min_lever_angle
            last_lever_time = time(i);
            if ~in_attempt
                in_attempt = true;
                hold_attempt_start(att_idx_b) = time(i);
                att_idx_b = att_idx_b + 1;
            end
        else
            time_since_last_lever_movement = time(i) - last_lever_time;
            if time_since_last_lever_movement >= lever_angle_sampling_time
                if in_attempt
                    hold_attempt_stop(att_idx_e) = time(i);
                    in_attempt = false;
                    att_idx_e = att_idx_e + 1;
                end
            end
        end

        if in_hold
            hold_count = hold_count + 1;
        end
        general_count = general_count + 1;
    end
    
    % trim preallocated arrays
    hold_durations = hold_durations(1:dur_idx-1);
    hold_full_start = hold_full_start(1:full_idx_b-1);
    hold_full_stop  = hold_full_stop(1:full_idx_e-1);
    hold_attempt_start = hold_attempt_start(1:att_idx_b-1);
    hold_attempt_stop  = hold_attempt_stop(1:att_idx_e-1);
    reward_times  = reward_times(1:rew_ind-1);

    % check in case lever held at end
    if in_hold
        final_hold = time(end) - current_hold_start;
        if final_hold >= min_hold_time
            hold_durations(end+1) = final_hold;
            hold_full_stop(end+1) = time(end);
        end
    end
    if in_attempt
        hold_attempt_stop(end+1) = time(end);
    end
    
    % Store extracted data
    extracted_data{f, 1} = hold_durations;
    extracted_data{f, 2} = hold_attempt_start;
    extracted_data{f, 3} = hold_attempt_stop;
    extracted_data{f, 4} = hold_full_start;
    extracted_data{f, 5} = hold_full_stop;
    extracted_data{f, 6} = time;
    extracted_data{f, 7} = lever;
    extracted_data{f, 8} = reward_times;
    extracted_data{f, 9} = reward_zone_lever_min;
    extracted_data{f, 10} = reward_zone_lever_max;
    
    % Plot with correct reward zone
    figure;
    plot(time, lever); hold on;
    for i = 1:length(hold_attempt_start)
        xline(hold_attempt_start(i), 'g', 'LineWidth', 1.5);
    end
    for i = 1:length(hold_attempt_stop)
        xline(hold_attempt_stop(i), 'r', 'LineWidth', 1.5);
    end
    
    % Plot reward zone in lever angle space
    xl = xlim;
    patch([xl(1) xl(2) xl(2) xl(1)], [reward_zone_lever_min reward_zone_lever_min reward_zone_lever_max reward_zone_lever_max], ...
          [0.5 0.7 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    
    title(sprintf('%s - File: %s', folder, files(f).name));
    xlabel('Time, s');
    ylabel('Lever Angle, degrees');
    legend('Lever Angle', 'Attempt Start', 'Attempt Stop', 'Reward Zone', 'Location', 'best');
end

%% Graph Individual Bouts with Correct Reward Zone

file_number = 1;

hold_attempt_start = extracted_data{file_number, 2};
hold_attempt_stop = extracted_data{file_number, 3};
time = extracted_data{file_number, 6};
lever = extracted_data{file_number, 7};
reward_times = extracted_data{file_number, 8};
reward_zone_lever_min = extracted_data{file_number, 9};
reward_zone_lever_max = extracted_data{file_number, 10};

for i = 1:length(hold_attempt_start)
    start_ind = find(time == hold_attempt_start(i));
    stop_ind = find(time == hold_attempt_stop(i));
    bout_time = time(start_ind:stop_ind);
    bout_lever = lever(start_ind:stop_ind);
    reward_times_cut = reward_times(reward_times >= hold_attempt_start(i) & reward_times <= hold_attempt_stop(i));
    
    figure;
    plot(bout_time, bout_lever, 'r');
    set(gca, 'YDir', 'reverse')
    hold on
    dot_indices = arrayfun(@(t)find(time==t,1), reward_times_cut);
    scatter(time(dot_indices), lever(dot_indices), 60, 'r', 'filled');
    
    % Plot reward zone with correct boundaries
    xl = xlim;
    patch([xl(1) xl(2) xl(2) xl(1)], [reward_zone_lever_min reward_zone_lever_min reward_zone_lever_max reward_zone_lever_max], ...
          [0.5 0.7 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    
    title(sprintf('Bout %d - Lever Angle vs Time', i));
    xlabel('Time, s');
    ylabel('Lever Angle, degrees');
    legend('Lever Angle', 'Rewards', 'Reward Zone', 'Location', 'best');
    hold off
end

fprintf('Analysis complete. Processed %d files.\n', file_count); 