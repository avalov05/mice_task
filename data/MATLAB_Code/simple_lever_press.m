%% Constants

ngraphs = 4;
folder = 'D140m44';     


%% Load Data
                     
files = dir(fullfile('../', folder, '*.csv'));         
filePaths = fullfile({files.folder}, {files.name});    
disp(filePaths)

last_time = 0;
combined_table = [];
session_times = [];

for k = 1:length(files)
    table = readtable(filePaths{k});
    disp(filePaths{k})
    table{:,1} = table{:,1} + last_time;
    disp(last_time)
    last_time = table{end, 1};
    session_times = [session_times; last_time];
    combined_table = [combined_table; table];
end

combined_table_stacked = [];

for k = 1:length(files)
    table = readtable(filePaths{k});
    combined_table_stacked = [combined_table_stacked; table];
end

%% Plot Raw
figure(1)
clf

subplot(ngraphs,1,1)
plot(combined_table{:,1}, combined_table{:,5});
for i = 1:length(session_times)
    xline(session_times(i), 'r', 'LineWidth', 1.5);
end
title(folder);
xlabel('Time, s')
ylabel('Reward')

%% Plot by Bins
subplot(ngraphs,1,2)
bin_size = 1.0;
start_time = combined_table{1,1};
end_time = combined_table{end,1};
edges = start_time:bin_size:end_time;

time = combined_table{:,1};
reward = combined_table{:,5};

N = histcounts(time(reward == 1), edges);

bar(edges(1:end-1), smoothdata(N, 'gaussian'), 'histc');
title(folder);
xlabel('Time, 10s')
ylabel('Reward')
%% Plot Angle

subplot(ngraphs,1,3)
plot(combined_table{:,1}, combined_table{:,2});
for i = 1:length(session_times)
    xline(session_times(i), 'r', 'LineWidth', 1.5);
end
title(folder);
xlabel('Time, s')
ylabel('Angle, degrees')

%% activity heat map

subplot(ngraphs,1,4)

bin_size = 1; % bin size in seconds
start_time = combined_table{1,1};
end_time = combined_table{end,1};
edges = start_time:bin_size:end_time;

num_sessions = length(session_times);
heatmap_matrix = zeros(num_sessions, length(edges)-1);

% Exponential decay kernel
decay_tau = 5; 
kernel_length = 5 * decay_tau;
t_kernel = 0:bin_size:kernel_length;
decay_kernel = exp(-t_kernel / decay_tau);

session_start = 0;
for s = 1:num_sessions
    session_end = session_times(s);
    mask = combined_table{:,1} >= session_start & combined_table{:,1} < session_end;
    session_data = combined_table(mask, :);
    
    % Bin the rewards
    rewards = session_data{:,5};
    times = session_data{:,1};
    binned = histcounts(times(rewards==1), edges);
    
    % Apply decay convolution
    binned_smoothed = conv(binned, decay_kernel, 'same');
    heatmap_matrix(s,:) = binned_smoothed;

    session_start = session_end;
end

% Plot heatmap
imagesc(edges(1:end-1), 1:num_sessions, heatmap_matrix)
colormap hot
colorbar
xlabel('Time (s)')
ylabel('Session')
title('Reward Activity Heatmap with Decay')


%% Bout Graph

figure(2)
clf

min_interval = 10; %seconds
min_letgo_time = 2; %seconds

bout_duration = {};
bout_num = {};
run = 1;
bout_count = 1;

cMap = jet(64); % get colormap array without setting global colormap
num_colors = size(cMap, 1);

break_count = 0;
hold_count = 0;

for i = 1:height(combined_table_stacked)
    if combined_table_stacked{i,1} == 0.1 && i ~= 1
        run = run + 1;
        bout_count = 1;
    end
    if combined_table_stacked{i,5} == 1
        hold_count = hold_count + 0.25;
        break_count = 0;
    else
        break_count = break_count + 0.1;
        if hold_count > 10
            if break_count > min_letgo_time
                if length(bout_duration) < run
                    bout_duration{run,1} = [];
                    bout_num{run,1} = [];
                end
                bout_duration{run,1} = [bout_duration{run,1}, hold_count];
                bout_num{run,1} = [bout_num{run,1}, bout_count];
                bout_count = bout_count + 1;
                hold_count = 0;
            end
        else
            if break_count > min_letgo_time
                hold_count = 0;
            end
        end
    end
end
hold on

for i = 1:length(bout_duration)
    color = cMap(randi(num_colors), :);  % get a random RGB color from colormap
    scatter(bout_num{i,1}, bout_duration{i,1}, 36, 'filled', 'MarkerFaceColor', color);
end



hold off
title([folder, " Lever Press Duration"])
xlabel("Bout num")
ylabel("Duration, s")


%% Distribution                    
files = dir(fullfile('../', folder, '*.csv'));
min_letgo_time = 2; % seconds
min_hold_time = 2; % seconds
edges = 2:2:200; % for histogram
hold_count = 0;
general_count = 0;
percent_mean = zeros(2, length(files));
file_count = 0;

for f = 1:length(files)
    filepath = fullfile(files(f).folder, files(f).name);
    disp(filepath)
    data = readtable(filepath);
    file_count = file_count + 1;
    
    time = data{:,1};
    reward = data{:,5};
    hold_durations = [];
    
    in_hold = false;
    current_hold_start = 0;
    last_reward_time = 0;
    
    for i = 1:length(reward)
        if reward(i) == 1
            last_reward_time = time(i);
            if ~in_hold
                in_hold = true;
                current_hold_start = time(i);
            end
        else
            if in_hold
                break_duration = time(i) - last_reward_time;
                if break_duration >= min_letgo_time
                    hold_duration = time(i) - current_hold_start;

                    if hold_duration >= min_hold_time
                        hold_durations(end+1) = hold_duration;
                    end
                    
                    in_hold = false;
                end
            end
        end
        if in_hold
            hold_count = hold_count + 1;
        end
        general_count = general_count + 1;
    end
    
    %check if still holding at end of session
    if in_hold
        final_hold = time(end) - current_hold_start;
        if final_hold >= min_hold_time
            hold_durations(end+1) = final_hold;
        end
    end

   %plot
    if ~isempty(hold_durations)
        figure;
        percent = hold_count/general_count*100;
        disp(percent);
        percent_mean(1,file_count) = percent;
        mean_v = mean(hold_durations);
        disp(mean_v)
        percent_mean(2,file_count) = mean_v;
        
        histogram(hold_durations, edges);
        xlabel('Seconds held (binned by 10s)');
        ylabel('Number of occurrences');
        
        tokens = regexp(files(f).name, '\d{8}', 'match');
        if ~isempty(tokens)
            file_date = tokens{1};
            title_str = sprintf('Hold Duration Distribution - %s', file_date);
            
        else
            title_str = 'Hold Duration Distribution';
        end
        title(title_str);
    else
        fprintf('No valid hold durations found in %s\n', files(f).name);
    end
end
subplot(2,1,1)
plot(1:length(files),percent_mean(1,:));
xlabel('Session #')
ylabel('Percent (%)')
title('Percent During Hold Per Sesson')
subplot(2,1,2)
plot(1:length(files),percent_mean(2,:));
xlabel('Session #')
ylabel('Mean Hold Time, Seconds')
title('Mean Hold Time Per Session')
