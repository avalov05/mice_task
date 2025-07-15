%==========Parameters==========
folder = 'DAT27m54';
min_press_angle = 2;
min_press_time = 3;
max_letgo_time = 10;
%==============================

%============Setup=============
files = dir(fullfile('../', folder, '*.csv'));
nFiles = length(files);
press_times_total = {};
trial_ends = cell(1, nFiles);
holding_during_trial_percent_ratio = cell(1, nFiles);
previous_count = 0;
last_empty = 0;
file_count = 0;
%==============================

for f = 1:nFiles
    filepath = fullfile(files(f).folder, files(f).name);
    disp(filepath);
    data = readtable(filepath);
    file_count = file_count + 1;

    time = data{:, 1};
    lever = data{:, 3};
    reward = data{:, 5};
    trial = data{:, 6};

    trials = {};
    trial_num = 1;
    last = 0;

    %holding times
    holding_during_trial = 0;
    holding_outside_trial = 0;
    total_trial_time = 0;
    total_outside_time = 0;

    start = 1;
    dt = diff([0; time]);

    for t = 1:length(trial)
        curr_trial = trial(t);
        if curr_trial ~= last
            if last ~= 0
                trials{trial_num} = data(start:t-1, :);
                trial_num = trial_num + 1;
            end
            start = t;
            last = curr_trial;
        end

        if last ~= 0
            if lever(t) >= 2
                holding_during_trial = holding_during_trial + dt(t);
            end
            total_trial_time = total_trial_time + dt(t);
        else
            if lever(t) >= 2
                holding_outside_trial = holding_outside_trial + dt(t);
            end
            total_outside_time = total_outside_time + dt(t);
        end
    end

    %last trial
    if trial_num > 1 && start <= height(data) && trial(end) > 0
        trials{trial_num} = data(start:end, :);
    end

    press_times = cell(1, length(trials));
    empty = 0;

    for t = 1:length(trials)
        trial_data = trials{t};
        start_time = trial_data{1,1};
        reward_col = trial_data{:, 5};

        first_idx = find(reward_col == 1, 1);
        if isempty(first_idx)
            press_times{t} = NaN;
            press_times_total{t+previous_count} = NaN;
            empty = empty + 1;
        else
            press_time = trial_data{first_idx, 1} - start_time;
            press_times{t} = press_time;
            press_times_total{t+previous_count} = press_time;
        end
    end

    previous_count = previous_count + length(trials);
    last_empty = last_empty + empty;
    trial_ends{f} = previous_count - last_empty;

    percent_holding_during = holding_during_trial / total_trial_time;
    percent_holding_outside = holding_outside_trial / total_outside_time;
    holding_during_trial_percent_ratio{f} = percent_holding_during / percent_holding_outside;
end

%=========== Plotting ===========
subplot(4, 1, 1)
press_times_total_numeric = cell2mat(press_times_total);
plot(1:length(press_times_total_numeric), press_times_total_numeric);
title('Raw Press Times (including NaNs)');
ylabel('Time to Press (s)');
xlabel('Trial #');

subplot(4, 1, 2)
clean_press_times = press_times_total_numeric(~isnan(press_times_total_numeric));
plot(1:length(clean_press_times), clean_press_times);
xlines = cell2mat(trial_ends);
xline(xlines, 'r', 'LineWidth', 1.5);
title('Time To Press For Successful Trials Only');
ylabel('Time to Press (s)');
xlabel('Trial #');

subplot(4, 1, 3)
chunks = cell(1, length(xlines));
start_idx = 1;
for i = 1:length(xlines)
    stop_idx = xlines(i);
    chunks{i} = clean_press_times(start_idx:stop_idx);
    start_idx = stop_idx + 1;
end

chunk_averages = cellfun(@mean, chunks);
plot(1:length(chunk_averages), chunk_averages);
set(gca, 'YDir', 'reverse');
title('Average Time to Press per Session');
ylabel('Avg Time to Press (s)');
xlabel('Session#');

subplot(4,1,4)
ratio = cell2mat(holding_during_trial_percent_ratio);
plot(1:length(ratio), ratio);
title('Holding Time Ratio of In-Trial Vs Ourside per Session');
ylabel('Avg Time to Press (s)');
xlabel('Session#');

