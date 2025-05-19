%% Constants

ngraphs = 3;


%% Load Data
folder = 'D147m50;';                          
files = dir(fullfile('../', folder, '*.csv'));         
filePaths = fullfile({files.folder}, {files.name});    
%disp(filePaths)

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
plot(combined_table{:,1}, combined_table{:,3});
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
reward = combined_table{:,3};

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
    if combined_table_stacked{i,3} == 1
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