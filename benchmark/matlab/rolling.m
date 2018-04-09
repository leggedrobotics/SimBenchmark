format long

% TODO dt_array from sh script
% TODO analytic solution for different F

%% path
% lib path
addpath(genpath('./lib/yamlmatlab'))

% data path
dir_path = '../../data/rolling-erp=0-dir=0/';
plot_path = strcat(dir_path, 'plots/');

% yaml path
yaml_path = '../rolling.yaml';

% plot path
mkdir(plot_path);

%% options
save_subplots = false;s

% always plot raisim
plot_options = struct(...
    'plot_bullet_seqImp', true, ...
    'plot_bullet_nncg', true, ...
    'plot_bullet_pgs', true, ...
    'plot_bullet_dantzig', true, ...
    'plot_bullet_lemke', false, ...
    'plot_ode_std', true, ...
    'plot_ode_quick', true, ...
    'plot_mujoco_pgs', true, ...
    'plot_mujoco_cg', true, ...
    'plot_mujoco_newton', true ...
    );

dt_array = {'0.000010',...
    '0.000040',...
    '0.000100',...
    '0.000400',...
    '0.001000',...
    '0.004000',...
    '0.010000',...
    '0.040000',...
    '0.100000'};

disp('==============================')
disp('plot sims: ')
[sims, solvers] = print_solvers(plot_options);
disp('data path: ')
fprintf('\t%s\n', dir_path)
disp('plot save path: ')
fprintf('\t%s\n', plot_path)
disp('==============================')

%% constants
yaml_data = yaml.ReadYaml(yaml_path);

const = yaml_data.constant;
const.mu1 = const.mu_box * const.mu_ground;
const.mu2 = const.mu_ball * const.mu_box;
% const.F_xy = yaml_data.options.force_direction; % true for xy, false for y
const.F_xy = false; % true for xy, false for y

simTime = const.T;

%% variables
data_array = {};

%% main loop
for i = 1:length(sims)
    sim = sims{i};
    solver = solvers{i};
    
    % simname, solvername, testoption
    data_ = data(sim, solver, struct());
    data_.data_content.ball_error = zeros(length(dt_array), 1);
    data_.data_content.box_error = zeros(length(dt_array), 1);
    data_.data_content.timer = zeros(length(dt_array), 1);
    data_.data_content.dt = zeros(length(dt_array), 1);
    
    for j = 1:length(dt_array)
        dt = dt_array{j};
        
        if save_subplots
            plot_ball_vel(dir_path, plot_path, sim, solver, dt);
            plot_box_vel(dir_path, plot_path, sim, solver, dt);
        end
        
        ball_e = ball_error(dir_path, plot_path, sim, solver, dt, const, save_subplots);
        box_e = box_error(dir_path, plot_path, sim, solver, dt, const, save_subplots);
        
        % save to data array
        data_.data_content.dt(j) = str2double(dt);
        data_.data_content.ball_error(j) = mean(ball_e);
        data_.data_content.box_error(j) = mean(box_e);
        data_.data_content.timer(j) = timer_value(dir_path, sim, solver, dt, const.T);
    end
    
    data_array{end+1} = data_;
end

%% error plot
% error plot vs dt
disp('plotting error vs timestep...')
plot_error_dt(plot_path, data_array)

% error vs realtime factor plot
disp('plotting error vs real-time-factor...')
plot_error_realtimefactor(plot_path, simTime, data_array)

disp('plotting is finished.')

%% functions
function plot_error_dt(plot_path, data_array)
% ball
h = figure('Name','ball error (cumulative)');
data_ = data_array{1};
plot(data_.data_content.dt, ...
    data_.data_content.ball_error, ...
    data_.mark, 'DisplayName', data_.disp_name)
set(gca, 'YScale', 'log')
hold on
for i = 2:length(data_array)
    data_ = data_array{i};
    plot(data_.data_content.dt, ...
        data_.data_content.ball_error, ...
        data_.mark, 'DisplayName', data_.disp_name)
end
hold off
title('Ball Velocity Error (cumulative)')
xlabel('timestep size')
ylabel('squared error (log scale)')
legend('Location', 'eastoutside');
saveas(h, strcat(plot_path, 'ballerror_dt.png'))

% box
h = figure('Name','box error (cumulative)');
data_ = data_array{1};
plot(data_.data_content.dt, ...
    data_.data_content.ball_error, ...
    data_.mark, 'DisplayName', data_.disp_name)
set(gca, 'YScale', 'log')
hold on
for i = 2:length(data_array)
    data_ = data_array{i};
    plot(data_.data_content.dt, ...
        data_.data_content.ball_error, ...
        data_.mark, 'DisplayName', data_.disp_name)
end
hold off
title('Box Velocity Error (cumulative)')
xlabel('timestep size')
ylabel('squared error (log scale)')
legend('Location', 'eastoutside');
saveas(h, strcat(plot_path, 'boxerror_dt.png'))

end

function plot_error_realtimefactor(plot_path, simTime, data_array)
% ball
h = figure('Name','ball error vs realtime factor (cumulative)');
data_ = data_array{1};
plot(simTime ./ data_.data_content.timer, ...
    data_.data_content.ball_error, ...
    data_.mark, 'DisplayName', data_.disp_name)
set(gca, 'YScale', 'log', 'XScale', 'log')
% set(gca, 'XScale', 'log')
hold on
for i = 2:length(data_array)
    data_ = data_array{i};
    plot(simTime ./ data_.data_content.timer, ...
        data_.data_content.ball_error, ...
        data_.mark, 'DisplayName', data_.disp_name)
end
hold off
title('Ball Velocity Error vs Realtime factor (cumulative)')
xlabel('realtime factor')
ylabel('squared error (log scale)')
legend('Location', 'eastoutside');
saveas(h, strcat(plot_path, 'ballerror_realtimefactor.png'))

% box
h = figure('Name','box error vs realtime factor (cumulative)');
data_ = data_array{1};
plot(simTime ./ data_.data_content.timer, ...
    data_.data_content.box_error, ...
    data_.mark, 'DisplayName', data_.disp_name)
set(gca, 'YScale', 'log', 'XScale', 'log')
% set(gca, 'XScale', 'log')
hold on
for i = 2:length(data_array)
    data_ = data_array{i};
    plot(simTime ./ data_.data_content.timer, ...
        data_.data_content.box_error, ...
        data_.mark, 'DisplayName', data_.disp_name)
end
hold off
title('Box Velocity Error vs Realtime factor (cumulative)')
xlabel('realtime factor')
ylabel('squared error (log scale)')
legend('Location', 'eastoutside');
saveas(h, strcat(plot_path, 'boxerror_realtimefactor.png'))

end

function plot_ball_vel(dir_path, plot_path, sim, solver, dtstr)
parent_dir = strcat(dir_path, sim, "/", solver, "/");
data_path = strcat(parent_dir, dtstr, "_velball.rlog");
data = dlmread(data_path,'',4,0);

h = figure('Name','ball velocity');
set(h, 'Visible', 'off');
plot(data(:, 1))
hold on
plot(data(:, 2))
plot(data(:, 3))
hold off
legend('vx', 'vy', 'vz')
title(strcat(sim, ' ', solver, ' ', dtstr))
saveas(h, strcat(plot_path, sim, '_', solver, '_', dtstr, "_velball.png"))

end

function plot_box_vel(dir_path, plot_path, sim, solver, dtstr)
parent_dir = strcat(dir_path, sim, "/", solver, "/");
data_path = strcat(parent_dir, dtstr, "_velbox.rlog");
data = dlmread(data_path,'',4,0);

h = figure('Name','box velocity');
set(h, 'Visible', 'off');
plot(data(:, 1))
hold on
plot(data(:, 2))
plot(data(:, 3))
hold off
legend('vx', 'vy', 'vz')
title(strcat(sim, ' ', solver, ' ', dtstr))
saveas(h, strcat(plot_path, sim, '_', solver, '_', dtstr, "_velbox.png"))
end

function error = ball_error(dir_path, plot_path, sim, solver, dtstr, const, save_subplots)

% analytical solution
g = -const.g;
m = const.m;
n = const.n;
M = const.M;
F = const.F;
mu1 = const.mu1;
mu2 = const.mu2;
simTime = const.T;

f1 = double(mu1 * (M + n * m)  * g);
f2 = double(1 / M * (150 - f1) / (3.5 / m + 25 / M));
a1 = double((F - f1 - n * f2) / M);
a2 = double(f2 / m);

dt = str2num(dtstr);
t = double(0:int32(simTime/dt - 1)) * dt;
v = a2 * double(t);

if const.F_xy
    v = [v' / sqrt(2), v' / sqrt(2), zeros(length(v), 1)];
else
    v = [zeros(length(v), 1), v', zeros(length(v), 1)];
end

% error
parent_dir = strcat(dir_path, sim, "/", solver, "/");
data_path = strcat(parent_dir, dtstr, "_velball.rlog");
data = dlmread(data_path,'',4,0);

if eq(size(v), size(data))
    error = sum((v - data).^2, 2);
    
    % error plots
    if save_subplots
        h = figure('Name','ball errors');
        set(h, 'Visible', 'off');
        plot(error)
        hold on
        plot((v(:,1) - data(:,1)).^2)
        plot((v(:,2) - data(:,2)).^2)
        plot((v(:,3) - data(:,3)).^2)
        hold off
        title(strcat(sim, ' ', solver, ' ', dtstr))
        legend('sum', 'x error sq', 'y error sq', 'z error sq')
        saveas(h, strcat(plot_path, sim, '_', solver, '_', dtstr, "_velballerror.png"))
    end
else
    % data size differs with analytical solution size
    error = nan;
    warning(['data size neq with solution size for ', sim, '-', solver])
end

end

function error = box_error(dir_path, plot_path, sim, solver, dtstr, const, save_subplots)

% analytical solution
g = -const.g;
m = const.m;
n = const.n;
M = const.M;
F = const.F;
mu1 = const.mu1;
mu2 = const.mu2;
simTime = const.T;

f1 = double(mu1 * (M + n * m)  * g);
f2 = double(1 / M * (150 - f1) / (3.5 / m + 25 / M));
a1 = double((F - f1 - n * f2) / M);
a2 = double(f2 / m);
dt = str2num(dtstr);
t = double(0:int32(simTime/dt - 1)) * dt;
v = a1 * t;

if const.F_xy
    v = [v' / sqrt(2), v' / sqrt(2), zeros(length(v), 1)];
else
    v = [zeros(length(v), 1), v', zeros(length(v), 1)];
end

% error
parent_dir = strcat(dir_path, sim, "/", solver, "/");
data_path = strcat(parent_dir, dtstr, "_velbox.rlog");
data = dlmread(data_path,'',4,0);

if eq(size(v), size(data))
    error = sum((v - data).^2, 2);
    
    % error plots
    if save_subplots
        h = figure('Name','box errors');
        set(h, 'Visible', 'off');
        plot(error)
        hold on
        plot((v(:,1) - data(:,1)).^2)
        plot((v(:,2) - data(:,2)).^2)
        plot((v(:,3) - data(:,3)).^2)
        hold off
        title(strcat(sim, ' ', solver, ' ', dtstr))
        legend('sum', 'x error sq', 'y error sq', 'z error sq')
        saveas(h, strcat(plot_path, sim, '_', solver, '_', dtstr, "_velboxerror.png"))
    end
else
    % data size differs with analytical solution size
    error = nan;
    warning(['data size neq with solution size for ', sim, '-', solver])
end

end

function time = timer_value(dir_path, sim, solver, dtstr, simTime)

parent_dir = strcat(dir_path, sim, '/', solver, '/');
data_path = strcat(parent_dir, dtstr, "timer.rlog");

text = fileread(char(data_path));
C = strsplit(text);

time = str2num(C{11});
end

function [sims, solvers] = print_solvers(options)

sims = {};
solvers = {};

fprintf('\t-rai\n') % always
sims{end+1} = 'rai';
solvers{end+1} = '';

% bullet
if options.plot_bullet_seqImp ...
        || options.plot_bullet_nncg ...
        || options.plot_bullet_pgs ...
        || options.plot_bullet_dantzig ...
        || options.plot_bullet_lemke
    
    fprintf('\t-bullet\n')
    
    if options.plot_bullet_seqImp
        fprintf('\t\t-seqImp\n')
        sims{end+1} = 'bullet';
        solvers{end+1} = 'seqImp';
    end
    if options.plot_bullet_nncg
        fprintf('\t\t-nncg\n')
        sims{end+1} = 'bullet';
        solvers{end+1} = 'nncg';
    end
    if options.plot_bullet_pgs
        fprintf('\t\t-pgs\n')
        sims{end+1} = 'bullet';
        solvers{end+1} = 'pgs';
    end
    if options.plot_bullet_dantzig
        fprintf('\t\t-dantzig\n')
        sims{end+1} = 'bullet';
        solvers{end+1} = 'dantzig';
    end
    if options.plot_bullet_lemke
        fprintf('\t\t-lemke\n')
        sims{end+1} = 'bullet';
        solvers{end+1} = 'lemke';
    end
end

% ode
if options.plot_ode_std ...
        || options.plot_ode_quick
    fprintf('\t-ode\n')
    
    if options.plot_ode_std
        fprintf('\t\t-std\n')
        sims{end+1} = 'ode';
        solvers{end+1} = 'std';
    end
    if options.plot_ode_quick
        fprintf('\t\t-quick\n')
        sims{end+1} = 'ode';
        solvers{end+1} = 'quick';
    end
end

% mujoco
if options.plot_mujoco_pgs ...
        || options.plot_mujoco_cg ...
        || options.plot_mujoco_newton
    fprintf('\t-mujoco\n')
    
    if options.plot_mujoco_pgs
        fprintf('\t\t-pgs\n')
        sims{end+1} = 'mujoco';
        solvers{end+1} = 'pgs';
    end
    if options.plot_mujoco_cg
        fprintf('\t\t-cg\n')
        sims{end+1} = 'mujoco';
        solvers{end+1} = 'cg';
    end
    if options.plot_mujoco_newton
        fprintf('\t\t-newton\n')
        sims{end+1} = 'mujoco';
        solvers{end+1} = 'newton';
    end
end

end
