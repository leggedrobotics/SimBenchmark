format long

% TODO dt_array from sh script
% TODO analytic solution for different F

%% path
% lib path
addpath(genpath('./lib/yamlmatlab'))

% data path
dir_path = '../../data/rolling-erp=0-dir=0/';

% yaml path 
yaml_path = '../rolling.yaml';

% plot path
mkdir(strcat(dir_path, 'plots'));

%% options
save_subplots = true;
plot_bullet = true;
plot_ode = true;
plot_mujoco = true;
plot_rai = true;

sims = {};

if plot_bullet
    sims{end+1} = 'bullet';
    bullet_solvers = {'seqImp', 'nncg', 'pgs', 'dantzig'}; % 'lemke',
end

if plot_ode
    sims{end+1} = 'ode';
    ode_solvers = {'std', 'quick'};
end

if plot_mujoco
    sims{end+1} = 'mujoco';
    mujoco_solvers = {'pgs', 'cg', 'newton'};
end

if plot_rai
    sims{end+1} = 'rai';
end

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
disp(sims)
disp('==============================')

%% constants
yaml_data = yaml.ReadYaml(yaml_path);

const = yaml_data.constant;
const.mu1 = const.mu_box * const.mu_ground;
const.mu2 = const.mu_ball * const.mu_box;
const.F_xy = yaml_data.options.force_direction; % true for xy, false for y

simTime = const.T;

%% variables
bullet_errors = zeros(length(bullet_solvers), length(dt_array), 2);
ode_errors = zeros(length(ode_solvers), length(dt_array), 2);
mujoco_errors = zeros(length(mujoco_solvers), length(dt_array), 2);
rai_errors = zeros(1, length(dt_array), 2);

bullet_sum_errors = zeros(length(bullet_solvers), length(dt_array), 2);
ode_sum_errors = zeros(length(ode_solvers), length(dt_array), 2);
mujoco_sum_errors = zeros(length(mujoco_solvers), length(dt_array), 2);
rai_sum_errors = zeros(1, length(dt_array), 2);

bullet_timer = zeros(length(bullet_solvers), length(dt_array));
ode_timer = zeros(length(ode_solvers), length(dt_array));
mujoco_timer = zeros(length(mujoco_solvers), length(dt_array));
rai_timer = zeros(1, length(dt_array));


%% main loop
for simid = 1:length(sims)
    sim = sims{simid};
    switch(sim)
        case "bullet"
            for solverid = 1:length(bullet_solvers)
                solver = bullet_solvers{solverid};
                
                for dtid = 1:length(dt_array)
                    dt = dt_array{dtid};
                    
                    if save_subplots
                        plot_ball_vel(dir_path, sim, solver, dt);
                        plot_box_vel(dir_path, sim, solver, dt);
                    end
                    
                    ball_e = ball_error(dir_path, sim, solver, dt, const, save_subplots);
                    box_e = box_error(dir_path, sim, solver, dt, const, save_subplots);
                    
                    bullet_errors(solverid, dtid, 1) = ball_e(end);
                    bullet_errors(solverid, dtid, 2) = box_e(end);
                    
                    bullet_sum_errors(solverid, dtid, 1) = mean(ball_e);
                    bullet_sum_errors(solverid, dtid, 2) = mean(box_e);
                    
                    bullet_timer(solverid, dtid) = timer_value(dir_path, sim, solver, dt, const.T);
                end
            end
        case "ode"
            for solverid = 1:length(ode_solvers)
                solver = ode_solvers{solverid};
                
                for dtid = 1:length(dt_array)
                    dt = dt_array{dtid};
                    
                    if save_subplots
                        plot_ball_vel(dir_path, sim, solver, dt);
                        plot_box_vel(dir_path, sim, solver, dt);
                    end
                    
                    ball_e = ball_error(dir_path, sim, solver, dt, const, save_subplots);
                    box_e = box_error(dir_path, sim, solver, dt, const, save_subplots);
                    
                    ode_errors(solverid, dtid, 1) = ball_e(end);
                    ode_errors(solverid, dtid, 2) = box_e(end);
                    
                    ode_sum_errors(solverid, dtid, 1) = mean(ball_e);
                    ode_sum_errors(solverid, dtid, 2) = mean(box_e);
                    
                    ode_timer(solverid, dtid) = timer_value(dir_path, sim, solver, dt, const.T);
                end
                
            end
        case "mujoco"
            for solverid = 1:length(mujoco_solvers)
                solver = mujoco_solvers{solverid};
                
                for dtid = 1:length(dt_array)
                    dt = dt_array{dtid};
                    
                    if save_subplots
                        plot_ball_vel(dir_path, sim, solver, dt);
                        plot_box_vel(dir_path, sim, solver, dt);
                    end
                    
                    ball_e = ball_error(dir_path, sim, solver, dt, const, save_subplots);
                    box_e = box_error(dir_path, sim, solver, dt, const, save_subplots);
                    
                    mujoco_errors(solverid, dtid, 1) = ball_e(end);
                    mujoco_errors(solverid, dtid, 2) = box_e(end);
                    
                    mujoco_sum_errors(solverid, dtid, 1) = mean(ball_e);
                    mujoco_sum_errors(solverid, dtid, 2) = mean(box_e);
                    
                    mujoco_timer(solverid, dtid) = timer_value(dir_path, sim, solver, dt, const.T);
                end
            end
        case "rai"
            for dtid = 1:length(dt_array)
                dt = dt_array{dtid};
                
                if save_subplots
                    plot_ball_vel(dir_path, sim, '.', dt);
                    plot_box_vel(dir_path, sim, '.', dt);
                end
                
                ball_e = ball_error(dir_path, sim, '.', dt, const, save_subplots);
                box_e = box_error(dir_path, sim, '.', dt, const, save_subplots);
                
                rai_errors(1, dtid, 1) = ball_e(end);
                rai_errors(1, dtid, 2) = box_e(end);
                
                rai_sum_errors(1, dtid, 1) = mean(ball_e);
                rai_sum_errors(1, dtid, 2) = mean(box_e);
                
                rai_timer(1, dtid) = timer_value(dir_path, sim, '.', dt, const.T);
            end
    end
end

% error plot
disp('plotting error vs timestep...')
plot_error_dt(bullet_sum_errors, ode_sum_errors, mujoco_sum_errors, rai_sum_errors, dt_array)

% error with realtime factor plot
disp('plotting error vs real-time-factor...')
plot_error_realtimefactor(bullet_sum_errors, ode_sum_errors, mujoco_sum_errors, rai_sum_errors, ...
    bullet_timer, ode_timer, mujoco_timer, rai_timer)

disp('plotting is finished.')

%% functions
function plot_error_dt(bullet_sum_errors, ode_sum_errors, mujoco_sum_errors, rai_sum_errors, dt_array)
% ball
h = figure('Name','ball error (cumulative)');
plot(bullet_sum_errors(1,:,1), '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log')
hold on
plot(bullet_sum_errors(2,:,1), '-r*', 'DisplayName', 'btNNCG')
plot(bullet_sum_errors(3,:,1), '-ro', 'DisplayName', 'btPGS')
% plot(bullet_sum_errors(4,:,1), '-rs', 'DisplayName', 'btLemke')
plot(bullet_sum_errors(4,:,1), 'r:', 'DisplayName', 'btDantzig')
plot(ode_sum_errors(1,:,1), 'm-', 'DisplayName', 'odeStd')
plot(ode_sum_errors(2,:,1), 'm--', 'DisplayName', 'odeQuick')
plot(mujoco_sum_errors(1,:,1), 'b-', 'DisplayName', 'mjcPGS')
plot(mujoco_sum_errors(2,:,1), '-b*', 'DisplayName', 'mjcCG')
plot(mujoco_sum_errors(3,:,1), '-bo', 'DisplayName', 'mjcNewton')
plot(rai_sum_errors(1,:,1), 'g-', 'DisplayName', 'rai')
hold off
title('Ball Velocity Error (cumulative)')
xlabel('timestep size')
ylabel('squared error (log scale)')
xticklabels(dt_array);
legend('Location', 'eastoutside');
saveas(h, strcat(dir_path, 'plots/', 'ballerror_dt.png'))

% box
h = figure('Name','box error (cumulative)');
plot(bullet_sum_errors(1,:,2), '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log')
hold on
plot(bullet_sum_errors(2,:,2), '-r*', 'DisplayName', 'btNNCG')
plot(bullet_sum_errors(3,:,2), '-ro', 'DisplayName', 'btPGS')
% plot(bullet_sum_errors(4,:,2), '-rs', 'DisplayName', 'btLemke')
plot(bullet_sum_errors(4,:,2), 'r:', 'DisplayName', 'btDantzig')
plot(ode_sum_errors(1,:,2), 'm-', 'DisplayName', 'odeStd')
plot(ode_sum_errors(2,:,2), 'm--', 'DisplayName', 'odeQuick')
plot(mujoco_sum_errors(1,:,2), 'b-', 'DisplayName', 'mjcPGS')
plot(mujoco_sum_errors(2,:,2), '-b*', 'DisplayName', 'mjcCG')
plot(mujoco_sum_errors(3,:,2), '-bo', 'DisplayName', 'mjcNewton')
plot(rai_sum_errors(1,:,2), 'g-', 'DisplayName', 'rai')
hold off
title('Box Velocity Error (cumulative)')
xlabel('timestep size')
ylabel('squared error (log scale)')
xticklabels(dt_array);
legend('Location', 'eastoutside');
saveas(h, strcat(dir_path, 'plots/', 'boxerror_dt.png'))

end

function plot_error_realtimefactor(bullet_sum_errors, ode_sum_errors, mujoco_sum_errors, rai_sum_errors, ...
    bullet_timer, ode_timer, mujoco_timer, rai_timer)
% ball
h = figure('Name','ball error vs realtime factor (cumulative)');
plot(simTime ./ bullet_timer(1,:), bullet_sum_errors(1,:,1),    '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log', 'XScale', 'log')
% set(gca, 'XScale', 'log')
hold on
plot(simTime ./ bullet_timer(2,:),  bullet_sum_errors(2,:,1),   '-r*', 'DisplayName', 'btNNCG')
plot(simTime ./ bullet_timer(3,:),  bullet_sum_errors(3,:,1),   '-ro', 'DisplayName', 'btPGS')
% plot(simTime ./ bullet_timer(4,:),  bullet_sum_errors(4,:,1),   '-rs', 'DisplayName', 'btLemke')
plot(simTime ./ bullet_timer(4,:),  bullet_sum_errors(4,:,1),   'r:', 'DisplayName', 'btDantzig')
plot(simTime ./ ode_timer(1,:),     ode_sum_errors(1,:,1),      'm-', 'DisplayName', 'odeStd')
plot(simTime ./ ode_timer(2,:),     ode_sum_errors(2,:,1),      'm--', 'DisplayName', 'odeQuick')
plot(simTime ./ mujoco_timer(1,:),  mujoco_sum_errors(1,:,1),   'b-', 'DisplayName', 'mjcPGS')
plot(simTime ./ mujoco_timer(2,:),  mujoco_sum_errors(2,:,1),   '-b*', 'DisplayName', 'mjcCG')
plot(simTime ./ mujoco_timer(3,:),  mujoco_sum_errors(3,:,1),   '-bo', 'DisplayName', 'mjcNewton')
plot(simTime ./ rai_timer(1,:),     rai_sum_errors(1,:,1),      'g-', 'DisplayName', 'rai')
hold off
title('Ball Velocity Error vs Realtime factor (cumulative)')
xlabel('realtime factor')
ylabel('squared error (log scale)')
legend('Location', 'eastoutside');
saveas(h, strcat(dir_path, 'plots/', 'ballerror_realtimefactor.png'))

% box
h = figure('Name','box error vs realtime factor (cumulative)');
plot(simTime ./ bullet_timer(1,:), bullet_sum_errors(1,:,2),    '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log', 'XScale', 'log')
% set(gca, 'XScale', 'log')
hold on
plot(simTime ./ bullet_timer(2,:),  bullet_sum_errors(2,:,2),   '-r*', 'DisplayName', 'btNNCG')
plot(simTime ./ bullet_timer(3,:),  bullet_sum_errors(3,:,2),   '-ro', 'DisplayName', 'btPGS')
% plot(simTime ./ bullet_timer(4,:),  bullet_sum_errors(4,:,2),   '-rs', 'DisplayName', 'btLemke')
plot(simTime ./ bullet_timer(4,:),  bullet_sum_errors(4,:,2),   'r:', 'DisplayName', 'btDantzig')
plot(simTime ./ ode_timer(1,:),     ode_sum_errors(1,:,2),      'm-', 'DisplayName', 'odeStd')
plot(simTime ./ ode_timer(2,:),     ode_sum_errors(2,:,2),      'm--', 'DisplayName', 'odeQuick')
plot(simTime ./ mujoco_timer(1,:),  mujoco_sum_errors(1,:,2),   'b-', 'DisplayName', 'mjcPGS')
plot(simTime ./ mujoco_timer(2,:),  mujoco_sum_errors(2,:,2),   '-b*', 'DisplayName', 'mjcCG')
plot(simTime ./ mujoco_timer(3,:),  mujoco_sum_errors(3,:,2),   '-bo', 'DisplayName', 'mjcNewton')
plot(simTime ./ rai_timer(1,:),     rai_sum_errors(1,:,2),      'g-', 'DisplayName', 'rai')
hold off
title('Box Velocity Error vs Realtime factor (cumulative)')
xlabel('realtime factor')
ylabel('squared error (log scale)')
legend('Location', 'eastoutside');
saveas(h, strcat(dir_path, 'plots/', 'boxerror_realtimefactor.png'))

end

function plot_ball_vel(dir_path, sim, solver, dtstr)
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
saveas(h, strcat(dir_path, 'plots/', sim, '_', solver, '_', dtstr, "_velball.png"))

end

function plot_box_vel(dir_path, sim, solver, dtstr)
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
saveas(h, strcat(dir_path, 'plots/', sim, '_', solver, '_', dtstr, "_velbox.png"))
end

function error = ball_error(dir_path, sim, solver, dtstr, const, save_subplots)

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
    saveas(h, strcat(dir_path, 'plots/', sim, '_', solver, '_', dtstr, "_velballerror.png"))
end

end

function error = box_error(dir_path, sim, solver, dtstr, const, save_subplots)

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
    saveas(h, strcat(dir_path, 'plots/', sim, '_', solver, '_', dtstr, "_velboxerror.png"))
end

end

function time = timer_value(dir_path, sim, solver, dtstr, simTime)

parent_dir = strcat(dir_path, sim, '/', solver, '/');
data_path = strcat(parent_dir, dtstr, "timer.rlog");

text = fileread(char(data_path));
C = strsplit(text);

time = str2num(C{11});
end
