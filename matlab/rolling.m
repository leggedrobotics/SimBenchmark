%% path
dir_path = './data/rolling-noerp/';

%% constants
simTime = 4;
sims = {'bullet', 'ode', 'mujoco', 'rai'};
bullet_solvers = {'seqImp', 'nncg', 'pgs', 'lemke', 'dantzig'};
ode_solvers = {'std', 'quick'};
mujoco_solvers = {'pgs', 'cg', 'newton'};

dt_array = {'0.000010',...
    '0.000040',...
    '0.000100',...
    '0.000400',...
    '0.001000',...
    '0.004000',...
    '0.010000',...
    '0.040000',...
    '0.100000'};

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


%% plotbox error (cumulative).png
for simid = 1:length(sims)
    sim = sims{simid};
    switch(sim)
        case "bullet"
            for solverid = 1:length(bullet_solvers)
                solver = bullet_solvers{solverid};
                
                for dtid = 1:length(dt_array)
                    dt = dt_array{dtid};
                    plot_ball_vel(dir_path, sim, solver, dt, simTime);
                    plot_box_vel(dir_path, sim, solver, dt, simTime);
                    
                    ball_e = ball_error(dir_path, sim, solver, dt, simTime);
                    box_e = box_error(dir_path, sim, solver, dt, simTime);
                    
                    bullet_errors(solverid, dtid, 1) = ball_e(end);
                    bullet_errors(solverid, dtid, 2) = box_e(end);
                    
                    bullet_sum_errors(solverid, dtid, 1) = mean(ball_e);
                    bullet_sum_errors(solverid, dtid, 2) = mean(box_e);
                    
                    bullet_timer(solverid, dtid) = timer_value(dir_path, sim, solver, dt, simTime);
                end
            end
        case "ode"
            for solverid = 1:length(ode_solvers)
                solver = ode_solvers{solverid};
                
                for dtid = 1:length(dt_array)
                    dt = dt_array{dtid};
                    plot_ball_vel(dir_path, sim, solver, dt, simTime);
                    plot_box_vel(dir_path, sim, solver, dt, simTime);
                    
                    ball_e = ball_error(dir_path, sim, solver, dt, simTime);
                    box_e = box_error(dir_path, sim, solver, dt, simTime);
                    
                    ode_errors(solverid, dtid, 1) = ball_e(end);
                    ode_errors(solverid, dtid, 2) = box_e(end);
                    
                    ode_sum_errors(solverid, dtid, 1) = mean(ball_e);
                    ode_sum_errors(solverid, dtid, 2) = mean(box_e);
                    
                    ode_timer(solverid, dtid) = timer_value(dir_path, sim, solver, dt, simTime);
                end
                
            end
        case "mujoco"
            for solverid = 1:length(mujoco_solvers)
                solver = mujoco_solvers{solverid};
                
                for dtid = 1:length(dt_array)
                    dt = dt_array{dtid};
                    plot_ball_vel(dir_path, sim, solver, dt, simTime);
                    plot_box_vel(dir_path, sim, solver, dt, simTime);
                    
                    ball_e = ball_error(dir_path, sim, solver, dt, simTime);
                    box_e = box_error(dir_path, sim, solver, dt, simTime);
                    
                    mujoco_errors(solverid, dtid, 1) = ball_e(end);
                    mujoco_errors(solverid, dtid, 2) = box_e(end);
                    
                    mujoco_sum_errors(solverid, dtid, 1) = mean(ball_e);
                    mujoco_sum_errors(solverid, dtid, 2) = mean(box_e);
                    
                    mujoco_timer(solverid, dtid) = timer_value(dir_path, sim, solver, dt, simTime);
                end
            end
        case "rai"
            for dtid = 1:length(dt_array)
                dt = dt_array{dtid};
                plot_ball_vel(dir_path, sim, '.', dt, simTime);
                plot_box_vel(dir_path, sim, '.', dt, simTime);
                
                ball_e = ball_error(dir_path, sim, '.', dt, simTime);
                box_e = box_error(dir_path, sim, '.', dt, simTime);
                
                rai_errors(1, dtid, 1) = ball_e(end);
                rai_errors(1, dtid, 2) = box_e(end);
                
                rai_sum_errors(1, dtid, 1) = mean(ball_e);
                rai_sum_errors(1, dtid, 2) = mean(box_e);
                
                rai_timer(1, dtid) = timer_value(dir_path, sim, '.', dt, simTime);
            end
    end
end

%% error plot
% ball
figure('Name','ball error');
plot(bullet_errors(1,:,1), '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log')
hold on
plot(bullet_errors(2,:,1), '-r*', 'DisplayName', 'btNNCG')
plot(bullet_errors(3,:,1), '-ro', 'DisplayName', 'btPGS')
% plot(bullet_errors(4,:,1), '-rs', 'DisplayName', 'btLemke')
plot(bullet_errors(5,:,1), 'r:', 'DisplayName', 'btDantzig')
plot(ode_errors(1,:,1), 'y-', 'DisplayName', 'odeStd')
plot(ode_errors(2,:,1), 'y--', 'DisplayName', 'odeQuick')
plot(mujoco_errors(1,:,1), 'b-', 'DisplayName', 'mjcPGS')
plot(mujoco_errors(2,:,1), '-b*', 'DisplayName', 'mjcCG')
plot(mujoco_errors(3,:,1), '-bo', 'DisplayName', 'mjcNewton')
plot(rai_errors(1,:,1), 'g-', 'DisplayName', 'rai')
hold off
title('Ball Velocity Error')
xlabel('timestep size')
ylabel('squared error (log scale)')
xticklabels(dt_array);
legend('Location', 'eastoutside');

figure('Name','ball error (cumulative)');
plot(bullet_sum_errors(1,:,1), '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log')
hold on
plot(bullet_sum_errors(2,:,1), '-r*', 'DisplayName', 'btNNCG')
plot(bullet_sum_errors(3,:,1), '-ro', 'DisplayName', 'btPGS')
% plot(bullet_sum_errors(4,:,1), '-rs', 'DisplayName', 'btLemke')
plot(bullet_sum_errors(5,:,1), 'r:', 'DisplayName', 'btDantzig')
plot(ode_sum_errors(1,:,1), 'y-', 'DisplayName', 'odeStd')
plot(ode_sum_errors(2,:,1), 'y--', 'DisplayName', 'odeQuick')
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

% box
figure('Name','box error');
plot(bullet_errors(1,:,2), '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log')
hold on
plot(bullet_errors(2,:,2), '-r*', 'DisplayName', 'btNNCG')
plot(bullet_errors(3,:,2), '-ro', 'DisplayName', 'btPGS')
% plot(bullet_errors(4,:,2), '-rs', 'DisplayName', 'btLemke')
plot(bullet_errors(5,:,2), 'r:', 'DisplayName', 'btDantzig')
plot(ode_errors(1,:,2), 'y-', 'DisplayName', 'odeStd')
plot(ode_errors(2,:,2), 'y--', 'DisplayName', 'odeQuick')
plot(mujoco_errors(1,:,2), 'b-', 'DisplayName', 'mjcPGS')
plot(mujoco_errors(2,:,2), '-b*', 'DisplayName', 'mjcCG')
plot(mujoco_errors(3,:,2), '-bo', 'DisplayName', 'mjcNewton')
plot(rai_errors(1,:,2), 'g-', 'DisplayName', 'rai')
hold off
title('Box Velocity Error')
xlabel('timestep size')
ylabel('squared error (log scale)')
xticklabels(dt_array);
legend('Location', 'eastoutside');

figure('Name','box error (cumulative)');
plot(bullet_sum_errors(1,:,2), '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log')
hold on
plot(bullet_sum_errors(2,:,2), '-r*', 'DisplayName', 'btNNCG')
plot(bullet_sum_errors(3,:,2), '-ro', 'DisplayName', 'btPGS')
% plot(bullet_sum_errors(4,:,2), '-rs', 'DisplayName', 'btLemke')
plot(bullet_sum_errors(5,:,2), 'r:', 'DisplayName', 'btDantzig')
plot(ode_sum_errors(1,:,2), 'y-', 'DisplayName', 'odeStd')
plot(ode_sum_errors(2,:,2), 'y--', 'DisplayName', 'odeQuick')
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

%% error with realtime factor plot
% ball
h = figure('Name','ball error vs realtime factor');
plot(simTime ./ bullet_timer(1,:), bullet_errors(1,:,1),    '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log', 'XScale', 'log')
% set(gca, 'XScale', 'log')
hold on
plot(simTime ./ bullet_timer(2,:),  bullet_errors(2,:,1),   '-r*', 'DisplayName', 'btNNCG')
plot(simTime ./ bullet_timer(3,:),  bullet_errors(3,:,1),   '-ro', 'DisplayName', 'btPGS')
% plot(simTime ./ bullet_timer(4,:),  bullet_errors(4,:,1),   '-rs', 'DisplayName', 'btLemke')
plot(simTime ./ bullet_timer(5,:),  bullet_errors(5,:,1),   'r:', 'DisplayName', 'btDantzig')
plot(simTime ./ ode_timer(1,:),     ode_errors(1,:,1),      'y-', 'DisplayName', 'odeStd')
plot(simTime ./ ode_timer(2,:),     ode_errors(2,:,1),      'y--', 'DisplayName', 'odeQuick')
plot(simTime ./ mujoco_timer(1,:),  mujoco_errors(1,:,1),   'b-', 'DisplayName', 'mjcPGS')
plot(simTime ./ mujoco_timer(2,:),  mujoco_errors(2,:,1),   '-b*', 'DisplayName', 'mjcCG')
plot(simTime ./ mujoco_timer(3,:),  mujoco_errors(3,:,1),   '-bo', 'DisplayName', 'mjcNewton')
plot(simTime ./ rai_timer(1,:),     rai_errors(1,:,1),      'g-', 'DisplayName', 'rai')
hold off
title('Ball Velocity Error vs Realtime factor')
xlabel('realtime factor')
ylabel('squared error (log scale)')
legend('Location', 'eastoutside');

h = figure('Name','ball error vs realtime factor (cumulative)');
plot(simTime ./ bullet_timer(1,:), bullet_sum_errors(1,:,1),    '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log', 'XScale', 'log')
% set(gca, 'XScale', 'log')
hold on
plot(simTime ./ bullet_timer(2,:),  bullet_sum_errors(2,:,1),   '-r*', 'DisplayName', 'btNNCG')
plot(simTime ./ bullet_timer(3,:),  bullet_sum_errors(3,:,1),   '-ro', 'DisplayName', 'btPGS')
% plot(simTime ./ bullet_timer(4,:),  bullet_sum_errors(4,:,1),   '-rs', 'DisplayName', 'btLemke')
plot(simTime ./ bullet_timer(5,:),  bullet_sum_errors(5,:,1),   'r:', 'DisplayName', 'btDantzig')
plot(simTime ./ ode_timer(1,:),     ode_sum_errors(1,:,1),      'y-', 'DisplayName', 'odeStd')
plot(simTime ./ ode_timer(2,:),     ode_sum_errors(2,:,1),      'y--', 'DisplayName', 'odeQuick')
plot(simTime ./ mujoco_timer(1,:),  mujoco_sum_errors(1,:,1),   'b-', 'DisplayName', 'mjcPGS')
plot(simTime ./ mujoco_timer(2,:),  mujoco_sum_errors(2,:,1),   '-b*', 'DisplayName', 'mjcCG')
plot(simTime ./ mujoco_timer(3,:),  mujoco_sum_errors(3,:,1),   '-bo', 'DisplayName', 'mjcNewton')
plot(simTime ./ rai_timer(1,:),     rai_sum_errors(1,:,1),      'g-', 'DisplayName', 'rai')
hold off
title('Ball Velocity Error vs Realtime factor (cumulative)')
xlabel('realtime factor')
ylabel('squared error (log scale)')
legend('Location', 'eastoutside');

% box
h = figure('Name','box error vs realtime factor');
plot(simTime ./ bullet_timer(1,:), bullet_errors(1,:,2),    '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log', 'XScale', 'log')
% set(gca, 'XScale', 'log')
hold on
plot(simTime ./ bullet_timer(2,:),  bullet_errors(2,:,2),   '-r*', 'DisplayName', 'btNNCG')
plot(simTime ./ bullet_timer(3,:),  bullet_errors(3,:,2),   '-ro', 'DisplayName', 'btPGS')
% plot(simTime ./ bullet_timer(4,:),  bullet_errors(4,:,2),   '-rs', 'DisplayName', 'btLemke')
plot(simTime ./ bullet_timer(5,:),  bullet_errors(5,:,2),   'r:', 'DisplayName', 'btDantzig')
plot(simTime ./ ode_timer(1,:),     ode_errors(1,:,2),      'y-', 'DisplayName', 'odeStd')
plot(simTime ./ ode_timer(2,:),     ode_errors(2,:,2),      'y--', 'DisplayName', 'odeQuick')
plot(simTime ./ mujoco_timer(1,:),  mujoco_errors(1,:,2),   'b-', 'DisplayName', 'mjcPGS')
plot(simTime ./ mujoco_timer(2,:),  mujoco_errors(2,:,2),   '-b*', 'DisplayName', 'mjcCG')
plot(simTime ./ mujoco_timer(3,:),  mujoco_errors(3,:,2),   '-bo', 'DisplayName', 'mjcNewton')
plot(simTime ./ rai_timer(1,:),     rai_errors(1,:,2),      'g-', 'DisplayName', 'rai')
hold off
title('Box Velocity Error vs Realtime factor')
xlabel('realtime factor')
ylabel('squared error (log scale)')
legend('Location', 'eastoutside');

h = figure('Name','box error vs realtime factor (cumulative)');
plot(simTime ./ bullet_timer(1,:), bullet_sum_errors(1,:,2),    '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log', 'XScale', 'log')
% set(gca, 'XScale', 'log')
hold on
plot(simTime ./ bullet_timer(2,:),  bullet_sum_errors(2,:,2),   '-r*', 'DisplayName', 'btNNCG')
plot(simTime ./ bullet_timer(3,:),  bullet_sum_errors(3,:,2),   '-ro', 'DisplayName', 'btPGS')
% plot(simTime ./ bullet_timer(4,:),  bullet_sum_errors(4,:,2),   '-rs', 'DisplayName', 'btLemke')
plot(simTime ./ bullet_timer(5,:),  bullet_sum_errors(5,:,2),   'r:', 'DisplayName', 'btDantzig')
plot(simTime ./ ode_timer(1,:),     ode_sum_errors(1,:,2),      'y-', 'DisplayName', 'odeStd')
plot(simTime ./ ode_timer(2,:),     ode_sum_errors(2,:,2),      'y--', 'DisplayName', 'odeQuick')
plot(simTime ./ mujoco_timer(1,:),  mujoco_sum_errors(1,:,2),   'b-', 'DisplayName', 'mjcPGS')
plot(simTime ./ mujoco_timer(2,:),  mujoco_sum_errors(2,:,2),   '-b*', 'DisplayName', 'mjcCG')
plot(simTime ./ mujoco_timer(3,:),  mujoco_sum_errors(3,:,2),   '-bo', 'DisplayName', 'mjcNewton')
plot(simTime ./ rai_timer(1,:),     rai_sum_errors(1,:,2),      'g-', 'DisplayName', 'rai')
hold off
title('Box Velocity Error vs Realtime factor (cumulative)')
xlabel('realtime factor')
ylabel('squared error (log scale)')
legend('Location', 'eastoutside');


%% functions
function plot_ball_vel(dir_path, sim, solver, dtstr, simTime)
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

function plot_box_vel(dir_path, sim, solver, dtstr, simTime)
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

function error = ball_error(dir_path, sim, solver, dtstr, simTime)

% analytical solution
g = 9.8;
m = 1;
n = 25;
M = 10;
F = 150;
mu1 = 0.4;
mu2 = 0.8;

f1 = double(mu1 * (M + n * m)  * g);
f2 = double(1 / M * (150 - f1) / (3.5 / m + 25 / M));
a1 = double((F - f1 - n * f2) / M);
a2 = double(f2 / m);

dt = str2num(dtstr);
t = (0:int32(simTime/dt - 1)) * dt;
v = a2 * double(t);
v = [zeros(length(v), 1), v', zeros(length(v), 1)];

% error
parent_dir = strcat(dir_path, sim, "/", solver, "/");
data_path = strcat(parent_dir, dtstr, "_velball.rlog");
data = dlmread(data_path,'',4,0);
error = sum((v - data).^2, 2);
end

function error = box_error(dir_path, sim, solver, dtstr, simTime)

% analytical solution
g = 9.8;
m = 1;
n = 25;
M = 10;
F = 150;
mu1 = 0.4;
mu2 = 0.8;

f1 = double(mu1 * (M + n * m)  * g);
f2 = double(1 / M * (150 - f1) / (3.5 / m + 25 / M));
a1 = double((F - f1 - n * f2) / M);
a2 = double(f2 / m);

dt = str2num(dtstr);
t = (0:int32(simTime/dt - 1)) * dt;
v = a1 * double(t);
v = [zeros(length(v), 1), v', zeros(length(v), 1)];

% error
parent_dir = strcat(dir_path, sim, "/", solver, "/");
data_path = strcat(parent_dir, dtstr, "_velbox.rlog");
data = dlmread(data_path,'',4,0);
error = sum((v - data).^2, 2);
end

function time = timer_value(dir_path, sim, solver, dtstr, simTime)

parent_dir = strcat(dir_path, sim, '/', solver, '/');
data_path = strcat(parent_dir, dtstr, "timer.rlog");

text = fileread(char(data_path));
C = strsplit(text);

time = str2num(C{11});
end