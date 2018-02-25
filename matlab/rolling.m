%% path
dir_path = './data/rolling/';

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

%% plot
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
                    bullet_errors(solverid, dtid, 1) = ball_error(dir_path, sim, solver, dt, simTime);
                    bullet_errors(solverid, dtid, 2) = box_error(dir_path, sim, solver, dt, simTime);
                end
            end
        case "ode"
            for solverid = 1:length(ode_solvers)
                solver = ode_solvers{solverid};
                
                for dtid = 1:length(dt_array)
                    dt = dt_array{dtid};
                    plot_ball_vel(dir_path, sim, solver, dt, simTime);
                    plot_box_vel(dir_path, sim, solver, dt, simTime);
                    ode_errors(solverid, dtid, 1) = ball_error(dir_path, sim, solver, dt, simTime);
                    ode_errors(solverid, dtid, 2) = box_error(dir_path, sim, solver, dt, simTime);
                end
                
            end
        case "mujoco"
            for solverid = 1:length(mujoco_solvers)
                solver = mujoco_solvers{solverid};
                
                for dtid = 1:length(dt_array)
                    dt = dt_array{dtid};
                    plot_ball_vel(dir_path, sim, solver, dt, simTime);
                    plot_box_vel(dir_path, sim, solver, dt, simTime);
                    mujoco_errors(solverid, dtid, 1) = ball_error(dir_path, sim, solver, dt, simTime);
                    mujoco_errors(solverid, dtid, 2) = box_error(dir_path, sim, solver, dt, simTime);
                end
            end
        case "rai"
            for dtid = 1:length(dt_array)
                dt = dt_array{dtid};
                plot_ball_vel(dir_path, sim, '.', dt, simTime);
                plot_box_vel(dir_path, sim, '.', dt, simTime);
                rai_errors(solverid, dtid, 1) = ball_error(dir_path, sim, solver, dt, simTime);
                rai_errors(solverid, dtid, 2) = box_error(dir_path, sim, solver, dt, simTime);
            end
    end
end


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
t = (0:simTime/dt) * dt;
v = a2 * t;
v = [zeros(length(v), 1), v', zeros(length(v), 1)];

% error
parent_dir = strcat(dir_path, sim, "/", solver, "/");
data_path = strcat(parent_dir, dtstr, "_velball.rlog");
data = dlmread(data_path,'',4,0);
error = sum((v(end,:) - data(end,:)).^2);
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
t = (0:simTime/dt) * dt;
v = a1 * t;
v = [zeros(length(v), 1), v', zeros(length(v), 1)];

% error
parent_dir = strcat(dir_path, sim, "/", solver, "/");
data_path = strcat(parent_dir, dtstr, "_velbox.rlog");
data = dlmread(data_path,'',4,0);
error = sum((v(end,:) - data(end,:)).^2);
end