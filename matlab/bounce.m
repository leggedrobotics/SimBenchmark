format long

%% path
dir_path = './data/bounce-erp/';

%% constants
sims = {'bullet', 'ode', 'rai'}; % 'mujoco'
bullet_solvers = {'seqImp', 'nncg', 'pgs', 'lemke', 'dantzig'};
ode_solvers = {'std', 'quick'};
% mujoco_solvers = {'pgs', 'cg', 'newton'};

const = struct(...
    'm', 1 , ...
    'M', 10, ...
    'g', 9.8, ...
    'T', 10.0, ...
    'H', 10.0, ...
    'R', 0.5);

simTime = const.T;

dt_array = {'0.000010',...
    '0.000040',...sqrt((H - R) * 2/ g
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
% mujoco_errors = zeros(length(mujoco_solvers), length(dt_array), 2);
rai_errors = zeros(1, length(dt_array), 2);

bullet_timer = zeros(length(bullet_solvers), length(dt_array), 2);
ode_timer = zeros(length(ode_solvers), length(dt_array), 2);
% mujoco_timer = zeros(length(mujoco_solvers), length(dt_array));
rai_timer = zeros(1, length(dt_array), 2);


%% main loop
for simid = 1:length(sims)
    sim = sims{simid};
    switch(sim)
        case "bullet"
            for solverid = 1:length(bullet_solvers)
                solver = bullet_solvers{solverid};

                for dtid = 1:length(dt_array)
                    dt = dt_array{dtid};
                    
                    % restitution = 1
%                     plot_ball_energy(dir_path, sim, solver, dt, '1.000000', const);
                    ball_e = ball_error(dir_path, sim, solver, dt, '1.000000', const);
                    bullet_errors(solverid, dtid, 1) = mean(ball_e);
                    bullet_timer(solverid, dtid, 1) = timer_value(dir_path, sim, solver, dt, '1.000000', const);
                    
                    % restitution = 0.7
%                     plot_ball_energy(dir_path, sim, solver, dt, '0.700000', const);
                    ball_e = ball_error(dir_path, sim, solver, dt, '0.700000', const);
                    bullet_errors(solverid, dtid, 2) = mean(ball_e);
                    bullet_timer(solverid, dtid, 2) = timer_value(dir_path, sim, solver, dt, '0.700000', const);
                end
            end
        case "ode"
            for solverid = 1:length(ode_solvers)
                solver = ode_solvers{solverid};
                
                for dtid = 1:length(dt_array)
                    dt = dt_array{dtid};
                    
                    % restitution = 1
%                     plot_ball_energy(dir_path, sim, solver, dt, '1.000000', const);
                    ball_e = ball_error(dir_path, sim, solver, dt, '1.000000', const);                  
                    ode_errors(solverid, dtid, 1) = mean(ball_e);
                    ode_timer(solverid, dtid, 1) = timer_value(dir_path, sim, solver, dt, '1.000000', const);
                    
                    % restitution = 0.7
%                     plot_ball_energy(dir_path, sim, solver, dt, '0.700000', const);
                    ball_e = ball_error(dir_path, sim, solver, dt, '0.700000', const);
                    ode_errors(solverid, dtid, 2) = mean(ball_e);
                    ode_timer(solverid, dtid, 2) = timer_value(dir_path, sim, solver, dt, '0.700000', const);
                end
                
            end
%         case "mujoco"
%             for solverid = 1:length(mujoco_solvers)
%                 solver = mujoco_solvers{solverid};
%                 
%                 for dtid = 1:length(dt_array)
%                     dt = dt_array{dtid};
%                     plot_ball_vel(dir_path, sim, solver, dt, simTime);
%                     plot_box_vel(dir_path, sim, solver, dt, simTime);
%                     
%                     ball_e = ball_error(dir_path, sim, solver, dt, simTime);
%                     box_e = box_error(dir_path, sim, solver, dt, simTime);
%                     
%                     mujoco_errors(solverid, dtid, 1) = ball_e(end);
%                     mujoco_errors(solverid, dtid, 2) = box_e(end);
%                     
%                     mujoco_sum_errors(solverid, dtid, 1) = mean(ball_e);
%                     mujoco_sum_errors(solverid, dtid, 2) = mean(box_e);
%                     
%                     mujoco_timer(solverid, dtid) = timer_value(dir_path, sim, solver, dt, simTime);
%                 endsqrt((H - R) * 2/ g
%             end
        case "rai"
      
            for dtid = 1:length(dt_array)
                dt = dt_array{dtid};
                
                % restitution = 1
%                 plot_ball_energy(dir_path, sim, '.', dt, '1.000000', const);
                ball_e = ball_error(dir_path, sim, '.', dt, '1.000000', const);
                rai_errors(1, dtid, 1) = mean(ball_e);
                rai_timer(1, dtid, 1) = timer_value(dir_path, sim, '.', dt, '1.000000', const);
                
                % restitution = 0.7
%                 plot_ball_energy(dir_path, sim, '.', dt, '0.700000', const);
                ball_e = ball_error(dir_path, sim, '.', dt, '0.700000', const);
                rai_errors(1, dtid, 2) = mean(ball_e);
                rai_timer(1, dtid, 2) = timer_value(dir_path, sim, '.', dt, '0.700000', const);
            end
    end
end

%% error plot vs timestep
% res = 1.0
figure('Name','energy error (cumulative) vs timestep, res = 1.0');
plot(6:9, bullet_sum_errors(1,6:9,1), '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log')
hold on
plot(6:9, bullet_sum_errors(2,6:9,1), '-r*', 'DisplayName', 'btNNCG')
plot(6:9, bullet_sum_errors(3,6:9,1), '-ro', 'DisplayName', 'btPGS')
% plot(bullet_sum_errors(4,:,1), '-rs', 'DisplayName', 'btLemke')
plot(6:9, bullet_sum_errors(5,6:9,1), 'r:', 'DisplayName', 'btDantzig')
plot(ode_sum_errors(1,:,1), 'y-', 'DisplayName', 'odeStd')
plot(ode_sum_errors(2,:,1), 'y--', 'DisplayName', 'odeQuick')
plot(rai_sum_errors(1,:,1), 'g-', 'DisplayName', 'rai')
hold off
title('Rest = 1.0, Energy Error (cumulative) vs timestep')
xlabel('timestep size')
ylabel('squared error (log scale)')
xticklabels(dt_array);
legend('Location', 'eastoutside');

% res = 0.49
figure('Name','energy error (cumulative) vs timestep, res = 0.49');
plot(6:9, bullet_sum_errors(1,6:9,2), '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log')
hold on
plot(6:9, bullet_sum_errors(2,6:9,2), '-r*', 'DisplayName', 'btNNCG')
plot(6:9, bullet_sum_errors(3,6:9,2), '-ro', 'DisplayName', 'btPGS')
% plot(bullet_sum_errors(4,:,1), '-rs', 'DisplayName', 'btLemke')
plot(6:9, bullet_sum_errors(5,6:9,2), 'r:', 'DisplayName', 'btDantzig')
plot(ode_sum_errors(1,:,2), 'y-', 'DisplayName', 'odeStd')
plot(ode_sum_errors(2,:,2), 'y--', 'DisplayName', 'odeQuick')
plot(rai_sum_errors(1,:,2), 'g-', 'DisplayName', 'rai')
hold off
title('Rest = 0.49, Energy Error (cumulative) vs timestep')
xlabel('timestep size')
ylabel('squared error (log scale)')
xticklabels(dt_array);
legend('Location', 'eastoutside');

%% error with realtime factor plot
h = figure('Name','energy error vs realtime factor (cumulative)');
plot(simTime ./ bullet_timer(1,6:9,1), bullet_sum_errors(1,6:9,1),    '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log', 'XScale', 'log')
% set(gca, 'XScale', 'log')
hold on
plot(simTime ./ bullet_timer(2,6:9,1),  bullet_sum_errors(2,6:9,1),   '-r*', 'DisplayName', 'btNNCG')
plot(simTime ./ bullet_timer(3,6:9,1),  bullet_sum_errors(3,6:9,1),   '-ro', 'DisplayName', 'btPGS')
% plot(simTime ./ bullet_timer(4,:),  bullet_sum_errors(4,:,1),   '-rs', 'DisplayName', 'btLemke')
plot(simTime ./ bullet_timer(5,6:9,1),  bullet_sum_errors(5,6:9,1),   'r:', 'DisplayName', 'btDantzig')
plot(simTime ./ ode_timer(1,:,1),     ode_sum_errors(1,:,1),      'm-', 'DisplayName', 'odeStd')
plot(simTime ./ ode_timer(2,:,1),     ode_sum_errors(2,:,1),      'm--', 'DisplayName', 'odeQuick')
plot(simTime ./ rai_timer(1,:,1),     rai_sum_errors(1,:,1),      'g-', 'DisplayName', 'rai')
hold off
title('Rest = 1.0, Energy Error (cumulative) vs realtime factor')
xlabel('realtime factor')
ylabel('squared error (log scale)')
legend('Location', 'eastoutside');

h = figure('Name','energy error vs realtime factor (cumulative)');
plot(simTime ./ bullet_timer(1,6:9,2), bullet_sum_errors(1,6:9,1),    '-r', 'DisplayName', 'btSeqImp')
set(gca, 'YScale', 'log', 'XScale', 'log')
% set(gca, 'XScale', 'log')
hold on
plot(simTime ./ bullet_timer(2,6:9,2),  bullet_sum_errors(2,6:9,1),   '-r*', 'DisplayName', 'btNNCG')
plot(simTime ./ bullet_timer(3,6:9,2),  bullet_sum_errors(3,6:9,1),   '-ro', 'DisplayName', 'btPGS')
% plot(simTime ./ bullet_timer(4,:),  bullet_sum_errors(4,:,1),   '-rs', 'DisplayName', 'btLemke')
plot(simTime ./ bullet_timer(5,6:9,2),  bullet_sum_errors(5,6:9,1),   'r:', 'DisplayName', 'btDantzig')
plot(simTime ./ ode_timer(1,:,2),     ode_sum_errors(1,:,1),      'm-', 'DisplayName', 'odeStd')
plot(simTime ./ ode_timer(2,:,2),     ode_sum_errors(2,:,1),      'm--', 'DisplayName', 'odeQuick')
plot(simTime ./ rai_timer(1,:,2),     rai_sum_errors(1,:,1),      'g-', 'DisplayName', 'rai')
hold off
title('Rest = 0.49, Energy Error (cumulative) vs realtime factor')
xlabel('realtime factor')
ylabel('squared error (log scale)')
legend('Location', 'eastoutside');


%% functions
function plot_ball_energy(dir_path, sim, solver, dtstr, resstr, const_struct)

parent_dir = strcat(dir_path, sim, "/", solver, "/");
pos_data_path = strcat(parent_dir, dtstr, resstr, "_posball.rlog");
vel_data_path = strcat(parent_dir, dtstr, resstr, "_velball.rlog");

pos_data = dlmread(pos_data_path,'',4,0);
vel_data = dlmread(vel_data_path,'',4,0);

energy = double(const_struct.m * const_struct.g * pos_data(:,3) + ...
    const_struct.m * 0.5 * sum(vel_data .^ 2, 2));

h = figure('Name','ball energy');
set(h, 'Visible', 'off');
plot(energy)
title(strcat(sim, ' ', solver, ' dt=', dtstr, ' res=', resstr))
saveas(h, strcat(dir_path, 'plots/', sim, '_', solver, '_', resstr, '_', dtstr, "_energyball.png"))

end

function time = timer_value(dir_path, sim, solver, dtstr, res, const_struct)

parent_dir = strcat(dir_path, sim, '/', solver, '/');
data_path = strcat(parent_dir, dtstr, res, "timer.rlog");

text = fileread(char(data_path));
C = strsplit(text);

time = str2num(C{11});
end

function error = ball_error(dir_path, sim, solver, dtstr, resstr, const_struct)

% analytical solution
T = const_struct.T;
dt = str2num(dtstr);
e = str2num(resstr);

t = double((0:int32(T/dt - 1))) * dt;
E = zeros(size(t), 1); % energy
 
energyTime = energyTimeArray(const_struct, e * e);

cnt = 1;
for t_i = t
   idx = find(energyTime(:,2) > t_i);
   
   if empty(idx)
       E(cnt, 1) = energyTime(end,1);
   else
       E(cnt, 1) = energyTime(idx(1),1);
   end
   
   cnt = cnt + 1; 
end

% data
parent_dir = strcat(dir_path, sim, "/", solver, "/");
pos_data_path = strcat(parent_dir, dtstr, resstr, "_posball.rlog");
vel_data_path = strcat(parent_dir, dtstr, resstr, "_velball.rlog");

pos_data = dlmread(pos_data_path,'',4,0);
vel_data = dlmread(vel_data_path,'',4,0);

energy = double(m * g * pos_data(:,3) + ...
    m * 0.5 * sum(vel_data .^ 2, 2));

h = figure('Name','ball energy with analyticl solution');
% set(h, 'Visible', 'off');
plot(E)
hold on
plot(energy)
hold off
legend('analytical', 'data')
title(strcat(sim, ' ', solver, ' dt=', dtstr, ' res=', resstr))
saveas(h, strcat(dir_path, 'plots/', sim, '_', solver, '_', resstr, '_', dtstr, "_compare.png"))

error = (energy - E) .^ 2;
end

function output = energyTimeArray(const_struct, e)

height = zeros(100,1);
velocity_sq_before_contact = zeros(100,1);
velocity_sq_after_contact = zeros(100,1);
time = zeros(100,1);
Energy = zeros(100,1);

% constant 
m = const_struct.m;
r = const_struct.R;
g = const_struct.g;

% first peak
height(1) = 10;
% first time to contact 
time(1) = sqrt(2 * (height(1) - r) / g);
% first velocity^2 when contact
velocity_sq_before_contact(1) = 2 * g * (height(1) - r);
velocity_sq_after_contact(1) = velocity_sq_before_contact(1) * e^2;
% energy 
Energy(1) = m * g * height(1);

delta_time = time(1); 

for i = 2:100
 
    % height update
    height(i) = (0.5 * m * velocity_sq_after_contact(i-1) + m * g * r) / (g * m);
    
    % time update
    delta_time = sqrt(velocity_sq_after_contact(i-1)) * 2 / g;
    time(i) = time(i-1) + delta_time;
    
    % velocity update
    velocity_sq_before_contact(i) = velocity_sq_after_contact(i-1);
    velocity_sq_after_contact(i) = velocity_sq_before_contact(i) * e^2;
    
    % energy 
    Energy(i) = m * g * height(i);
end 

output = [Energy, time];

end