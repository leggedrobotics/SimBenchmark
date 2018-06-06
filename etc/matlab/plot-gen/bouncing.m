format long

%% path
% lib path
addpath(genpath('../lib/yamlmatlab'))

% data path
data_dir = '../../../data/bouncing';
% plot_path = strcat(data_dir, 'plots/');

% yaml path
yaml_path = '../../../benchmark/yaml/bouncing.yaml';

% plot path
% mkdir(plot_path);

%% options
save_subplots = false;

disp('===================================================================')
disp('data path: ')
fprintf('\t%s\n', data_dir)
disp('===================================================================')

%% constants and variables
% constant
yaml_data = yaml.ReadYaml(yaml_path);
const = yaml_data.constant;

% variables
curr = data_dir;

dinfo = dir(curr);
dirflags = [dinfo.isdir] & [~ ismember({dinfo.name}, {'.', '..'})];

testOptions = {dinfo.name};
testOptions = {testOptions{:, dirflags}};

%% save data to table

% csv format
formatSpec = '%C%C%C%C%d%d%f%f%f';

% load csv
T = readtable(...
    '../../../data/bouncing/sample.csv', ...
    'Delimiter', ',', ...
    'Format',formatSpec ...
    );

% create table
entry = {...
    'SIM', ...
    'SOLVER', ...
    'DETECTOR', ...
    'INTEGRATOR', ...
    'ERP', ...
    'RESTITUTION', ...
    'TIMESTEP', ...
    'ERROR', ...
    'TIME'
    };
T.Properties.VariableNames = entry;
plotSpec = plotspec;


%% error plot
% plot option
erpNe1 = plotoption;
% erpNe1.BULLETMLCPLEMKEBULLET = false;

erpNe08 = plotoption;
% erpNe08.BULLETMLCPLEMKEBULLET = false;

erpYe1 = plotoption;
erpYe08 = plotoption;

% error plot vs dt
disp('plotting error vs timestep...')
plot_error_speed(T, const, plotSpec, false, 1.0, '-noerp-e=1.0', '(No Erp / e = 1.0)', erpNe1);
% plot_error_speed(T, const, plotSpec, false, 0.8, '-noerp-e=0.8', '(No Erp / e = 0.8)', erpNe08);
plot_error_speed(T, const, plotSpec, true, 1.0, '-erp-e=1.0', '(Erp / e = 1.0)', erpYe1);
% plot_error_speed(T, const, plotSpec, true, 0.8, '-erp-e=0.8', '(Erp / e = 0.8)', erpYe08);

%% bar plot (for min dt)
T2 = T(T.ERP == false & T.RESTITUTION == 1, :);
dt = min(T2.TIMESTEP);

simTime = const.T;
numIter = simTime / dt;

% filtering
T2 = T2(T2.TIMESTEP == dt, :);
T2 = sortrows(T2, 9);

speed = numIter ./ T2.TIME ./ 1000;

disp('plotting bar graph')
h = figure('Name', 'speed', 'Position', [0, 0, 600, 500]);
set(gca, ...
    'YMinorTick', 'off', ...
    'XMinorTick', 'off', ...
    'YMinorGrid', 'off', ...
    'XMinorGrid', 'off')
box on 
% grid on
hold on
for i = 1:size(T2, 1)
    data = T2(i, :);
    
    spec = getfield(plotSpec, strcat(char(data.SIM), char(data.SOLVER), char(data.INTEGRATOR)));
    
    bar(categorical(cellstr(spec{2})), ...
        speed(i), ...
        'FaceColor', spec{3})
end
hold off
title(sprintf('Rolling test speed'))
% numbers on bars
text(1:length(speed), ...
    speed, ...
    num2str(speed, '%0.2f'),...
    'vert', 'bottom', ...
    'horiz','center', ...
    'FontWeight','bold');
ylabel(sprintf('timestep per second (kHz) \n FAST →'))
% ylim([0, 25])
saveas(h, strcat('bouncing-plots/rollingbar.png'))
saveas(h, strcat('bouncing-plots/rollingbar.eps'), 'epsc')
saveas(h, strcat('bouncing-plots/rollingbar.fig'), 'fig')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plot_error_speed(dataTable, const, plotSpec, erpYN, e, fileName, plotTitle, plotOption)

% filter
dataTable = dataTable(...
    dataTable.ERP == erpYN & ...
    dataTable.RESTITUTION == e ...
    ,:);

% ball + box
sims = unique(dataTable.SIM);

h = figure('Name','error','Position', [0, 0, 600, 500]);
set(gca, ...
    'YScale', 'log', ...
    'XScale', 'log', ...
    'Ydir', 'reverse', ...
    'YMinorTick', 'off', ...
    'XMinorTick', 'off', ...
    'YMinorGrid', 'off', ...
    'XMinorGrid', 'off')
box on
grid on
hold on
for i = 1:length(sims)
    
    sim = sims(i);
    Tsim = dataTable(dataTable.SIM == categorical(sim), :);
    
    solvers = unique(Tsim.SOLVER);
    
    for j = 1:length(solvers)
        
        solver = solvers(j);
        
        % e.g. RAIRAI or BULLETNNCG
        name = strcat(cellstr(sim), cellstr(solver), cellstr(sim));
        
        % check plot option
        if ~getfield(plotOption, char(name))
            continue;
        end
        
        % data
        data = Tsim(Tsim.SOLVER == categorical(solver), :);
        data = sortrows(data, 5);
        
        % plot
        plotspec = getfield(plotSpec, char(name));
        
        plot(const.T ./ data.TIME, ...
            data.ERROR, ...
            plotspec{1}, ...
            'DisplayName', plotspec{2}, ...
            'color', plotspec{3})
    end
    % end solvers
end
% end sims
hold off
title(['Energy Error ', plotTitle])
xlabel(sprintf('real time factor \n FAST →'))
ylabel(sprintf('squared error (log scale) \n ACCURATE →'))
legend('Location', 'northeast')
saveas(h, strcat('bouncing-plots/bounce-error-speed', fileName, '.png'))
saveas(h, strcat('bouncing-plots/bounce-error-speed', fileName, '.eps'), 'epsc')
saveas(h, strcat('bouncing-plots/bounce-error-speed', fileName, '.fig'), 'fig')

end

function error = energy_error(consts, options, data)

h0 = consts.H;
R = consts.R;
g = abs(consts.g);
T = consts.T;
m = consts.m;
n = consts.n * consts.n;

erp = options.erp;
e = options.e;
dt = options.dt;

% params
t_tol = 1e-5;

% init
t0 = sqrt(2 * (h0 - R) / g);
E0 = m * g * h0 * n;

h_array = [h0];
E_array = [E0];
t_array = [t0];
t_accum = [t0];

hi = h0;
ti = t0;
while true
    hi = e^2 * (hi - R) + R;
    ti = 2 * sqrt(2 * (hi - R) / g);
    
    h_array = [h_array, hi];
    t_array = [t_array, ti];
    t_accum = [t_accum, t_accum(end) + ti];
    E_array = [E_array, m * g * hi * n];
    
    if ti < t_tol || t_accum(end) > T
        break;
    end
end

v = zeros(length(0:dt:T), 2);
cnt = 1;
for t = 0:dt:T
    idx = find(t_accum >= t, 1);
    v(cnt, :) = [t, E_array(idx)];
    cnt = cnt + 1;
end

% error
if eq(size(v, 2), size(data))
    error = (v(:, 2) - data).^2;
    
    % dir
    energy_dir = strcat('subplot/', num2str(erp), '/', num2str(e), '/energy/');
    error_dir = strcat('subplot/', num2str(erp), '/', num2str(e), '/error/');
    
    if (~exist(energy_dir))
        mkdir(energy_dir);
    end
    if (~exist(error_dir))
        mkdir(error_dir);
    end
    
    % error plots
    h = figure('Name','ball energy');
    set(h, 'Visible', 'off');
    plot(v(:,2))
    hold on 
    plot(data)
    hold off
    ylim([0, 2e3])
    saveas(h, strcat(energy_dir, options.sim, '_', options.solver, '_', num2str(dt), ".png"))
    
    h = figure('Name','ball errors');
    set(h, 'Visible', 'off');
    plot(error)
    saveas(h, strcat(error_dir, options.sim, '_', options.solver, '_', num2str(dt), ".png"))
elseif abs(size(v, 1) - size(data, 1))
    minidx = min(size(v, 1), size(data, 1));
    error = (v(1:minidx, 2) - data(1:minidx)).^2;
    
   % dir
    energy_dir = strcat('subplot/', num2str(erp), '/', num2str(e), '/energy/');
    error_dir = strcat('subplot/', num2str(erp), '/', num2str(e), '/error/');
    
    if (~exist(energy_dir))
        mkdir(energy_dir);
    end
    if (~exist(error_dir))
        mkdir(error_dir);
    end
    
    % error plots
    h = figure('Name','ball energy');
    set(h, 'Visible', 'off');
    plot(v(:,2))
    hold on 
    plot(data)
    hold off
    ylim([0, 2e3])
    saveas(h, strcat(energy_dir, options.sim, '_', options.solver, '_', num2str(dt), ".png"))
    
    h = figure('Name','ball errors');
    set(h, 'Visible', 'off');
    plot(error)
    saveas(h, strcat(error_dir, options.sim, '_', options.solver, '_', num2str(dt), ".png"))
else
    % data size differs with analytical solution size
    error = {nan};
    
    data_size = size(data);
    soln_size = size(v);
    
    warning(['data size differs to solution size for ', ...
        options.sim, '-', options.solver, ...
        ' (data size = ', num2str(data_size(1)), ' x ', num2str(data_size(2))...
        ' / soln size = ', num2str(soln_size(1)), ' x ', num2str(soln_size(2)), ')'])
end

end

% get data value array from log file
function data = data_values(dirPath, fileName)

data = dlmread(strcat(dirPath, '/', fileName),'',4,0);
end

% get timer value from timer log file
function time = timer_value(dirPath)

data_path = strcat(dirPath, '/vartimer.rlog');
string = fileread(data_path);
time = regexp(string, 'min: (\d+\.?\d*)', 'tokens');
time = str2double(time{1}{1});
end