format long
% TODO analytic solution for different F

%% path
% lib path
addpath(genpath('../lib/yamlmatlab'))

% data path
data_dir = '../../../data/rolling/';
% plot_path = strcat(data_dir, 'plots/');

% yaml path
yaml_path = '../../../benchmark/yaml/rolling.yaml';

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
const.mu1 = const.raiSim.mu_ground * const.raiSim.mu_box;
const.mu2 = const.raiSim.mu_box * const.raiSim.mu_ball;

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
    '../../../data/rolling/sample.csv', ...
    'Delimiter', ',', ...
    'Format',formatSpec ...
    );

entry = {...
    'SIM', ...
    'SOLVER', ...
    'DETECTOR', ...
    'INTEGRATOR', ...
    'ERP', ...
    'DIRECTION', ...
    'TIMESTEP', ...
    'ERROR', ...
    'TIME' ...
    };
T.Properties.VariableNames = entry;
plotSpec = plotspec;


%% error plot
% plot option
erpNdirY = plotoption;
% erpNdirY.ODESTANDARD = false;   % ODE-std fails

erpNdirXY = plotoption;
erpNdirXY.ODESTANDARDODE = false;  % ODE is pyramid friction cone (and simulation fails)
% erpNdirXY.ODEQUICK = false;     % ODE is pyramid friction cone
% erpNdirXY.DARTDANTZIG = false;  % DART is pyramid friction cone
% erpNdirXY.DARTPGS = false;      % DART is pyramid friction cone
% erpNdirXY.MUJOCOPGSRK4 = false;
% erpNdirXY.MUJOCOCGRK4 = false;
% erpNdirXY.MUJOCONEWTONRK4 = false;

erpYdirY = plotoption;

erpYdirXY = plotoption;
% erpYdirXY.ODESTANDARD = false;  % ODE is pyramid friction cone (and simulation fails)
% erpYdirXY.ODEQUICK = false;     % ODE is pyramid friction cone
% erpYdirXY.DARTDANTZIG = false;  % DART is pyramid friction cone
% erpYdirXY.DARTPGS = false;      % DART is pyramid friction cone

disp('plotting error vs real-time-factor...')
plot_error_speed(T, const, plotSpec, false, false, '-noerp-y', '(No Erp / Y force)', erpNdirY);
plot_error_speed(T, const, plotSpec, false, true, '-noerp-xy', '(No Erp / XY force)', erpNdirXY);
% plot_error_speed(T, const, plotSpec, true, false, '-erp-y', '(Erp / Y force)', erpYdirY);
plot_error_speed(T, const, plotSpec, true, true, '-erp-xy', '(Erp / XY force)', erpYdirXY);

%% bar plot (for min dt)
T2 = T(T.ERP == false & T.DIRECTION == true, :);
dt = min(T2.TIMESTEP);

simTime = const.T;
numIter = simTime / dt;

% filtering
T2 = T2(T2.TIMESTEP == dt, :);
T2 = sortrows(T2, 9);

speed = numIter ./ T2.TIME ./ 1000;

disp('plotting bar graph')
h = figure('Name', 'speed', 'Position', [0, 0, 720, 600]);
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
ylim([0, 25])
saveas(h, strcat('rolling-plots/rolling-speed-bar.png'))
saveas(h, strcat('rolling-plots/rolling-speed-bar.eps'), 'epsc')
saveas(h, strcat('rolling-plots/rolling-speed-bar.fig'), 'fig')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plot_error_speed(dataTable, const, plotSpec, erpYN, dir, fileName, plotTitle, plotOption)

% filter
dataTable = dataTable(...
    dataTable.ERP == erpYN & ...
    dataTable.DIRECTION == dir ...
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
grid on
box on
hold on
for i = 1:length(sims)
    
    sim = sims(i);
    Tsim = dataTable(dataTable.SIM == categorical(sim), :);
    
    solvers = unique(Tsim.SOLVER);
    
    for j = 1:length(solvers)
        
        solver = solvers(j);
        Tsimsol = Tsim(Tsim.SOLVER == categorical(solver), :);
        
        integrators = unique(Tsimsol.INTEGRATOR);
        
        for k = 1:length(integrators)
            
            integrator = integrators(k);
            
            % e.g. RAIRAI or BULLETNNCG
            name = strcat(cellstr(sim), cellstr(solver), cellstr(integrator));
            
            % check plot option
            if ~getfield(plotOption, char(name))
                continue;
            end
            
            % data
            data = Tsimsol(Tsimsol.INTEGRATOR == categorical(integrator), :);
            data = sortrows(data, 9, 'descend');
            
            % plot
            plotspec = getfield(plotSpec, char(name));
            
            plot(...
                const.T ./ data.TIME, ...
                data.ERROR, ...
                plotspec{1}, ...
                'DisplayName', plotspec{2}, ...
                'color', plotspec{3})
        end
        % end of integrator
    end
    % end solvers
end
% end sims
hold off
title(['Velocity Error ', plotTitle])
xlabel(sprintf('real time factor \n FAST →'))
ylabel(sprintf('squared error (log scale) \n ACCURATE →'))
ylim([1e-13, 1e2])
lgd = legend('Location', 'northeast');
lgd.NumColumns = 2;
saveas(h, strcat('rolling-plots/rolling-error-speed', fileName, '.png'))
saveas(h, strcat('rolling-plots/rolling-error-speed', fileName, '.eps'), 'epsc')
saveas(h, strcat('rolling-plots/rolling-error-speed', fileName, '.fig'), 'fig')

end

function error = vel_error(consts, options, data, isBall)

% analytical solution
g = -consts.g;
m = consts.m;
n = consts.n * consts.n;
M = consts.M;
F = consts.F;
mu1 = consts.mu1;
mu2 = consts.mu2;
simTime = consts.T;
dt = options.dt;

f1 = double(mu1 * (M + n * m)  * g);
f2 = double(1 / M * (150 - f1) / (3.5 / m + 25 / M));
a1 = double((F - f1 - n * f2) / M);
a2 = double(f2 / m);

t = double(0:int32(simTime/dt - 1)) * dt;

if isBall
    v = a2 * double(t);
else
    v = a1 * double(t);
end

if options.dir
    % xy
    v = [v' / sqrt(2), v' / sqrt(2), zeros(length(v), 1)];
else
    % y
    v = [zeros(length(v), 1), v', zeros(length(v), 1)];
end

% error
if eq(size(v), size(data))
    error = sum((v - data).^2, 2);
    
    subplot_dir = strcat('rolling-subplot/', num2str(options.erp), '/', num2str(options.dir), '/');
    
    if (~exist(subplot_dir))
        mkdir(subplot_dir);
    end
    
    % error plots
    %     h = figure('Name','ball errors');
    %     set(h, 'Visible', 'off');
    %     plot(error)
    %     hold on
    %     plot((v(:,1) - data(:,1)).^2)
    %     plot((v(:,2) - data(:,2)).^2)
    %     plot((v(:,3) - data(:,3)).^2)
    %     hold off
    %     title(strcat(options.sim, ' ', options.solver, ' ', num2str(options.dt)))
    %     legend('sum', 'x error sq', 'y error sq', 'z error sq')
    %     saveas(h, strcat(subplot_dir, options.sim, '_', options.solver, '_', num2str(options.dt), ".png"))
elseif abs(size(v, 1) - size(data, 1))
    minidx = min(size(v, 1), size(data, 1));
    error = sum((v(1:minidx, :) - data(1:minidx, :)).^2, 2);
    
    subplot_dir = strcat('rolling-subplot/', num2str(options.erp), '/', num2str(options.dir), '/');
    
    if (~exist(subplot_dir))
        mkdir(subplot_dir);
    end
    
    % error plots
    %     h = figure('Name','ball errors');
    %     set(h, 'Visible', 'off');
    %     plot(error)
    %     hold on
    %     plot((v(1:minidx,1) - data(1:minidx,1)).^2)
    %     plot((v(1:minidx,2) - data(1:minidx,2)).^2)
    %     plot((v(1:minidx,3) - data(1:minidx,3)).^2)
    %     hold off
    %     title(strcat(options.sim, ' ', options.solver, ' ', num2str(options.dt)))
    %     legend('sum', 'x error sq', 'y error sq', 'z error sq')
    %     saveas(h, strcat(subplot_dir, options.sim, '_', options.solver, '_', num2str(options.dt), ".png"))
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