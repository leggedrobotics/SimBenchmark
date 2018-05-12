format long
% TODO analytic solution for different F

%% path
% lib path
addpath(genpath('../lib/yamlmatlab'))

% data path
data_dir = '../../../data/bouncing/';
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

% create table
entry = {...
    'SIM', ...
    'SOLVER', ...
    'ERP', ...
    'RESTITUTION', ...
    'TIMESTEP', ...
    'ERROR'
    };

plotSpec = plotspec;

T = cell2table(cell(0, length(entry)));
T.Properties.VariableNames = entry;

% testOptions
for i = 1:length(testOptions)
    
    % erp: 1/0
    % res: double number
    testOption = testOptions{i};
    erp = regexp(testOption, 'erp=([0-9])', 'tokens');
    erp = str2num(erp{1}{1});                % 0: false / 1: true
    res = regexp(testOption, 'res=(\d+\.?\d*)', 'tokens');
    res = str2double(res{1}{1});    % 0: y     / 1: xy
    
    optionDir = strcat(data_dir, '/', testOption);
    sims = strsplit(ls(optionDir));
    sims = sims(1:end-1);
    
    % simulators
    % RAI / BULLET / DART / MUJOCO / ODE
    for j = 1:length(sims)
        sim = sims{j};
        
        simDir = strcat(optionDir, '/', sim);
        solvers = strsplit(ls(simDir));
        solvers = solvers(1:end-1);
        
        % solvers
        % RAI    : RAI
        % BULLET : SEQUENCEIMPULSE, NNCG, ...
        % ODE    : QUICK, STANDARD
        % DART   : DANTZIG, PGS
        % MUJOCO : PGS, CG, NEWTON
        for k = 1:length(solvers)
            
            solver = solvers{k};
            
            solverDir = strcat(simDir, '/', solver);
            timesteps = strsplit(ls(solverDir));
            timesteps = timesteps(1:end-1);
            
            % timesteps
            % 0.00010, 0.000040 ...
            for l = 1:length(timesteps)
                
                timestep = timesteps{l};
                
                curr = strcat(solverDir, '/', timestep);
                energy = data_values(curr, 'var_energy.rlog');
                
                opt = struct(...
                    'erp', logical(erp), ...
                    'e', res, ...
                    'dt', str2double(timestep), ...
                    'sim', sim, ...
                    'solver', solver ...
                    );
                
                meanerror = mean(energy_error(const, opt, energy, false));
                
                data = {...
                    sim, ...
                    solver, ...
                    logical(erp), ...
                    res, ...
                    str2double(timestep), ...
                    meanerror...
                    };
                T = [T; data];
            end
            % end timesteps
        end
        % end solvers
    end
    % end sims
end
% end testOtions

% save table to csv file
writetable(T, 'bouncing-log.csv', 'Delimiter', ',', 'QuoteStrings', true)



%% error plot
% plot option
erpNe1 = plotoption;
erpNe08 = plotoption;
erpYe1 = plotoption;
erpYe08 = plotoption;

% error plot vs dt
disp('plotting error vs timestep...')
plot_error_dt(T, const, plotSpec, false, 1.0, '-noerp-e=1.0', '(No Erp / e = 1.0)', erpNe1);
plot_error_dt(T, const, plotSpec, false, 0.8, '-noerp-e=0.8', '(No Erp / e = 0.8)', erpNe08);
plot_error_dt(T, const, plotSpec, true, 1.0, '-erp-e=1.0', '(Erp / e = 1.0)', erpYe1);
plot_error_dt(T, const, plotSpec, true, 0.8, '-erp-e=0.8', '(Erp / e = 0.8)', erpYe08);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plot_error_dt(dataTable, const, plotSpec, erpYN, e, fileName, plotTitle, plotOption)

% filter
dataTable = dataTable(...
    dataTable.ERP == erpYN & ...
    dataTable.RESTITUTION == e ...
    ,:);

% ball + box
sims = unique(dataTable.SIM);

h = figure('Name','error','Position', [0, 0, 800, 600]);
hold on
set(gca, 'YScale', 'log')
for i = 1:length(sims)
    
    sim = sims(i);
    Tsim = dataTable(dataTable.SIM == categorical(sim), :);
    
    solvers = unique(Tsim.SOLVER);
    
    for j = 1:length(solvers)
        
        solver = solvers(j);
        
        % e.g. RAIRAI or BULLETNNCG
        name = strcat(cellstr(sim), cellstr(solver));
        
        % check plot option
        if ~getfield(plotOption, char(name))
            continue;
        end
        
        % data
        data = Tsim(Tsim.SOLVER == categorical(solver), :);
        data = sortrows(data, 5);
        
        % plot
        plotspec = getfield(plotSpec, char(name));
        
        plot(data.TIMESTEP, ...
            data.ERROR, ...
            plotspec{1}, ...
            'DisplayName', plotspec{2}, ...
            'color', plotspec{3})
    end
    % end solvers
end
% end sims
hold off
title(['Velocity Error ', plotTitle])
xlabel('timestep size')
ylabel('squared error (log scale)')
legend('Location', 'eastoutside');
saveas(h, strcat('plots/bounce-error-dt', fileName, '.png'))
saveas(h, strcat('plots/bounce-error-dt', fileName, '.eps'), 'epsc')
saveas(h, strcat('plots/bounce-error-dt', fileName, '.fig'), 'fig')
% 
% % box
% h = figure('Name','box error','Position', [0, 0, 800, 600]);
% hold on
% set(gca, 'YScale', 'log')
% for i = 1:length(sims)
%     
%     sim = sims(i);
%     Tsim = dataTable(dataTable.SIM == categorical(sim), :);
%     
%     solvers = unique(Tsim.SOLVER);
%     
%     for j = 1:length(solvers)
%         
%         solver = solvers(j);
%         
%         % e.g. RAIRAI or BULLETNNCG
%         name = strcat(cellstr(sim), cellstr(solver));
%         
%         % check plot option
%         if ~getfield(plotOption, char(name))
%             continue;
%         end
%         
%         % data
%         data = Tsim(Tsim.SOLVER == categorical(solver), :);
%         data = sortrows(data, 5);
%         
%         % plot
%         plotspec = getfield(plotSpec, char(name));
%         
%         plot(data.TIMESTEP, ...
%             data.BOXERROR + data.BOXERROR, ...
%             plotspec{1}, ...
%             'DisplayName', plotspec{2}, ...
%             'color', plotspec{3})
%     end
%     % end solvers
% end
% % end sims
% hold off
% title(['Box Velocity Error ', plotTitle])
% xlabel('timestep size')
% ylabel('squared error (log scale)')
% legend('Location', 'eastoutside');
% saveas(h, strcat('plots/boxerror-dt', fileName, '.png'))
% saveas(h, strcat('plots/boxerror-dt', fileName, '.epsc'), 'epsc')
% saveas(h, strcat('plots/boxerror-dt', fileName, '.fig'), 'fig')

end

function error = energy_error(consts, options, data, isBall)

h0 = consts.H;
R = consts.R;
g = abs(consts.g);
T = consts.T;
m = consts.m;

e = options.e;
dt = options.dt;

% params
t_tol = 1e-5;

% init
t0 = sqrt(2 * (h0 - R) / g);
E0 = m * g * h0;

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
    E_array = [E_array, m * g * hi];
    
    if ti < t_tol || t_accum(end) > T
        break;
    end
end

v = zeros(0, 2);
for t = 0:dt:T
    idx = find(t_accum >= t, 1);
    v = [v; [t, E_array(idx)]];
end

% error
if eq(size(v, 2), size(data))
    error = sum((v(:, 2) - data).^2, 2);
    
    % error plots
    h = figure('Name','ball errors');
    set(h, 'Visible', 'off');
    plot(v(:,2))
    hold on 
    plot(data)
    hold off
    saveas(h, strcat('subplot/', options.sim, '_', options.solver, '_', num2str(dt), ".png"))
elseif abs(size(v, 1) - size(data, 1))
    minidx = min(size(v, 1), size(data, 1));
    error = sum((v(1:minidx, 2) - data(1:minidx, :)).^2, 2);
    
    % error plots
    h = figure('Name','ball errors');
    set(h, 'Visible', 'off');
    plot(v(:,2))
    hold on 
    plot(data)
    hold off
    saveas(h, strcat('subplot/', options.sim, '_', options.solver, '_', num2str(dt), ".png"))
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