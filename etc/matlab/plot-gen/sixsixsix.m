format long

%% path
% lib path
addpath(genpath('../lib/yamlmatlab'))

% data path
data_dir = '../../../data/666/';
file_name = 'sample-elastic.csv';

% yaml path
yaml_path = '../../../benchmark/yaml/666.yaml';

% plot path
% mkdir(plot_path);

%% options
save_subplots = false;

disp('===================================================================')
disp('data path: ')
fprintf('\t%s\n', data_dir)
disp('===================================================================')

%% constants and variables
% const 
yaml_data = yaml.ReadYaml(yaml_path);
const = yaml_data.constant;
const.T = 15;       % TODO should be get from somewhere

% csv format
formatSpec = '%C%C%d%d%f%f%f%f';

T = readtable(...
    strcat(data_dir, file_name), ...
    'Delimiter', ',', ...
    'Format',formatSpec ...
    );

entry = {...
    'SIM', ...
    'SOLVER', ...
    'ERP', ...
    'ELASTIC', ...
    'TIMESTEP', ...
    'PENETRATION', ...
    'ENERGYERROR', ...
    'TIME' ...
    };

T.Properties.VariableNames = entry;
plotSpec = plotspec;

%% error plot
% plot option
erpN = plotoption;
erpY = plotoption;

% error plot vs dt
disp('plotting penetration error vs timestep...')
plot_error_dt(T, const, plotSpec, false, false, '-noerp', '(No ERP)', erpN);
plot_error_dt(T, const, plotSpec, true, false, '-erp', '(ERP)', erpY);

disp('plotting penetration error vs real-time-factor...')
plot_error_speed(T, const, plotSpec, false, false, '-noerp', '(No ERP)', erpN);
plot_error_speed(T, const, plotSpec, true, false, '-erp', '(ERP)', erpY);

% error energy plot (only for elastic collision)
disp('plotting penetration error vs real-time-factor...')
plot_energy_error_speed(T, const, plotSpec, false, '-noerp', '(No ERP)', erpN)
plot_energy_error_speed(T, const, plotSpec, true, '-erp', '(ERP)', erpY)

%% bar plot (for min dt)
% T2 = T(T.ERP == false & T.DIRECTION == true, :);
% dt = min(T2.TIMESTEP);
% 
% simTime = const.T;
% numIter = simTime / dt;
% 
% % filtering
% T2 = T2(T2.TIMESTEP == dt, :);
% T2 = sortrows(T2, 12);
% 
% speed = numIter ./ T2.TIME ./ 1000;
% 
% disp('plotting bar graph')
% h = figure('Name', 'speed', 'Position', [0, 0, 800, 600])
% hold on
% for i = 1:size(T2, 1)
%     data = T2(i, :);
%     
%     spec = getfield(plotSpec, char(strcat(data.SIM, data.SOLVER)));
%     
%     bar(categorical(cellstr(spec{2})), ...
%         speed(i), ...
%         'FaceColor', spec{3})
% end
% hold off
% title(sprintf('Rolling test speed (timestep = %f)', dt))
% % numbers on bars
% text(1:length(speed), ...
%     speed, ...
%     num2str(speed, '%0.2f'),...
%     'vert', 'bottom', ...
%     'horiz','center', ...
%     'FontWeight','bold');
% ylabel(sprintf('timestep per second (kHz) \n FAST →'))
% ylim([0, 16])
% saveas(h, strcat('plots/rollingbar.png'))
% saveas(h, strcat('plots/rollingbar.eps'), 'epsc')
% saveas(h, strcat('plots/rollingbar.fig'), 'fig')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plot_error_dt(dataTable, const, plotSpec, erpYN, elastic, fileName, plotTitle, plotOption)

% filter
dataTable = dataTable(...
    dataTable.ERP == erpYN & dataTable.ELASTIC == elastic, :);

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
        data = sortrows(data, 4);
        
        % plot
        plotspec = getfield(plotSpec, char(name));
        
        plot(data.TIMESTEP, ...
            data.PENETRATION, ...
            plotspec{1}, ...
            'DisplayName', plotspec{2}, ...
            'color', plotspec{3})
    end
    % end solvers
end
% end sims
hold off
title(['Penetration Error ', plotTitle])
xlabel('timestep size')
ylabel('squared error (log scale)')
legend('Location', 'eastoutside');
saveas(h, strcat('plots/thousand-error-dt', fileName, '.png'))
saveas(h, strcat('plots/thousand-error-dt', fileName, '.eps'), 'epsc')
saveas(h, strcat('plots/thousand-error-dt', fileName, '.fig'), 'fig')

end

function plot_error_speed(dataTable, const, plotSpec, erpYN, elastic, fileName, plotTitle, plotOption)

% filter
dataTable = dataTable(...
    dataTable.ERP == erpYN & dataTable.ELASTIC == elastic, :);

% ball + box
sims = unique(dataTable.SIM);

h = figure('Name','error','Position', [0, 0, 800, 600]);
hold on
set(gca, 'YScale', 'log', 'XScale', 'log', 'Ydir', 'reverse')
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
        data = sortrows(data, 4);
        
        % plot
        plotspec = getfield(plotSpec, char(name));
        
        plot(...
            const.T ./ data.TIME, ...
            data.PENETRATION, ...
            plotspec{1}, ...
            'DisplayName', plotspec{2}, ...
            'color', plotspec{3}, ...
            'LineWidth', 1)
    end
    % end solvers
end
% end sims
hold off
title(['Penetration Error ', plotTitle])
xlabel(sprintf('real time factor \n FAST →'))
ylabel(sprintf('squared error (log scale) \n ACCURATE →'))
xlim([1e-2 10^2.5])
legend('Location', 'eastoutside');
saveas(h, strcat('plots/thousand-error-speed', fileName, '.png'))
saveas(h, strcat('plots/thousand-error-speed', fileName, '.eps'), 'epsc')
saveas(h, strcat('plots/thousand-error-speed', fileName, '.fig'), 'fig')

end

function plot_energy_error_speed(dataTable, const, plotSpec, erpYN, fileName, plotTitle, plotOption)

% filter
dataTable = dataTable(...
    dataTable.ERP == erpYN & dataTable.ELASTIC == true, :);

% ball + box
sims = unique(dataTable.SIM);

h = figure('Name','error','Position', [0, 0, 800, 600]);
hold on
set(gca, 'YScale', 'log', 'XScale', 'log', 'Ydir', 'reverse')
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
        data = sortrows(data, 4);
        
        % plot
        plotspec = getfield(plotSpec, char(name));
        
        plot(...
            const.T ./ data.TIME, ...
            data.ENERGYERROR, ...
            plotspec{1}, ...
            'DisplayName', plotspec{2}, ...
            'color', plotspec{3}, ...
            'LineWidth', 1)
    end
    % end solvers
end
% end sims
hold off
title(['Energy Error ', plotTitle])
xlabel(sprintf('real time factor \n FAST →'))
ylabel(sprintf('squared error (log scale) \n ACCURATE →'))
xlim([1e-2 10^2.5])
legend('Location', 'eastoutside');
saveas(h, strcat('plots/thousand-energy-error-speed', fileName, '.png'))
saveas(h, strcat('plots/thousand-energy-error-speed', fileName, '.eps'), 'epsc')
saveas(h, strcat('plots/thousand-energy-error-speed', fileName, '.fig'), 'fig')

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