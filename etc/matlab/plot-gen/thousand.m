format long

%% path
% lib path
addpath(genpath('../lib/yamlmatlab'))

% data path
data_dir = '../../../data/thousand';
% plot_path = strcat(data_dir, 'plots/');

% yaml path
yaml_path = '../../../benchmark/yaml/thousand.yaml';

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
    'TIMESTEP', ...
    'ERROR', ...
    'TIME' ...
    };

plotSpec = plotspec;

T = cell2table(cell(0, length(entry)));
T.Properties.VariableNames = entry;

% testOptions
for i = 1:length(testOptions)
    
    % erp: 1/0
    testOption = testOptions{i};
    erp = regexp(testOption, 'erp=([0-9])', 'tokens');
    erp = str2num(erp{1}{1});                % 0: false / 1: true
    
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
                
                error = data_values(curr, 'var_error.rlog');   
                time = timer_value(curr);
                
                opt = struct(...
                    'erp', logical(erp), ...
                    'dt', str2double(timestep), ...
                    'sim', sim, ...
                    'solver', solver ...
                    );
                
                data = {...
                    sim, ...
                    solver, ...
                    logical(erp), ...
                    str2double(timestep), ...
                    mean(error), ...
                    time ...
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
writetable(T, 'thousand-log.csv', 'Delimiter', ',', 'QuoteStrings', true)


%% error plot
% plot option
erpN = plotoption;
erpY = plotoption;

% error plot vs dt
disp('plotting error vs timestep...')
plot_error_dt(T, const, plotSpec, false, '-noerp-y', '(No ERP)', erpN);
plot_error_dt(T, const, plotSpec, true, '-erp-y', '(ERP)', erpY);

disp('plotting error vs real-time-factor...')
plot_error_speed(T, const, plotSpec, false, '-noerp-y', '(No ERP)', erpN);
plot_error_speed(T, const, plotSpec, true, '-erp-y', '(ERP)', erpY);

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

function plot_error_dt(dataTable, const, plotSpec, erpYN, fileName, plotTitle, plotOption)

% filter
dataTable = dataTable(...
    dataTable.ERP == erpYN, :);

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
            data.ERROR, ...
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

function plot_error_speed(dataTable, const, plotSpec, erpYN, fileName, plotTitle, plotOption)

% filter
dataTable = dataTable(...
    dataTable.ERP == erpYN, :);

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
            data.ERROR, ...
            plotspec{1}, ...
            'DisplayName', plotspec{2}, ...
            'color', plotspec{3})
    end
    % end solvers
end
% end sims
hold off
title(['Penetration Error ', plotTitle])
xlabel(sprintf('real time factor \n FAST →'))
ylabel(sprintf('squared error (log scale) \n ACCURATE →'))
legend('Location', 'eastoutside');
saveas(h, strcat('plots/thousand-error-speed', fileName, '.png'))
saveas(h, strcat('plots/thousand-error-speed', fileName, '.eps'), 'epsc')
saveas(h, strcat('plots/thousand-error-speed', fileName, '.fig'), 'fig')

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