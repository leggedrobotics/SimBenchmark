format long
% TODO analytic solution for different F

%% path
% lib path
addpath(genpath('../lib/yamlmatlab'))

% data path
data_dir = '../../../data/kapla/';
file_name = 'sample.csv';
% plot_path = strcat(data_dir, 'plots/');

% yaml path
yaml_path = '../../../benchmark/yaml/kapla.yaml';

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
const.dt = 0.01;
% const.mu1 = const.raiSim.mu_ground * const.raiSim.mu_box;
% const.mu2 = const.raiSim.mu_box * const.raiSim.mu_ball;

%% save data to table
% csv format: 
% sim/solver/detector/integrator/erp/maxiter/steps/avg.contact/time
formatSpec = '%C%C%C%C%d%d%d%f%f';

% get table
T = readtable(...
    strcat(data_dir, file_name), ...
    'Delimiter', ',', ...
    'Format',formatSpec ...
    );

plotSpec = plotspec;

entry = {...
    'SIM', ...
    'SOLVER', ...
    'DETECTOR', ...
    'INTEGRATOR', ...
    'ERP',...
    'MAXITER', ...
    'NUMSTEP',...
    'AVGCONTACT',...
    'TIME' ...
    };
T.Properties.VariableNames = entry;


%% error plot
% plot option
plotOption = plotoption;

% error plot vs dt
disp('plotting error vs real-time-factor...')
plot_error_speed(T, const, plotSpec, false, '-noerp', '(No Erp)', plotOption);
plot_error_speed(T, const, plotSpec, true, '-erp', '(Erp)', plotOption);

%% bar plot (for min dt)
% T2 = T;
% dt = min(T2.TIMESTEP);
% 
% simTime = const.T;
% numIter = simTime / dt;
% 
% % filtering
% T2 = T2(T2.TIMESTEP == dt, :);
% T2 = sortrows(T2, 7);
% 
% speed = numIter ./ T2.TIME ./ 1000;
% 
% disp('plotting bar graph')
% h = figure('Name', 'speed', 'Position', [0, 0, 600, 500]);
% set(gca, ...
%     'XTick', {}, ...
%     'YMinorTick', 'off', ...
%     'XMinorTick', 'off', ...
%     'YMinorGrid', 'off', ...
%     'XMinorGrid', 'off')
% box on 
% % grid on
% hold on
% for i = 1:size(T2, 1)
%     data = T2(i, :);
% 
%     spec = getfield(plotSpec, strcat(char(data.SIM), char(data.SOLVER), char(data.INTEGRATOR)));
% 
%     bar(categorical(cellstr(spec{2})), ...
%         speed(i), ...
%         'FaceColor', spec{3})
% end
% hold off
% title('ANYmal Energy')
% % numbers on bars
% text(1:length(speed), ...
%     speed, ...
%     num2str(speed, '%0.2f'),...
%     'vert', 'bottom', ...
%     'horiz','center', ...
%     'FontWeight','bold', ...
%     'FontSize', 8);
% ylabel(sprintf('timestep per second (kHz) \n FAST →'))
% % ylim([0, 190])
% saveas(h, strcat('anymal-energy-plots/anymal-energy-speed-bar.png'))
% saveas(h, strcat('anymal-energy-plots/anymal-energy-speed-bar.eps'), 'epsc')
% saveas(h, strcat('anymal-energy-plots/anymal-energy-speed-bar.fig'), 'fig')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plot_error_speed(dataTable, const, plotSpec, erpYN, fileName, plotTitle, plotOption)

% filter
dataTable = dataTable(...
    dataTable.ERP == erpYN, :);

% ball + box
sims = unique(dataTable.SIM);

h = figure('Name','error','Position', [0, 0, 600, 500]);
hold on
box on
grid on
set(gca, ...
    'YScale', 'log', ...
    'XScale', 'log', ...
    'YMinorTick', 'off', ...
    'XMinorTick', 'off', ...
    'YMinorGrid', 'off', ...
    'XMinorGrid', 'off')
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
            data = sortrows(data, 9);
            
            % plot
            plotspec = getfield(plotSpec, char(name));
            
            plot(...
                const.dt * double(data.NUMSTEP) ./ data.TIME, ...
                data.NUMSTEP, ...
                plotspec{1}, ...
                'DisplayName', plotspec{2}, ...
                'color', plotspec{3})
        end
        % end integrator
    end
    % end solvers
end
% end sims
hold off
title(['Kapla', plotTitle])
xlabel(sprintf('real time factor \n FAST →'))
ylabel(sprintf('number of steps until collapse \n ACCURATE →'))
% xlim([1e-1, 10^5.5])
% ylim([1e-8, 1e6])
lgd = legend('Location', 'northeast');
lgd.NumColumns = 2;
saveas(h, strcat('kapla-plots/kapla-steps-speed', fileName, '.png'))
saveas(h, strcat('kapla-plots/kapla-steps-speed', fileName, '.eps'), 'epsc')
saveas(h, strcat('kapla-plots/kapla-steps-speed', fileName, '.fig'), 'fig')


end