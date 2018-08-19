%% path
% lib path
addpath(genpath('../lib/yamlmatlab'))

% yaml path
yaml_path = '../../../benchmark/yaml/bouncing.yaml';

%% parameters
% option
H = 5.0;               % drop height 
n = 7;

% constants
yaml_data = yaml.ReadYaml(yaml_path);
const = yaml_data.constant;

n = const.n;
mass = const.m;
radius = const.R;
inertia = ones(1, 3) * 0.4 * mass * radius^2;

mu = const.mu_ball;
gap = 2;

%% code generation
text = fileread('xml-template/bounce_head.txt');
text = strcat(text, sprintf('\\n'));

% sphere
output = sprintf('bouncing%d.xml', n^2);

% objects
for i = 1:n
    for j = 1:n
        position = [(i-1)*gap, (j-1)*gap, H];

        text = strcat(text, ...
            sprintf('\t\t<body pos="%f %f %f" quat="1 0 0 0"> \\n', ...
            position(1), position(2), position(3)));
        text = strcat(text, ...
            sprintf('\t\t\t<inertial pos="0 0 0" mass="%f" diaginertia="%f %f %f"/>\\n', mass, inertia(1), inertia(2), inertia(3)));
        text = strcat(text, ...
            sprintf('\t\t\t<freejoint/>\\n'));
        text = strcat(text, ...
            sprintf('\t\t\t<geom type="sphere" size="%f" friction="%f 0 0"/>\\n', radius, mu));
        text = strcat(text, ...
            sprintf('\t\t</body> \\n')); ...
    end
end

text = strcat(text, fileread('xml-template/bounce_tail.txt'));

%% save file
fid = fopen(strcat('output/', sprintf(output)),'wt');
fprintf(fid, text);
fclose(fid);
