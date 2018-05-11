% 1 = sphere
% 2 = box
% 3 = capsule
type = 1;

%% parameters
z_height = 5.501;    % drop height 
number_one_dim = 8;
gap = 1.1;
perturb = 0.001;

%% code generation
text = fileread('xml-template/ball_head.txt');
text = strcat(text, sprintf('\\n'));

if type == 1
    % sphere
    output = sprintf('sphere%d.xml', number_one_dim^3);
    
    % size
    radius_ = 0.5;
    
    % inertial
    mass = 1500;
    inertia = ones(1, 3) * 0.4 * mass * radius_^2;
    
    % object geometry
    text = strcat(text, ...
        sprintf('\t\t\t<geom type="sphere" material="geom" rgba=".9 .1 .1 1" size="%f"/>\\n', radius_));
    text = strcat(text, fileread('xml-template/ball_body.txt'));
    text = strcat(text, sprintf('\\n'));
elseif type == 2
    % box
    warning('inertia not implemented!')
    output = sprintf('box%d.xml', number_one_dim^3);
    
    % size
    length_x = 0.8;
    length_y = 0.4;
    length_z = 0.2;
    
    % inertial
%     mass = 170;
    
    % object geometry
    text = strcat(text, ...
        sprintf('\t\t\t<geom type="box" material="geom" rgba=".9 .1 .1 1" size="%f %f %f"/>\\n', ...
        length_x * 0.5, length_y * 0.5, length_z * 0.5));
    text = strcat(text, fileread('xml-template/ball_body.txt'));
    text = strcat(text, sprintf('\\n'));
elseif type == 3
    % capsule
    warning('inertia not implemented!')
    output = sprintf('capsule%d.xml', number_one_dim^3);
    
    % size
    radius_ = 0.2;
    height = 0.6;
    
    % inertial
%     mass = 300;
    
    % object geometry
    text = strcat(text, ...
        sprintf('\t\t\t<geom type="capsule" material="geom" rgba=".9 .1 .1 1" size="%f %f"/>\\n', ...
        radius_, height * 0.5));
    text = strcat(text, fileread('xml-template/ball_body.txt'));
    text = strcat(text, sprintf('\\n'));
end

% 1000 objects
for i = 1:number_one_dim
    for j = 1:number_one_dim
        for k = 1:number_one_dim
            position = [i*gap+rand()*perturb, ...
                j*gap+rand()*perturb, ...
                k*gap+rand()*perturb + z_height];
            quat = quatnormalize([randminusonetoone(), ...
                randminusonetoone(), ...
                randminusonetoone(), ...
                randminusonetoone()]);
            
            text = strcat(text, ...
                sprintf('\t\t<body pos="%f %f %f" quat="%f %f %f %f"> ', ...
                position(1), position(2), position(3), quat(1), quat(2), quat(3), quat(4)));
            text = strcat(text, ...
                sprintf('<inertial pos="0 0 0" mass="%f" diaginertia="%f %f %f"/> <freejoint/> <geom class="object5"/> </body> \\n', ...
                mass, inertia(1), inertia(2), inertia(3)));
        end
    end
end

text = strcat(text, fileread('xml-template/ball_tail.txt'));

%% save file
fid = fopen(strcat('output/', sprintf(output)),'wt');
fprintf(fid, text);
fclose(fid);

%% local functions
function n = randminusonetoone()
n = (rand() - 1/2)*2;
end