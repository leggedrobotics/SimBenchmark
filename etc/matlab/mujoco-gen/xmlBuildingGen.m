text = fileread('xml-template/bldg_head.txt');
text = strcat(text, sprintf('\\n'));

% box
output = sprintf('building.xml');

% size
shortLen = 0.05;
longLen = 0.2;
heightLen = 0.1;
% mass = 10;

% num of blocks = numFloor x numBase + numFloor x (numWall x 2 + 1)
numFloor = 12;
numBase = 10;
numWall = numBase / 2;

% object geometry wall
text = strcat(text, ...
    sprintf('\t\t<default class="wall">\\n'));
text = strcat(text, ...
    sprintf('\t\t\t<geom type="box" material="geom" rgba=".9 .9 .9 1" size="%f %f %f"/>\\n', ...
    longLen * 0.5, shortLen * 0.5, heightLen * 0.5));
text = strcat(text, ...
    sprintf('\t\t</default>\\n'));

% object geometry wall
text = strcat(text, ...
    sprintf('\t\t<default class="base">\\n'));
text = strcat(text, ...
    sprintf('\t\t\t<geom type="box" material="geom" rgba=".9 .9 .9 1" size="%f %f %f"/>\\n', ...
    shortLen * 0.5, (longLen + 0.05) * 0.5, heightLen * 0.5));
text = strcat(text, ...
    sprintf('\t\t</default>\\n\t</default>\\n'));

text = strcat(text, fileread('xml-template/bldg_body.txt'));
text = strcat(text, sprintf('\\n'));

% building objects
for i=0:numFloor-1
    % i floor
    for j=0:numBase-1
        % base
        position = [...
            j * longLen, ...
            0, ...
            i * heightLen * 2 + 0.05];
        quat = [1, 0, 0, 0];
        
        text = strcat(text, ...
            sprintf('\t\t<body pos="%f %f %f" quat="%f %f %f %f"> <freejoint/> <geom class="base"/> </body> \\n', ...
            position(1), position(2), position(3), quat(1), quat(2), quat(3), quat(4)));
    end

    for j=0:numWall-1
        % right wall
        position = [...
            j * longLen * 2 + 0.1,
            -0.5 * longLen,
            i * heightLen * 2 + 0.15];
        quat = [1, 0, 0, 0];
        
        text = strcat(text, ...
            sprintf('\t\t<body pos="%f %f %f" quat="%f %f %f %f"> <freejoint/> <geom class="wall"/> </body> \\n', ...
            position(1), position(2), position(3), quat(1), quat(2), quat(3), quat(4)));
    end
    
    for j=0:numWall-2
        % left wall
        position = [...
            j * longLen * 2 + 0.3, ...
            0.5 * longLen, ...
            i * heightLen * 2 + 0.15];
        quat = [1, 0, 0, 0];
        
        text = strcat(text, ...
            sprintf('\t\t<body pos="%f %f %f" quat="%f %f %f %f"> <freejoint/> <geom class="wall"/> </body> \\n', ...
            position(1), position(2), position(3), quat(1), quat(2), quat(3), quat(4)));
    end
    
    % first wall on left
    position = [...
        0.1, 0.5 * longLen, i * heightLen * 2 + 0.15];
    quat = [1, 0, 0, 0];
    
    text = strcat(text, ...
        sprintf('\t\t<body pos="%f %f %f" quat="%f %f %f %f"> <freejoint/> <geom class="wall"/> </body> \\n', ...
        position(1), position(2), position(3), quat(1), quat(2), quat(3), quat(4)));
    
    % last wall on left
    position = [...
        (numWall - 1) * longLen * 2 + 0.1, 0.5 * longLen, i * heightLen * 2 + 0.15];
    quat = [1, 0, 0, 0];
    
    text = strcat(text, ...
        sprintf('\t\t<body pos="%f %f %f" quat="%f %f %f %f"> <freejoint/> <geom class="wall"/> </body> \\n', ...
        position(1), position(2), position(3), quat(1), quat(2), quat(3), quat(4)));
end

text = strcat(text, fileread('xml-template/bldg_tail.txt'));

%% save file
fid = fopen(sprintf(output), 'wt');
fprintf(fid, text);
fclose(fid);