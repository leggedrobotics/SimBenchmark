% arg
numrow = 2;

%% generate string
head = fileread('urdf-template/head.urdf');
body = fileread('urdf-template/body.urdf');
tail = fileread('urdf-template/tail.urdf');

% head
text = head;
text = strcat(text, '\n');

% body
i = 0;
for row = 0:numrow-1
    for col = 0:numrow-1
        base_pos = [row * 2, col * 2, 1];
        
        text = strcat(text, ...
            sprintf(body, ...
            i, i, i,...
            base_pos(1), base_pos(2), base_pos(3), ...
            i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, ...
            i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, ...
            i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, ...
            i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, ...
            i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i...
            ) ...
            );
        
        i = i+1;
    end
end

% tail
text = strcat(text, '\n');
text = strcat(text, tail);

%% save file
fid = fopen(sprintf('robot%d.urdf', numrow * numrow),'wt');
fprintf(fid, text);
fclose(fid);