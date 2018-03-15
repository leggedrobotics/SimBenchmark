function result = mergeimports(data, verb)
import yaml.*;
if ~exist('verb','var')
        verb = 0;
    end;
    result = recurse(data, 0, [], verb);
end
function result = recurse(data, level, addit, verb)
import yaml.*;
indent = repmat(' | ',1,level); % for debugging
    if iscell(data)
        result = iter_cell(data, level, addit, verb);
    elseif isstruct(data)
        result = iter_struct(data, level, addit, verb);
    else
        if any(verb == 1) % for debugging
            fprintf([indent,'Some data: ']);
            disp(data);
        end;
        result = data;
    end;
end
function result = iter_cell(data, level, addit, verb)
import yaml.*;
indent = repmat(' | ',1,level); % for debugging
    result = {};
    if any(verb == 1); fprintf([indent,'cell {\n']); end; % for debugging
    for i = 1:length(data)
        itemcontent = recurse(data{i}, level + 1, addit, verb);
        result{end + 1} = itemcontent;
    end;
    if any(verb == 1); fprintf([indent,'} cell\n']); end; % for debugging
end
function result = iter_struct(data, level, addit, verb)
import yaml.*;
indent = repmat(' | ',1,level); % for debugging
    result = struct();
    collected_imports = {};
    if any(verb == 1); fprintf([indent,'struct {\n']); end; % for debugging
    for i = fields(data)'
        fld = char(i);
        if any(verb == 1); fprintf([indent,' +-field ',fld,':\n']); end; % for debugging
        result.(fld) = recurse(data.(fld), level + 1, addit, verb);
        if isequal(fld, 'import')
            processed_import = process_import_field(result.(fld));
            result = rmfield(result, 'import');
            if isstruct(processed_import)
                collected_imports{end+1} = processed_import;
            else
                disp(processed_import);
                error('Expected struct, otherwise it cannot be merged with the rest.');
            end;
        end;
    end;
    for i = 1:length(collected_imports)
        result = merge_struct(result, collected_imports{i}, {}, 'deep');
    end;
    if any(verb == 1); fprintf([indent,'} struct\n']); end; % for debugging
end
function result = process_import_field(data)
import yaml.*;
if iscell(data)
        merged_structs = struct();
        collected_nonstruct = {};
        for i = 1:length(data)
            if isstruct(data{i})
                merged_structs = merge_struct(merged_structs, data{i}, {}, 'deep');
            else
                collected_nonstruct{end+1} = data{i};
            end;
        end;
        if isempty(collected_nonstruct)
            result = merged_structs;
        elseif isempty(merged_structs)
            result = collected_nonstruct;
        else
            result = {merged_structs; collected_nonstruct};
        end;
    else
        error('BUG: import field should always contain a cell.');
    end;
end
