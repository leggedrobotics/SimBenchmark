function result = deflateimports(r)
import yaml.*;
result = recurse(r, 0, []);
end
function result = recurse(data, level, addit)
import yaml.*;
if iscell(data) && ~ismymatrix(data)
        result = iter_cell(data, level, addit);
    elseif isstruct(data)
        result = iter_struct(data, level, addit);
    else
        result = data;
    end;
end
function result = iter_cell(data, level, addit)
import yaml.*;
result = {};
    icollect = {};
    ii = 1;
    for i = 1:length(data)
        datai = data{i};
        if issingleimport(datai)
            if ~iscell(datai.import)
                datai.import = {datai.import};
            end;
            for j = 1:length(datai.import)
                icollect{end + 1} = datai.import{j};
            end;
        else
            result{ii} = recurse(datai, level + 1, addit);
            ii = ii + 1;
        end;
    end;
    if ~isempty(icollect)
        result{end + 1} = struct('import',{icollect});
    end;
end
function result = iter_struct(data, level, addit)
import yaml.*;
result = struct();
    for i = fields(data)'
        fld = char(i);
        result.(fld) = recurse(data.(fld), level + 1, addit);
    end;
end
function result = issingleimport_all(r)
import yaml.*;
result = all(cellfun(@issingleimport, r));
end
function result = issingleimport(r)
import yaml.*;
result = isstruct(r) && length(fields(r)) == 1 && isfield(r, 'import');
end
function result = addall(list1, list2)
import yaml.*;
for i = 1:length(list2)
        list1{end + 1} = list2{i};
    end;
    result = list1;
end
