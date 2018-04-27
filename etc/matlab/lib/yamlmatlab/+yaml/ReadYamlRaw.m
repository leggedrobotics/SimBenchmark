function result = ReadYamlRaw(filename, verbose, nosuchfileaction, treatasdata)
import yaml.*;
if ~exist('verbose','var')
        verbose = 0;
    end;
    if ~exist('nosuchfileaction','var')
        nosuchfileaction = 0;
    end;
    if ~ismember(nosuchfileaction,[0,1])
        error('nosuchfileexception parameter must be 0,1 or missing.');
    end;
    if(~exist('treatasdata','var'))
        treatasdata = 0;
    end;
    if ~ismember(treatasdata,[0,1])
        error('treatasdata parameter must be 0,1 or missing.');
    end;
    [pth,~,~] = fileparts(mfilename('fullpath'));       
    try
        import('org.yaml.snakeyaml.*');
        javaObject('Yaml');
    catch
        dp = [pth filesep 'external' filesep 'snakeyaml-1.9.jar'];
        if not(ismember(dp, javaclasspath ('-dynamic')))
        	javaaddpath(dp); % javaaddpath clears global variables!?
        end
        import('org.yaml.snakeyaml.*');
    end;
    setverblevel(verbose);
    result = load_yaml(filename, nosuchfileaction, treatasdata);
end
function result = load_yaml(inputfilename, nosuchfileaction, treatasdata)
import yaml.*;
persistent nsfe;
    if exist('nosuchfileaction','var') %isempty(nsfe) && 
        nsfe = nosuchfileaction;
    end;
    persistent tadf;
    if isempty(tadf) && exist('treatasdata','var')
        tadf = treatasdata;
    end;
    yaml = org.yaml.snakeyaml.Yaml(); % It appears that Java objects cannot be persistent!?
    if ~tadf
        [filepath, filename, fileext] = fileparts(inputfilename);
        if isempty(filepath)
            pathstore = cd();
        else
            pathstore = cd(filepath);
        end;
    end;
    try
        if ~tadf
            result = scan(yaml.load(fileread([filename, fileext])));
        else
            result = scan(yaml.load(inputfilename));
        end;
    catch ex
        if ~tadf
            cd(pathstore);
        end;
        switch ex.identifier
            case 'MATLAB:fileread:cannotOpenFile'
                if nsfe == 1
                    error('MATLAB:MATYAML:FileNotFound', ['No such file to read: ',filename,fileext]);
                elseif nsfe == 0
                    warning('MATLAB:MATYAML:FileNotFound', ['No such file to read: ',filename,fileext]);
                    result = struct();
                    return;
                end;
        end;
        rethrow(ex);
    end;
    if ~tadf
        cd(pathstore);    
    end;
end
function result = scan(r)
import yaml.*;
if isa(r, 'char')
        result = scan_string(r);
    elseif isa(r, 'double')
        result = scan_numeric(r);
    elseif isa(r, 'logical')
        result = scan_logical(r);
    elseif isa(r, 'java.util.Date')
        result = scan_datetime(r);
    elseif isa(r, 'java.util.List')
        result = scan_list(r);
    elseif isa(r, 'java.util.Map')
        result = scan_map(r);
    else
        error(['Unknown data type: ' class(r)]);
    end;
end
function result = scan_string(r)
import yaml.*;
result = char(r);
end
function result = scan_numeric(r)
import yaml.*;
result = double(r);
end
function result = scan_logical(r)
import yaml.*;
result = logical(r);
end
function result = scan_datetime(r)
import yaml.*;
result = DateTime(r);
end
function result = scan_list(r)
import yaml.*;
result = cell(r.size(),1);
    it = r.iterator();
    ii = 1;
    while it.hasNext()
        i = it.next();
        result{ii} = scan(i);
        ii = ii + 1;
    end;
end
function result = scan_map(r)
import yaml.*;
it = r.keySet().iterator();
    while it.hasNext()
        next = it.next();
        i = next;
        ich = char(i);
        if iskw_import(ich)
            result.(ich) = perform_import(r.get(java.lang.String(ich)));
        else
            result.(genvarname(ich)) = scan(r.get(java.lang.String(ich)));
        end;
    end;
    if not(exist('result','var'))
        result={};
    end
end
function result = iskw_import(r)
import yaml.*;
result = isequal(r, 'import');
end
function result = perform_import(r)
import yaml.*;
r = scan(r);
    if iscell(r) && all(cellfun(@ischar, r))
        result = cellfun(@load_yaml, r, 'UniformOutput', 0);
    elseif ischar(r)
        result = {load_yaml(r)};
    else
        disp(r);
        error(['Importer does not unterstand given filename. '               'Invalid node displayed above.']);
    end;
end
function setverblevel(level)
import yaml.*;
global verbose_readyaml;
    verbose_readyaml = 0;
    if exist('level','var')
        verbose_readyaml = level;
    end;
end
function result = getverblevel()
import yaml.*;
global verbose_readyaml; 
    result = verbose_readyaml;
end
function info(level, text, value_to_display)
import yaml.*;
if getverblevel() >= level
        fprintf(text);
        if exist('value_to_display','var')
            disp(value_to_display);
        else
            fprintf('\n');
        end;
    end;
end
