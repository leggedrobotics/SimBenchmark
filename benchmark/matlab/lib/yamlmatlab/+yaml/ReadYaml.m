function result = ReadYaml(filename, nosuchfileaction, makeords, treatasdata, dictionary)
import yaml.*;
if ~exist('nosuchfileaction','var')
        nosuchfileaction = 0;
    end;
    if ~ismember(nosuchfileaction,[0,1])
        error('nosuchfileexception parameter must be 0,1 or missing.');
    end;
    if ~exist('makeords','var')
        makeords = 0;
    end;
    if ~ismember(makeords,[0,1])
        error('makeords parameter must be 0,1 or missing.');
    end;    
    if(~exist('treatasdata','var'))
        treatasdata = 0;
    end;
    if ~ismember(treatasdata,[0,1])
        error('treatasdata parameter must be 0,1 or missing.');
    end; 
    ry = ReadYamlRaw(filename, 0, nosuchfileaction, treatasdata);
    ry = deflateimports(ry);
    if iscell(ry) &&         length(ry) == 1 &&         isstruct(ry{1}) &&         length(fields(ry{1})) == 1 &&         isfield(ry{1},'import')        
        ry = ry{1};
    end;
    ry = mergeimports(ry);    
    ry = doinheritance(ry);
    ry = makematrices(ry, makeords);    
    if exist('dictionary','var')
        ry = dosubstitution(ry, dictionary);
    end;
    result = ry;
    clear global nsfe;
end
