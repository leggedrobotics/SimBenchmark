function result = isord(obj)
import yaml.*;
result = ~iscell(obj) && any(size(obj) > 1);
end
