function result = ismymatrix(obj)
import yaml.*;
result = ndims(obj) == 2 && all(size(obj) > 1);
end
