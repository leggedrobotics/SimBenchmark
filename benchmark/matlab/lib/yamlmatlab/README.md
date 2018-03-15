yamlmatlab
----------

This is a github copy of https://code.google.com/p/yamlmatlab/
that has been packaged into +yaml namespace by Yauhen Yakimovich.


Installation
------------

Just add the codes and all subfolders to Matlab path by

```matlab
addpath(genpath('path/to/codes'));
```


Usage
-----

Reading in:

```matlab
yaml_file = 'test.yaml';
YamlStruct = yaml.ReadYaml(yaml_file);
```

Writing out

 ```matlab
 x.name='Martin';
 yaml.WriteYaml('test.yaml',x)
```


Main authors
------------

* Jiri Cigler, Dept. of Control Engineering, CTU Prague http://support.dce.felk.cvut.cz/pub/ciglejir/
* Jan Siroky, Energocentrum PLUS, s.r.o.
* Pavel Tomasko, student at Faculty of Electrical Engineering, CTU Prague and Institute of Chemical Technology, Prague.
