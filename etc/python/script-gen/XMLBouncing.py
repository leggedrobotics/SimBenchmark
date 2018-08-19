import os
import yaml
import numpy as np

out_dir = 'output/bouncing' # ADJUST

if not os.path.exists(out_dir):
    os.makedirs(out_dir)

# load constants from yaml
with open("../../../benchmark/yaml/bouncing.yaml", 'r') as stream:
    try:
        yaml_data = yaml.load(stream)
        const = yaml_data['constant']

        n = const['n']
        mass = const['m']
        radius = const['R']
        H = const['H']
        mu = const['mu_ball']

        inertia = np.ones(3) * 0.4 * mass * radius ** 2

    except yaml.YAMLError as exc:
        raise Exception(exc)

# other constants
gap = 2.0

text = open('xml-template/bounce_head.txt', 'r').read()
text += '\n'

# sphere
output_fname = 'bouncing{}.xml'.format(n**2)

# objects
for i in range(1, n+1):
    for j in range(1, n+1):
        position = ((i-1)*gap, (j-1)*gap, H)

        text += '\t\t<body pos="{} {} {}" quat="1 0 0 0"> \n'\
            .format(position[0], position[1], position[2])
        text += '\t\t\t<inertial pos="0 0 0" mass="{}" diaginertia="{} {} {}"/>\n'\
            .format(mass, inertia[0], inertia[1], inertia[2])
        text += '\t\t\t<freejoint/>\n'

        text += '\t\t\t<geom type="sphere" size="{}" friction="{} 0 0"/>\n'\
            .format(radius, mu)
        text += '\t\t</body> \n'

text += open('xml-template/bounce_tail.txt', 'r').read()

# save
print('save URDF for n={}...'.format(n))

with open(os.path.join(out_dir, output_fname), "w") as text_file:
    text_file.write(text)

print('Bouncing XML generate finished.')