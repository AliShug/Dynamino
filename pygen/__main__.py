import os, errno, sys, yaml
from jinja2 import Environment, PackageLoader

# Jinja2 environment, configured to 'template/' dir
env = Environment(  loader=PackageLoader('pygen', 'template'),
                    trim_blocks=True,
                    lstrip_blocks=True)

core_cpp = ''
core_h = ''

# Load license text
stream = file('LICENSE.txt')
license = stream.read()
stream.close()

generated_protocols = []

def gen_target(target):
    global core_cpp
    global core_h

    protocol = target['protocol']
    if protocol not in generated_protocols:
        # Load protocol configuration file
        stream = file('config/{0}_shared.yaml'.format(protocol))
        protocol_config = yaml.load(stream)
        stream.close()
        # Render protocol templates
        cpp_template = env.get_template('{0}_shared.template.cpp'.format(protocol))
        h_template = env.get_template('{0}_shared.template.h'.format(protocol))
        core_cpp += cpp_template.render(protocol_config)
        core_h += h_template.render(protocol_config)

        generated_protocols.append(protocol)



# Configuration
stream = file('pygen_config.yaml')
core_config = yaml.load(stream)
stream.close()

print(core_config['make_targets'])

# Load templating configuration for each target servo type
targets = []
for target in core_config['make_targets']:
    stream = file('config/servos/{0}.yaml'.format(target))
    target_config = yaml.load(stream)
    stream.close()
    targets.append(target_config)

for t in targets:
    gen_target(t)

# Save the generated files
stream = file('Dynamino/Dynamino.cpp', 'w')
stream.write('/*\n{0}\n*/\n\n'.format(license))
stream.write('namespace Dynamino {\n')
stream.write(core_cpp)
stream.write('\n}\n')
stream.close()

stream = file('Dynamino/Dynamino.h', 'w')
stream.write('/*\n{0}\n*/\n\n'.format(license))
stream.write('namespace Dynamino {\n')
stream.write(core_h)
stream.write('\n}\n')
stream.close()
