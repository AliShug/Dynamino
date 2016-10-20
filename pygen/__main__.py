import os
import errno
import sys
import platform
import shutil

import yaml

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
        print(protocol_config)
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
fmt_str = '/*\n{0}\n*/\n\n{1}\n\n'
full_cpp = fmt_str.format(license, core_cpp)
full_h = fmt_str.format(license, core_h);

stream = file('Dynamino/Dynamino.cpp', 'w')
stream.write(full_cpp)
stream.close()

stream = file('Dynamino/Dynamino.h', 'w')
stream.write(full_h)
stream.close()

if platform.system() == 'Windows':
    # Update the local Arduino lib
    profilePath = os.environ.get('USERPROFILE')
    if profilePath != None:
        libPath = profilePath + '/Documents/Arduino/libraries/Dynamino'
        # cname = 'Dynamino'
        # try:
        #     os.makedirs('{0}/{1}'.format(libPath,cname))
        # except OSError as exception:
        #     if exception.errno != errno.EEXIST:
        #         raise
        # file('{lib}/{c}/{c}.cpp'.format(lib=libPath, c=cname), 'w').write(full_cpp)
        # file('{lib}/{c}/{c}.h'.format(lib=libPath, c=cname), 'w').write(full_h)
        try:
            shutil.rmtree(libPath, ignore_errors=True)
            shutil.copytree('Dynamino', libPath)
        except shutil.Error as exception:
            print(exception)
    else:
        print 'Unable to locate current user\'s directory'
