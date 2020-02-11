#!/usr/bin/env python
# This program adds DmpBbo namespace to the class names
# in a dmp xml archive to solve boost serialization's
# unregistered_class error
import glob
import re

# class_names = \
#     ['ExponentialSystem', 'TimeSystem', 'SigmoidSystem',
#     'FunctionApproximatorLWR', 'MetaParametersLWR', 'ModelParametersLWR']

xml_files = glob.glob("*.xml")

ptr = re.compile('(class_name\s*=\s*\"\s*)(DmpBbo::)?([^"]+)')

for fname in xml_files:
    with open (fname, 'r+' ) as f:
        content = f.read()
        content_new = re.sub(ptr, r'\1DmpBbo::\3', content)
        f.seek(0)
        f.write(content_new)
        f.truncate()
