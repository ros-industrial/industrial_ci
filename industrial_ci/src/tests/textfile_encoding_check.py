#!/usr/bin/python

# Copyright 2017 ROS-Industrial project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# **Description**
# Search { CHANGELOG.rst, package.xml } files in the target software.
# Parse and decode in utf-8 and see if error occurs.

targetfile_extensions = ['.py', '.rst']
files_match = []
for root, dirs, files in os.walk("."):
    for file in files:
        for targetfile_extension in targetfile_extensions:
            #print('DEBUG: filename: {}, targetfile_extension: {}'.format(file, targetfile_extension))
            if file.endswith(targetfile_extension):
                files_match.append(os.path.join(root, file))

for matchedfile in files_match:
    content = file(matchedfile).read()
    try:
        v = content.decode('utf-8')
    except UnicodeDecodeError as e:
        print(e)
        raise e
