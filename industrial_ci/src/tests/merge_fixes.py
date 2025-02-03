#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022, Robert Haschke
# All rights reserved.
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

import yaml
import sys


def key(item):
   name = item.get("DiagnosticName")
   msg = item.get("DiagnosticMessage")
   file = msg.get("FilePath")
   offset = msg.get("FileOffset")
   return name, file, offset

def merge_fixes(files):
   """Merge all fixes files into mergefile"""
   # The fixes suggested by clang-tidy >= 4.0.0 are given under
   # the top level key 'Diagnostics' in the output yaml files
   mergefile = files[0]
   mergekey = "Diagnostics"
   merged = []
   seen = set()  # efficiently remember fixes already inserted

   def have(x):
      k = key(x)
      return k in seen or seen.add(k)

   def unique(seq):
      return [x for x in seq if not have(x)]

   for file in files:
      try:
         with open(file, 'r') as inp:
            content = yaml.safe_load(inp)
            if not content:
               continue  # Skip empty files.
            merged.extend(unique(content.get(mergekey, [])))
      except FileNotFoundError:
         pass

   with open(mergefile, 'w') as out:
      if merged:
         # Assemble output dict with MainSourceFile=''.
         output = {'MainSourceFile': '', mergekey: merged}
         yaml.safe_dump(output, out)


if __name__ == "__main__":
   merge_fixes(sys.argv[1:])
