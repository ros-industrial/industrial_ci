#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from xml.etree import ElementTree
import sys

p = ElementTree.parse(sys.argv[1])
sys.exit(p.find('.//export/metapackage') == None)
