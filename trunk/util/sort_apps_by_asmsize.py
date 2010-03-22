#! /usr/bin/env python

import sys, string, os

# expect environment variable: TOSROOT_DIR=/Users/doina/tinyos-2.x/
if len(sys.argv) > 0:
  print "Expecting zero command line args; ignoring any given args."



filein_name  = sys.argv[1]
filein       = open(filein_name, 'r')
filein.close()
