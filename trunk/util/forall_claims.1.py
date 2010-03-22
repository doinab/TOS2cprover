#! /usr/bin/env python

import sys, string, os

# expect: $1 = file name
if len(sys.argv) < 1:
  print "Usage: ./forall_IRQ_calls [file.c] [list of claim names] "
  sys.exit("Missing input, exiting.")

loopidx = len(sys.argv)-2

for i in range(loopidx):
  print '\n' + '-'*80
  print 'VERIFICATION TRIAL '+str(i)+'\n'
  sys.stdout.flush()
  os.system('./c outrf-sel.c '+sys.argv[i+2])
  sys.stdout.flush()


