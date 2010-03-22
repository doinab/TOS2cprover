#! /usr/bin/env python

import sys, string, os

# expect: $1 = file name #2 claim file name
if len(sys.argv) < 1:
  print "Usage: ./forall_IRQ_calls [file.c] [claim file name] "
  sys.exit("Missing input, exiting.")

filein_name  = sys.argv[2]
filein       = open(filein_name, 'r')
contents = filein.readline()
filein.close()

claims = contents.split()
print 'Claims (%s)' %(len(claims))
print claims

loopidx = len(claims)

for i in range(loopidx):
  print '\n' + '-'*80
  print 'VERIFICATION TRIAL '+str(i)+'\n'
  sys.stdout.flush()
  os.system('./c '+ sys.argv[1] + ' ' + claims[i])
  sys.stdout.flush()

"""
loopidx = len(sys.argv)-2

for i in range(loopidx):
  print '\n' + '-'*80
  print 'VERIFICATION TRIAL '+str(i)+'\n'
  sys.stdout.flush()
  os.system('./c outrf-sel.c '+sys.argv[i+2])
  sys.stdout.flush()
"""

