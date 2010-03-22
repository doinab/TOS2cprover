#! /usr/bin/env python

import sys, string, os

# expect: $1 = file name
if len(sys.argv) < 1:
  print "Usage: ./forall_IRQ_calls [file.c] "
  sys.exit("Missing input, exiting.")

filein_name  = sys.argv[1]
filein       = open(filein_name, 'r')

fileout_name_root = filein_name[0:-2]
fileout           = {}

contents = filein.read()
filein.close()

irqstr  = '''//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_'''

loop = True
startidx = 0
loopidx = 0

while loop:
  res = contents.find(irqstr, startidx)
  print startidx, res
  if res > -1: 

    # uncomment
    newcontents = contents[0:res] 
    newcontents += '  '
    newcontents += contents[res + 2:] 

    # write back to new file
    fileout[loopidx] = open(fileout_name_root + '_' + str(loopidx) + '.c', 'w')
    fileout[loopidx].write(newcontents)
    fileout[loopidx].close()

    # run reachability verification
    irq_type = contents[res+len(irqstr):res+len(irqstr)+4]
    print '\n' + '-'*80
    print 'REACHABILITY TRIAL - ' + irq_type + ' - ' + str(loopidx) + '\n'
    sys.stdout.flush()
    if irq_type == 'TIME':
      os.system('./c outrf_' + str(loopidx) + '.c sig_TIMERB0_VECTOR.1')
    if irq_type == 'ADC_':
      os.system('./c outrf_' + str(loopidx) + '.c sig_ADC_VECTOR.1')
    sys.stdout.flush()

    # increment loopidx and startidx
    startidx = res + len(irqstr)
    loopidx += 1

  else:
    loop = False

print str(loopidx) + ' IRQ calls found'

"""
for i in range(loopidx):
  print '\n' + '-'*80
  print 'REACHABILITY TRIAL - TIMERB0 - '+str(i)+'\n'
  sys.stdout.flush()
  os.system('./c outrf_'+str(i)+'.c sig_TIMERB0_VECTOR.1')
  sys.stdout.flush()
"""

