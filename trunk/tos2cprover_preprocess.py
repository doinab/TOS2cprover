#! /usr/bin/env python
import sys, re, string

# ------------------------------------------------------------------------------
# transform '$'s into '_' indiscriminately

def turn_dll_into__(line):
  return line.replace('$', '_')

# ------------------------------------------------------------------------------
# remove # line N and # N "file" directives 

def remove_line_directives(line):
  if re.match(r'#\s*\d+\s+\".*\n$', line) or \
     re.match(r'#\s*line\s+\d+\s*\n$', line):
    return '/* ' + line[0:-1] + ' */\n'
  return line

# ------------------------------------------------------------------------------
# simplify __attribute((packed)) into __attribute_packed
# remove __attribute((wakeup)) 
# remove __attribute((noinline)) 
# remove __attribute((naked)) 
# simplify __attribute((interrupt(N))) into __attribute_interrupt_N

def simplify_attr(line):
  l1 = line.replace(r'__attribute__((packed))', '__attribute_packed')
  l1 = l1.replace(r'__attribute((packed))', '__attribute_packed')

  l2 = l1.replace(r'__attribute((wakeup)) ', '')
  l2 = l2.replace(r'__attribute((noinline)) ', '')
  l2 = l2.replace(r'__attribute((naked)) ', '')
  l2 = l2.replace(r'__attribute((unused)) ', '')

#  l3 = re.sub(r'__attribute\(\(interrupt\((\d+)\)\)\)',
#         r'__attribute_interrupt \1',
#         l2)
  # eliminate the attr for now
  l3 = re.sub(r'__attribute\(\(interrupt\((\d+)\)\)\)',
         r'',
         l2)

  return l3

# ------------------------------------------------------------------------------
def turn_asm_line_into_c(line):
  # split by ';'
  instr_list = re.split(r';', line)

  final_instr_list = [];

  for instr in instr_list:
    # if this is an __asm instruction, call its handling function
    if re.match(r'\s*__asm\s+', instr):
      final_instr_list.append(turn_asm_instr_into_c(instr))
    else:
      final_instr_list.append(instr)

  # return inlined final list
  return ';'.join(final_instr_list)

# ------------------------------------------------------------------------------
# turn into C asm instructions of the type
# __asm volatile ("eint");}
# __asm ("mov.w r2,%0" : "=r"(t->regs.status));
# __asm volatile ("" :  :  : "memory");
# __asm ("mov.w r4,%0" : "=m"(TinyThreadSchedulerP_yielding_thread->regs.r4)); ... on a long line

def turn_asm_instr_into_c(instr):
  pattern = re.compile(r'\s*__asm\s+'              
            r'(?P<qualifier>\w*)\s*'                   # at most one word-qualifier 
            r'\(\s*'                                   # (
            r'(?P<asm_instr>.*)\s*'                    # followed by an asm instr
            r'\)\s*')                                  # )

  asm_instr = pattern.match(instr)

  if asm_instr is None:
    # unrecognized assembly instruction
    print 'unknown asm instruction ' + instr
    return instr

  # translate instruction
  # nop
  if re.match(r'^\"nop\"$', asm_instr.group('asm_instr')):
    return ''

  # eint
  if re.match(r'^\"eint\"$', asm_instr.group('asm_instr')):
    return '_R2 |= 0x0008'

  # dint
  if re.match(r'^\"dint\"$', asm_instr.group('asm_instr')):
    return '_R2 &= 0xfff7'

  # "" :  :  : "memory" ; memory clobbering, against gcc optimizations
  if re.match(r'^\"\"\s*:\s*:\s*:\s*\"memory\"\s*$', asm_instr.group('asm_instr')):
    return ''

  # "bis  %0, r2" :  : "m"(temp) ; "bis src, dst" := "src or dst -> dst"
  pattern = re.compile(r'^\"bis(\.w)?\s+'
            r'%0,\s*'
            r'r(?P<reg_no>\d+)\"\s*:\s*:\s*'
            r'\"[mr]\"\('
            r'(?P<c_code>.*)'
            r'\)\s*$')
  mov_instr = pattern.match(asm_instr.group('asm_instr'))  
  if mov_instr is not None:
    return '_R' + mov_instr.group('reg_no') + ' |= ' + mov_instr.group('c_code')

  # "mov(.w)? rN,%0" : "=m"(C code) ; any memory operand allowed
  # "mov(.w)? rN,%0" : "=r"(C code) ; a general register allowed
  # __asm ("mov.w r2,%0" : "=r"(t->regs.status));
  pattern = re.compile(r'^\"mov(\.w)?\s+'
            r'r(?P<reg_no>\d+),\s*'
            r'%0\"\s*:\s*'
            r'\"=[mr]\"\('
            r'(?P<c_code>.*)'
            r'\)\s*$')
  mov_instr = pattern.match(asm_instr.group('asm_instr'))  
  if mov_instr is not None:
    return mov_instr.group('c_code') + ' = _R' + mov_instr.group('reg_no') 

  # "mov(.w)? %0, rN" :  : "=m"(C code) 
  # "mov(.w)? %0, rN" :  : "=r"(C code) 
  pattern = re.compile(r'^\"mov(\.w)?\s+'
            r'%0,\s*'
            r'r(?P<reg_no>\d+)\"\s*:\s*:\s*'
            r'\"[mr]\"\('
            r'(?P<c_code>.*)'
            r'\)\s*$')
  mov_instr = pattern.match(asm_instr.group('asm_instr'))  
  if mov_instr is not None:
    return '_R' + mov_instr.group('reg_no') + ' = ' + mov_instr.group('c_code')

  # untreated assembly instruction
  else:
    print asm_instr.group('asm_instr')

  return instr

 