#! /usr/bin/env python

import sys, re, string
from pycparser import c_parser, c_ast, parse_file

# the set of defined function names
defd_func = set() 
# a dictionary of function-to-function call edges
calls = {}
# a set of reachable functions
reach = set()
# the set of IRQs, "sig_XXX_VECTOR"
sigs = set() 
sig_no = 0

# the set of defined global variables
defd_glob = set() 
# a dictionary of function-to-data accesses; read/write
raccs = {}
waccs = {}
# a dictionary of sig -> main-and-sig common global data
thr_raccs = {}    # data reads
thr_waccs = {}    # data writes
common = {}       # common data, at least one write
thr_faccs = {}    # thread functions
common_func = {}  # common thread functions

# -----------------------------------------------------------------
# Computes "sigs" out of "defd_func"
def get_sigs():

  global sigs, sig_no
  
  pattern = re.compile(r'sig_'
                       r'[A-Z0-9_]+'
                       r'_VECTOR$')
  for f in defd_func:
    sig_func = pattern.match(f)
    if sig_func:
      sigs.add(f)

  # Clean off identical timers
  for i in ['sig_TIMERA0_VECTOR', 'sig_TIMERA1_VECTOR', 'sig_TIMERB1_VECTOR']:
      sigs.discard(i)

  sig_no = len(sigs)

# -----------------------------------------------------------------
# Computes r/w data-access sets out of "r/waccs" and "calls"
def combine_acc_call():

  global thr_raccs, thr_waccs

  for root in set(['main']) | sigs:
    working_list = [root] # a list

    while len(working_list) > 0:
      # pop the first item from working_list
      fn = working_list.pop(0)

      # and push back to working_list
      if calls.has_key(fn):
        working_list.extend(calls[fn])

      # while filling in thr_r/waccs
      if raccs.has_key(fn): 
        if thr_raccs.has_key(root):
          thr_raccs[root].update(raccs[fn])
        else:
          thr_raccs[root] = raccs[fn]

      if waccs.has_key(fn): 
        if thr_waccs.has_key(root):
          thr_waccs[root].update(waccs[fn])
        else:
          thr_waccs[root] = waccs[fn]

# -----------------------------------------------------------------
def compute_common():

  for sig_i in sigs:
    if thr_waccs.has_key(sig_i):
      common[sig_i] = (thr_raccs['main'] & thr_waccs[sig_i])
    else:
      common[sig_i] = set()

#    if thr_raccs.has_key(sig_i):
#      common[sig_i].update(thr_raccs['main'] & thr_raccs[sig_i]) 

# -----------------------------------------------------------------
# Recursively takes an AST node, fills in "defd_glob", "r/waccs", 
# calculates and returns "common" 
def comdata(node, current_function='', rw=''):

  global defd_glob, raccs, waccs

  if calls == {}:
    print "Call deadfn() before comdata()"
    return None 

  tp = type(node)

  # FileAST: [ext**]
  # - ext: Decl, Typedef or FuncDef
  if tp == c_ast.FileAST:

    # fill in "r/waccs"
    if node.ext:
      exts = []
      for e in node.ext:
        comdata(e)
    
    # get_sigs()
    # compute "common" and return it
    combine_acc_call()
    compute_common()

    print 'Global variables (%s):' %(len(defd_glob))
    print defd_glob
    print '\nData accesses (read) (%s):' %(len(raccs))
    print raccs
    print '\nData accesses (write) (%s):' %(len(waccs))
    print waccs
    print '\nFunction calls (%s):' %(len(calls))
    print calls
    print '\nIRQs (%s):' %(len(sigs))
    print sigs
    print '\nThread data accesses (read) (%s):' %(len(thr_raccs))
    print thr_raccs
    print '\nThread data accesses (write) (%s):' %(len(thr_waccs))
    print thr_waccs
  
    return common

  # Decl: [name, quals, storage, type*, init*, bitsize*]
  # - name: the variable being declared, or None
  # - quals: list of qualifiers (const, volatile)
  # - storage: list of storage specifiers (extern, register, etc.)
  # - type: declaration type (probably nested with all the modifiers)
  # - init: initialization value, or None; can be a function call!
  # - bitsize: bit field size, or None
  elif tp == c_ast.Decl:
    # if this Decl is a global var decl, add the Decl.name to "defd_glob"
    if node.type and \
       node.name and \
       type(node.type) != c_ast.FuncDecl and \
       current_function == '':
      # for fixed-address variables, declname is 'P1IE@"0x0025"'
      lst = node.name.split('@')
      defd_glob.add(lst[0]) 

    # else (in a function), only look at Decl.init to fill "raccs"
    if current_function != '' and \
       node.init:
      comdata(node.init, current_function=current_function, rw='r')

  # Typedef: [name, quals, storage, type*]
  elif tp == c_ast.Typedef:
    pass

  # FuncDef: [decl*, param_decls**, body*]
  # - decl: a Decl for the function name 
  # - body: a Compound statement
  # - param_decls: (optional, old K&R-style) list of parameter decls
  elif tp == c_ast.FuncDef:
    if node.decl and \
       node.body:
      comdata(node.body, current_function=node.decl.name)

  # FuncDecl: [args*, type*, quals] 
  # type (args) quals
  elif tp == c_ast.FuncDecl:
    pass

  # ParamList: [params**]
  # a list of comma-separated function parameter declarations
  elif tp == c_ast.ParamList:
    pass

  # TypeDecl: [declname, quals, type*]
  # A base type declaration
  elif tp == c_ast.TypeDecl:
    pass

  # IdentifierType: [names]
  # Holder for types that are a simple identifier 
  # (e.g. the built-in void, char etc. and typedef-defined types)
  elif tp == c_ast.IdentifierType:
    pass

  # Constant: [type, value]
  # - type: int, char, float, etc. as in CLexer
  elif tp == c_ast.Constant:
    pass

  # Struct: [name, decls**]
  # - name: struct tag name
  # - decls: declaration of members
  # - packed: True if __attribute((packed))
  elif tp == c_ast.Struct:
    pass

  # Union: [name, decls**]
  # - name: union tag name
  # - decls: declaration of members
  elif tp == c_ast.Union:
    pass

  # PtrDecl: [quals, type*]
  elif tp == c_ast.PtrDecl:
    pass

  # ArrayDecl: [type*, dim*]
  elif tp == c_ast.ArrayDecl:
    pass

  # ArrayRef: [name*, subscript*]
  elif tp == c_ast.ArrayRef:
    if node.name:
      comdata(node.name, current_function=current_function, rw=rw)
    if node.subscript:
      comdata(node.subscript, current_function=current_function, rw=rw)

  # Assignment: [op, lvalue*, rvalue*]
  # - op: =, +=, /= etc.
  elif tp == c_ast.Assignment:
    if node.lvalue:
      comdata(node.lvalue, current_function=current_function, rw='w')
    if node.rvalue:
      comdata(node.rvalue, current_function=current_function, rw='r')

  # BinaryOp: [op, left*, right*]
  elif tp == c_ast.BinaryOp:
    if node.left:
      comdata(node.left, current_function=current_function, rw=rw)
    if node.right:
      comdata(node.right, current_function=current_function, rw=rw)

  # Break: []
  elif tp == c_ast.Break:
    pass

  # Case: [expr*, stmt*]
  elif tp == c_ast.Case:
    if node.expr:
      comdata(node.expr, current_function=current_function, rw='r')
    if node.stmt:
      comdata(node.stmt, current_function=current_function)

  # Cast: [to_type*, expr*]
  elif tp == c_ast.Cast:
    if node.expr:
      comdata(node.expr, current_function=current_function)

  # Compound: [decls**, stmts**]
  elif tp == c_ast.Compound:
    if node.decls:
      for d in node.decls:
        comdata(d, current_function=current_function) 
    if node.stmts:
      for s in node.stmts:
        comdata(s, current_function=current_function) 

  # Continue: []
  elif tp == c_ast.Continue:
    pass

  # Default: [stmt*]
  elif tp == c_ast.Default:
    if node.stmt:
      comdata(node.stmt, current_function=current_function)

  # EllipsisParam: []
  # Represents ellipsis (...) parameter in a function declaration
  elif tp == c_ast.EllipsisParam:
    pass

  # Enum: [name, values*], enumeration type specifier
  # - name: an optional ID
  # - values: an EnumeratorList
  elif tp == c_ast.Enum:
    if node.values:
      comdata(node.values, current_function=current_function)

  # Enumerator: [name, value*]
  # a name/value pair for enumeration values
  elif tp == c_ast.Enumerator:
    if node.value:
      comdata(node.value, current_function=current_function, rw='r')

  # EnumeratorList: [enumerators**]
  # a list of enumerators
  elif tp == c_ast.EnumeratorList:
    if node.enumerators:
      for e in node.enumerators:
        comdata(e, current_function=current_function) 

  # ExprList: [exprs**]
  #  a list of comma-separated expressions
  elif tp == c_ast.ExprList:
    if node.exprs:
      for e in node.exprs:
        comdata(e, current_function=current_function) 

  # For: [init*, cond*, next*, stmt*]
  # for (init; cond; next) stmt
  elif tp == c_ast.For:
    if node.init:
      comdata(node.init, current_function=current_function)
    if node.cond:
      comdata(node.cond, current_function=current_function, rw='r')
    if node.next:
      comdata(node.next, current_function=current_function)
    if node.stmt:
      comdata(node.stmt, current_function=current_function)

  # FuncCall: [name*, args*]
  # - name: Id
  # - args: ExprList
  elif tp == c_ast.FuncCall:
    if node.args:
      comdata(node.args, current_function=current_function, rw='r')

  # Goto: [name]
  elif tp == c_ast.Goto:
    pass

  # ID: [name]
  elif tp == c_ast.ID:
    # for fixed-address variables, declname is 'P1IE@"0x0025"'
    lst = node.name.split('@')
    nn = lst[0]
 
    if nn in defd_glob:
      if rw == 'r':
        if raccs.has_key(current_function):
          raccs[current_function].add(nn)
        else:
          raccs[current_function] = set([nn])
      elif rw == 'w':
        if waccs.has_key(current_function):
          waccs[current_function].add(nn)
        else:
          waccs[current_function] = set([nn])

  # If: [cond*, iftrue*, iffalse*]
  elif tp == c_ast.If:
    if node.iftrue:
      comdata(node.iftrue, current_function=current_function, rw='r')
    if node.cond:
      comdata(node.cond, current_function=current_function)
    if node.iffalse:
      comdata(node.iffalse, current_function=current_function)

  # While: [cond*, stmt*]
  elif tp == c_ast.While:
    if node.cond:
      comdata(node.cond, current_function=current_function, rw='r')
    if node.stmt:
      comdata(node.stmt, current_function=current_function)

  # DoWhile: [cond*, stmt*]
  elif tp == c_ast.DoWhile:
    if node.cond:
      comdata(node.cond, current_function=current_function, rw='r')
    if node.stmt:
      comdata(node.stmt, current_function=current_function)

  # Label: [name, stmt*]
  elif tp == c_ast.Label:
    if node.stmt:
      comdata(node.stmt, current_function=current_function)

  # Return: [expr*]
  elif tp == c_ast.Return:
    if node.expr:
      comdata(node.expr, current_function=current_function, rw='r')

  # StructRef: [name*, type, field*]
  # - type: . or -> (name.field or name->field)
  # name can be missing for designated initializers
  # the name can be a function call!
  elif tp == c_ast.StructRef:
    if node.name and \
       node.name in defd_glob:
      comdata(node.name, current_function=current_function)

  # Switch: [cond*, stmt*]
  elif tp == c_ast.Switch:
    if node.cond:
      comdata(node.cond, current_function=current_function, rw='r')
    if node.stmt:
      comdata(node.stmt, current_function=current_function)

  # TernaryOp: [cond*, iftrue*, iffalse*]
  # cond ? iftrue : iffalse
  elif tp == c_ast.TernaryOp:
    if node.iftrue:
      comdata(node.iftrue, current_function=current_function, rw='r')
    if node.cond:
      comdata(node.cond, current_function=current_function)
    if node.iffalse:
      comdata(node.iffalse, current_function=current_function)

  # Typename: [quals, type*]
  elif tp == c_ast.Typename:
    pass

  # UnaryOp: [op, expr*]
  elif tp == c_ast.UnaryOp:
    if node.expr:
      comdata(node.expr, current_function=current_function, rw=rw)

  return set()

# -----------------------------------------------------------------
# Computes a reachability set out of a call graph
def compute_reach(calls):

  global reach

  get_sigs()
  working_list = ['main'] + list(sigs) # a list

  while len(working_list) > 0:
    # pop the first item from working_list
    fn = working_list.pop(0)

    # and push back to working_list the list calls[fn]
    # if circular calls, this will fail
    if calls.has_key(fn):
      working_list.extend(calls[fn])
    reach.add(fn)

# -----------------------------------------------------------------
# Computes thread reachability out of a call graph
def compute_thread_reach(calls):

  global thr_faccs

  for root in ['main'] + list(sigs):
    
    thr_faccs[root] = set()
    working_list = [root] # a list

    while len(working_list) > 0:
      # pop the first item from working_list
      fn = working_list.pop(0)

      # and push back to working_list the list calls[fn]
      # if circular calls, this will fail
      if calls.has_key(fn):
        working_list.extend(calls[fn])
      thr_faccs[root].add(fn)

# -----------------------------------------------------------------
# Computes common thread reachability out of thread reachability
def compute_common_func():

  global common_func

  for root in list(sigs):
    common_func[root] = thr_faccs['main'] & thr_faccs[root]

# -----------------------------------------------------------------
# Fills in the defd_func and calls from an AST, recursively;
# takes an AST node and returns a dead_func set and sigs
def deadfn(node, current_function=''):

  global defd_func, calls

  tp = type(node)

  # FileAST: [ext**]
  # - ext: Decl, Typedef or FuncDef
  if tp == c_ast.FileAST:

    # fills in "defd_func" and "calls"
    if node.ext:
      exts = []
      for e in node.ext:
        deadfn(e)
    
    #print 'Defd_func (%s):' %(len(defd_func))
    #print defd_func
    #print 'Calls:'
    #print calls
  
    compute_reach(calls)
    print 'Reach (%s):' %(len(reach))
    print reach
    compute_thread_reach(calls)
    print 'Thread reach (%s):' %(len(thr_faccs))
    print thr_faccs
    compute_common_func()
    print '\nThread common reach: (%s):' %(len(common_func))
    print common_func

    return (defd_func.difference(reach), common_func, sigs)

  # Decl: [name, quals, storage, type*, init*, bitsize*]
  # - name: the variable being declared, or None
  # - quals: list of qualifiers (const, volatile)
  # - storage: list of storage specifiers (extern, register, etc.)
  # - type: declaration type (probably nested with all the modifiers)
  # - init: initialization value, or None; can be a function call!
  # - bitsize: bit field size, or None
  elif tp == c_ast.Decl:
    if node.type and \
       type(node.type) == c_ast.FuncDecl and\
       node.name != 'main':
      defd_func.add(node.name) 
    if node.init:
      deadfn(node.init, current_function=current_function)
    if node.bitsize:
      deadfn(node.bitsize, current_function=current_function)

  # Typedef: [name, quals, storage, type*]
  elif tp == c_ast.Typedef:
    if node.type:
      deadfn(node.type, current_function=current_function)

  # FuncDef: [decl*, param_decls**, body*]
  # - decl: a Decl for the function name 
  # - body: a Compound statement
  # - param_decls: (optional, old K&R-style) list of parameter decls
  elif tp == c_ast.FuncDef:
    if node.param_decls:
      for decl in node.param_decls:
        deadfn(decl, current_function=current_function) 
    if node.decl:
      defd_func.add(node.decl.name)
      if node.body:
        deadfn(node.body, current_function=node.decl.name)

  # FuncDecl: [args*, type*, quals] 
  # type (args) quals
  elif tp == c_ast.FuncDecl:
    pass

  # ParamList: [params**]
  # a list of comma-separated function parameter declarations
  elif tp == c_ast.ParamList:
    if node.params:
      for param in node.params:
        deadfn(param, current_function=current_function) 

  # TypeDecl: [declname, quals, type*]
  # A base type declaration
  elif tp == c_ast.TypeDecl:
    if node.type:
      deadfn(node.type, current_function=current_function)

  # IdentifierType: [names]
  # Holder for types that are a simple identifier 
  # (e.g. the built-in void, char etc. and typedef-defined types)
  elif tp == c_ast.IdentifierType:
    pass

  # Constant: [type, value]
  # - type: int, char, float, etc. as in CLexer
  elif tp == c_ast.Constant:
    pass

  # Struct: [name, decls**]
  # - name: struct tag name
  # - decls: declaration of members
  # - packed: True if __attribute((packed))
  elif tp == c_ast.Struct:
    if node.decls:
      for d in node.decls:
        deadfn(d, current_function=current_function) 

  # Union: [name, decls**]
  # - name: union tag name
  # - decls: declaration of members
  elif tp == c_ast.Union:
    if node.decls:
      for d in node.decls:
        deadfn(d, current_function=current_function) 

  # PtrDecl: [quals, type*]
  elif tp == c_ast.PtrDecl:
    if node.type:
      deadfn(node.type, current_function=current_function)

  # ArrayDecl: [type*, dim*]
  elif tp == c_ast.ArrayDecl:
    if node.type:
      deadfn(node.type, current_function=current_function)
    if node.dim:
      deadfn(node.dim, current_function=current_function)

  # ArrayRef: [name*, subscript*]
  elif tp == c_ast.ArrayRef:
    if node.name:
      deadfn(node.name, current_function=current_function)
    if node.subscript:
      deadfn(node.subscript, current_function=current_function)

  # Assignment: [op, lvalue*, rvalue*]
  # - op: =, +=, /= etc.
  elif tp == c_ast.Assignment:
    if node.lvalue:
      deadfn(node.lvalue, current_function=current_function)
    if node.rvalue:
      deadfn(node.rvalue, current_function=current_function)

  # BinaryOp: [op, left*, right*]
  elif tp == c_ast.BinaryOp:
    if node.left:
      deadfn(node.left, current_function=current_function)
    if node.right:
      deadfn(node.right, current_function=current_function)

  # Break: []
  elif tp == c_ast.Break:
    pass

  # Case: [expr*, stmt*]
  elif tp == c_ast.Case:
    if node.expr:
      deadfn(node.expr, current_function=current_function)
    if node.stmt:
      deadfn(node.stmt, current_function=current_function)

  # Cast: [to_type*, expr*]
  elif tp == c_ast.Cast:
    if node.to_type:
      deadfn(node.to_type, current_function=current_function)
    if node.expr:
      deadfn(node.expr, current_function=current_function)

  # Compound: [decls**, stmts**]
  elif tp == c_ast.Compound:
    if node.decls:
      for d in node.decls:
        deadfn(d, current_function=current_function) 
    if node.stmts:
      for s in node.stmts:
        deadfn(s, current_function=current_function) 

  # Continue: []
  elif tp == c_ast.Continue:
    pass

  # Default: [stmt*]
  elif tp == c_ast.Default:
    if node.stmt:
      deadfn(node.stmt, current_function=current_function)

  # EllipsisParam: []
  # Represents ellipsis (...) parameter in a function declaration
  elif tp == c_ast.EllipsisParam:
    pass

  # Enum: [name, values*], enumeration type specifier
  # - name: an optional ID
  # - values: an EnumeratorList
  elif tp == c_ast.Enum:
    if node.values:
      deadfn(node.values, current_function=current_function)

  # Enumerator: [name, value*]
  # a name/value pair for enumeration values
  elif tp == c_ast.Enumerator:
    if node.value:
      deadfn(node.value, current_function=current_function)

  # EnumeratorList: [enumerators**]
  # a list of enumerators
  elif tp == c_ast.EnumeratorList:
    if node.enumerators:
      for e in node.enumerators:
        deadfn(e, current_function=current_function) 

  # ExprList: [exprs**]
  #  a list of comma-separated expressions
  elif tp == c_ast.ExprList:
    if node.exprs:
      for e in node.exprs:
        deadfn(e, current_function=current_function) 

  # For: [init*, cond*, next*, stmt*]
  # for (init; cond; next) stmt
  elif tp == c_ast.For:
    if node.init:
      deadfn(node.init, current_function=current_function)
    if node.cond:
      deadfn(node.cond, current_function=current_function)
    if node.next:
      deadfn(node.next, current_function=current_function)
    if node.stmt:
      deadfn(node.stmt, current_function=current_function)

  # FuncCall: [name*, args*]
  # - name: Id
  # - args: ExprList
  elif tp == c_ast.FuncCall:
    if node.name:
      nm = node.name.name
      if calls.has_key(current_function):
        calls[current_function].append(nm)
      else:
        calls[current_function] = [nm]
         
    if node.args:
      deadfn(node.args, current_function=current_function)

  # Goto: [name]
  elif tp == c_ast.Goto:
    pass

  # ID: [name]
  elif tp == c_ast.ID:
    pass

  # If: [cond*, iftrue*, iffalse*]
  elif tp == c_ast.If:
    if node.iftrue:
      deadfn(node.iftrue, current_function=current_function)
    if node.cond:
      deadfn(node.cond, current_function=current_function)
    if node.iffalse:
      deadfn(node.iffalse, current_function=current_function)

  # While: [cond*, stmt*]
  elif tp == c_ast.While:
    if node.cond:
      deadfn(node.cond, current_function=current_function)
    if node.stmt:
      deadfn(node.stmt, current_function=current_function)

  # DoWhile: [cond*, stmt*]
  elif tp == c_ast.DoWhile:
    if node.cond:
      deadfn(node.cond, current_function=current_function)
    if node.stmt:
      deadfn(node.stmt, current_function=current_function)

  # Label: [name, stmt*]
  elif tp == c_ast.Label:
    if node.stmt:
      deadfn(node.stmt, current_function=current_function)

  # Return: [expr*]
  elif tp == c_ast.Return:
    if node.expr:
      deadfn(node.expr, current_function=current_function)

  # StructRef: [name*, type, field*]
  # - type: . or -> (name.field or name->field)
  # name can be missing for designated initializers
  # the name can be a function call!
  elif tp == c_ast.StructRef:
    if node.name:
      deadfn(node.name, current_function=current_function)

  # Switch: [cond*, stmt*]
  elif tp == c_ast.Switch:
    if node.cond:
      deadfn(node.cond, current_function=current_function)
    if node.stmt:
      deadfn(node.stmt, current_function=current_function)

  # TernaryOp: [cond*, iftrue*, iffalse*]
  # cond ? iftrue : iffalse
  elif tp == c_ast.TernaryOp:
    if node.iftrue:
      deadfn(node.iftrue, current_function=current_function)
    if node.cond:
      deadfn(node.cond, current_function=current_function)
    if node.iffalse:
      deadfn(node.iffalse, current_function=current_function)

  # Typename: [quals, type*]
  elif tp == c_ast.Typename:
    if node.type:
      deadfn(node.type, current_function=current_function)

  # UnaryOp: [op, expr*]
  elif tp == c_ast.UnaryOp:
    if node.expr:
      deadfn(node.expr, current_function=current_function)

  return set()
