#! /usr/bin/env python

import sys, re, string
from pycparser import c_parser, c_ast, parse_file

sys.path.append('./lib')
from msp430_hw import *

fixed_vars = {}    # an empty dict
dead_func  = set()
include_dead = True
cdata = {}
cfunc = None # a set, to flatten common_data
sigs = set()
irq_instr_no = 0

# The AST is a n-nary tree of named nodes. 
# See: - c_ast.py for options to pass it  
#      -_c_ast.yaml for the names and types of nodes 
#      - cdecl.py for recursively visiting the AST 

# Generates an IRQ call 
# -----------------------------------------------------------------
def gen_irq_call(irq):

  global irq_instr_no

  instr = '''  /* IRQ INSTR ''' + str(irq_instr_no) + ''' */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; ''' + irq + '''(); _R2 |= 0x0008; }
'''

  irq_instr_no += 1

  return instr  

# -----------------------------------------------------------------
# Generates C code from an AST, recursively; takes an AST node and 
# returns (string, set)
def code(node, \
    offset=0, \
    add_ptr=False, \
    add_arr=False, \
    arr_str='', \
    end_statement=False, \
    base_struct='', \
    include_deadfn=True, \
    deadfn_set=(), \
    common_data=None, \
    common_func=None, \
    rename_func=False, \
    sig_func=None):

  lead = ' ' * offset
  semicolon = ';' if end_statement else ''
  tp = type(node)

  # FileAST: [ext**]
  # - ext: Decl, Typedef or FuncDef
  if tp == c_ast.FileAST:

    global dead_func
    dead_func = deadfn_set
    global include_dead
    include_dead = include_deadfn

    global cdata, cfunc
    cdata = common_data
    if common_func:
      cfunc = set()
      for i in common_func:
        cfunc |= common_func[i]

    global sigs
    if sig_func:
      sigs = sig_func
  
    if node.ext:
      exts = []
      for e in node.ext:

        cd, ct = code(e, end_statement=True, rename_func=False)
        if cd != '': exts.append(cd)

        if (type(e) == c_ast.Decl and \
            e.type and \
            type(e.type) == c_ast.FuncDecl and \
            cfunc and e.name in cfunc) or \
           (type(e) == c_ast.FuncDef and \
            e.decl and \
            cfunc and e.decl.name in cfunc):
          rcd, rct = code(e, end_statement=True, rename_func=True)
          if rcd != '': exts.append(rcd)

      return ('\n\n'.join(exts), None)

    return ('', None)

  # Decl: [name, quals, storage, type*, init*, bitsize*]
  # - name: the variable being declared, or None
  # - quals: list of qualifiers (const, volatile)
  # - storage: list of storage specifiers (extern, register, etc.)
  # - type: declaration type (probably nested with all the modifiers)
  # - init: initialization value, or None
  # - bitsize: bit field size, or None
  elif tp == c_ast.Decl:
    # if this is a FuncDecl of a deadfn, ignore
    if not include_dead and \
       type(node.type) == c_ast.FuncDecl and \
       node.name in dead_func:
      return ('', set())

    # else
    ty, tt = (code(node.type, offset, rename_func=rename_func)) if node.type else ''
    if ty is '':
      return ('', set())

    nn = node.name if node.name else ''

    qs = ' '.join(node.quals) 
    if len(node.quals) > 0: qs += ' '
    st = ' '.join(node.storage)
    if len(node.storage) > 0: st += ' '

    bc, bt = code(node.bitsize) if node.bitsize else ('', set())
    bs = (' :' + bc) if node.bitsize else ''

    # determining the initialization block
    it = ''
    itt = set()
    itl = (type(node.init) == c_ast.ExprList)
    if node.init:
      c1, itt = code(node.init, rename_func=rename_func)
      if itl:
        it = ' = {' + c1 + '}'
      else:
        it = ' = ' + c1

    # designated initializers for unions
    if node.type and \
       type(node.type) == c_ast.TypeDecl and \
       node.type.type and \
       type(node.type.type) == c_ast.Union and \
       itl:
      it, t1 = code(node.init, offset, base_struct=nn, end_statement=True)
      return (lead + qs + st + ty + bs + semicolon + '\n' + it, tt | itt)

    res_code = lead + qs + st + ty + it + bs + semicolon
    return (res_code, tt | itt)

  # Typedef: [name, quals, storage, type*]
  elif tp == c_ast.Typedef:
    qs = ' '.join(node.quals)
    if len(node.quals): qs += ' '
    # storage is a redundant 'typedef'
    # name is a redundant name, also in TypeDecl

    return (lead + 'typedef ' + qs + code(node.type)[0] + ';', set())

  # FuncDef: [decl*, param_decls**, body*]
  # - decl: a Decl for the function name 
  # - body: a Compound statement
  # - param_decls: (optional, old K&R-style) list of parameter decls
  elif tp == c_ast.FuncDef:
    # if this is a FuncDef of a deadfn, ignore
    if not include_dead and \
       node.decl and \
       node.decl.type and \
       type(node.decl.type) == c_ast.FuncDecl and \
       node.decl.name in dead_func:
      return ('', set())

    # else
    params = [] 
    ps = ''
    if node.param_decls:
      params = [code(decl, offset, rename_func=rename_func)[0] for decl in node.param_decls]
      ps = '\n'.join(params)

    return (lead + code(node.decl, offset, rename_func=rename_func)[0] + ps+'\n'+ \
            code(node.body, offset+2, rename_func=rename_func)[0], 
            set())

  # FuncDecl: [args*, type*, quals] 
  # type (args) quals
  elif tp == c_ast.FuncDecl:
    as_ = ''
    if node.args:
      as_ = code(node.args)[0]

    ty = code(node.type, add_arr=add_arr, add_ptr=add_ptr, arr_str=arr_str, rename_func=rename_func)[0]
    qc = node.quals or ''

    return (ty + '(' + as_ + ') ' + qc, set())

  # ParamList: [params**]
  # a list of comma-separated function parameter declarations
  elif tp == c_ast.ParamList:
    as_ = []    
    if node.params:
      as_ = [code(param)[0] for param in node.params]
  
    return (', '.join(as_), set())

  # TypeDecl: [declname, quals, type*]
  # A base type declaration
  elif tp == c_ast.TypeDecl:

    # NB: there might not be a name: "(void *)f"
    if not node.declname:
      ty = code(node.type, add_ptr=add_ptr, add_arr=add_arr, arr_str=arr_str)[0]
    else:
      ty = code(node.type)[0]

    dn = ''

    if node.declname:
      # for fixed-address variables, declname is 'P1IE@"0x0025"'
      # if node.declname:
      lst = node.declname.split('@')

      if add_arr: dn += '('
      if add_ptr: dn += '*'
      if rename_func and cfunc and (lst[0] in cfunc):
        dn += 'SIG_' + lst[0]
      else:
        dn += lst[0]

      if add_arr: dn += arr_str + ')'
 
      # if a fixed-address variable declaration
      if len(lst) > 1: 
        # save the mapping declname-to-addr
        addr = lst[1].strip('\"')
        fixed_vars[lst[0]] = addr
        # and return nothing for this declaration
        return ('', set())

    # if not a fixed-address variable declaration
    return (ty + ' ' + dn, set())

  # IdentifierType: [names]
  # Holder for types that are a simple identifier 
  # (e.g. the built-in void, char etc. and typedef-defined types)
  elif tp == c_ast.IdentifierType:
    node.names.reverse()
    star = ' *' if add_ptr else ''
    # arr  = arr_str if add_arr else ''
    return (' '.join(node.names) + star, set())

  # Constant: [type, value]
  # - type: int, char, float, etc. as in CLexer
  elif tp == c_ast.Constant:
    return (node.value if node.value else '', set())

  # Struct: [name, decls**]
  # - name: struct tag name
  # - decls: declaration of members
  # - packed: True if __attribute((packed))
  elif tp == c_ast.Struct:
    nn = (node.name + ' ') if node.name else ''
    ds = []
    dc = ''
    if node.decls:
      ds = [code(d, offset+2, end_statement=True)[0] for d in node.decls]
      dc = '{\n' + '\n'.join(ds)
      dc += ('\n') if len(node.decls) > 0 else '' 
      dc += lead + '}'
    pc = ' __attribute((packed))' if node.packed else ''

    return ('struct ' + nn + dc + pc, set())

  # Union: [name, decls**]
  # - name: union tag name
  # - decls: declaration of members
  elif tp == c_ast.Union:
    nn = (node.name + ' ') if node.name else ''
    ds = []
    dc = ''
    if node.decls:
      ds = [code(d, offset+2, end_statement=True)[0] for d in node.decls]
      dc = ' {\n' + '\n'.join(ds) 
      dc += ('\n') if len(node.decls) > 0 else '' 
      dc += lead + '}'

    return ('union ' + nn + dc, set())

  # PtrDecl: [quals, type*]
  elif tp == c_ast.PtrDecl:
    qs = (' '.join(node.quals) + ' ') if node.quals else ''

    return (qs + code(node.type, add_ptr=True, add_arr=add_arr, arr_str=arr_str)[0],
            set())

  # ArrayDecl: [type*, dim*]
  elif tp == c_ast.ArrayDecl:
    arr = ''
    if node.dim: 
      arr = '[%s]' % code(node.dim)[0]

    return (code(node.type, add_arr=True, arr_str=arr)[0], 
            set())

  # ArrayRef: [name*, subscript*]
  elif tp == c_ast.ArrayRef:
    nc, nt = code(node.name) 
    sc, st = code(node.subscript, rename_func=rename_func)
    return (nc + '[' + sc + ']',
            nt | st)

  # Assignment: [op, lvalue*, rvalue*]
  # - op: =, +=, /= etc.
  elif tp == c_ast.Assignment:
    lc, lt = code(node.lvalue, base_struct=base_struct, rename_func=rename_func)
    # if a plain write, with no read, the left hand doesn't matter
    if node.op == '=':
      lt = set()

    rc_temp, rt = code(node.rvalue, rename_func=rename_func)
    # print lc + ' ' + str(rt)
    rc = rc_temp

    if type(node.rvalue) == c_ast.ExprList:
      rc = '{' + rc_temp + '}'

    return (lead + lc + ' ' + node.op + ' ' + rc + semicolon,
            lt | rt)

  # BinaryOp: [op, left*, right*]
  elif tp == c_ast.BinaryOp:
    lc, lt = code(node.left, rename_func=rename_func)
    rc, rt = code(node.right, rename_func=rename_func)

    if type(node.left) == c_ast.BinaryOp or type(node.left) == c_ast.Assignment:
      lc = '(' + lc + ')'
    if type(node.right) == c_ast.BinaryOp or type(node.left) == c_ast.Assignment:
      rc = '(' + rc + ')'

    # print 'BinaryOp ' + lc + ' ' + node.op + ' ' + rc

    return (lc + ' ' + node.op + ' ' + rc + semicolon,
            lt | rt)

  # Break: []
  elif tp == c_ast.Break:
    return (lead + 2*' ' + 'break' + semicolon,
            set())

  # Case: [expr*, stmt*]
  elif tp == c_ast.Case:
    ec, et = code(node.expr, rename_func=rename_func)
    sc, st = code(node.stmt, offset+2, end_statement=True, rename_func=rename_func)

    irq_instr = ''
    if not rename_func:
      for i in st:
        irq_instr += gen_irq_call(i)

    return (lead + 'case ' + ec + ':\n' + irq_instr + sc,
            et)

  # Cast: [to_type*, expr*]
  elif tp == c_ast.Cast:
    if    type(node.to_type) == c_ast.Typename \
      and type(node.expr) == c_ast.Constant \
      and type(node.to_type.type) == c_ast.PtrDecl:

      # map the constant (e.g. .value '49U' or '0x0083') to hex address
      if node.expr.value == '0':            # 0
        ds_hexaddr = '0x0000'
      elif node.expr.value[0:2] == '0x': # '0x0083'
        # print '*** Pointer cast from hex ' + node.expr.value

        ds_hexaddr_temp = '' + node.expr.value.upper()
        ds_hexaddr = '0x' + '0'*(6-len(ds_hexaddr_temp)) + ds_hexaddr_temp[2:]
      else:                                 # '49U'
        # print '*** Pointer cast from dec ' + node.expr.value

        decstr = node.expr.value[0:-1]
        decaddr = int(decstr)
        hexaddr = hex(decaddr)
        ds_hexaddr_temp = '' + str(hexaddr).upper()
        ds_hexaddr = '0x' + '0'*(6-len(ds_hexaddr_temp)) + ds_hexaddr_temp[2:]

      return (MSP430_map[ds_hexaddr], set())

    ec, et = code(node.expr, rename_func=rename_func)
    tc, tt = code(node.to_type, rename_func=rename_func)

    return ('(' + tc + ')(' + ec + ')',
            et)

  # Compound: [decls**, stmts**]
  elif tp == c_ast.Compound:
    flag = False

    # look at the statements first
    j = 0
    sc = ''    
    if node.stmts:
      # calculate the code for the rest of the statement list
      for s in node.stmts:
        ss, st = code(s, offset, end_statement=True, rename_func=rename_func)
        # if the first one is a Compound or a tainted stmt, set a flag for later
        if j == 0 and (type(s) == c_ast.Compound or len(st) > 0):
          flag = True

        irq_instr = ''
        if not rename_func:
          for i in st:
            irq_instr += gen_irq_call(i)

        sc += irq_instr + ss + '\n'
        j += 1

    j = 0
    dc = ''
    if node.decls:
      for d in node.decls:
        ds, dt = code(d, offset, end_statement=True, rename_func=rename_func)

        irq_instr = ''
        if not rename_func:
          # if this Decl was tainted, instrument
          for i in dt:
            irq_instr += gen_irq_call(i)
          # if this is the last Decl, and it's a __nesc_atomic_start() call, instrument
          if j == len(node.decls) - 1 and \
             flag and \
             d.init and \
             type(d.init) == c_ast.FuncCall and \
             d.init.name.name == '__nesc_atomic_start':
            for i in sigs:
              irq_instr += gen_irq_call(i)

        dc += irq_instr + ds + '\n'
        j += 1


    return (' '*(offset-2) + '{\n' + dc + sc + ' '*(offset-2) + '}',
            set())

  # Continue: []
  elif tp == c_ast.Continue:
    return (lead + 'continue' + semicolon,
            set())

  # Default: [stmt*]
  elif tp == c_ast.Default:
    return (lead + 'default :\n' + code(node.stmt, offset+2, end_statement=True, rename_func=rename_func)[0],
            set())

  # EllipsisParam: []
  # Represents ellipsis (...) parameter in a function declaration
  elif tp == c_ast.EllipsisParam:
    return ('...', set())

  # Enum: [name, values*], enumeration type specifier
  # - name: an optional ID
  # - values: an EnumeratorList
  elif tp == c_ast.Enum:
    nn = node.name if node.name else ''
    nv = (' { ' + code(node.values)[0] + ' }') if node.values else ''
    return ('enum ' + nn + nv, set())

  # Enumerator: [name, value*]
  # a name/value pair for enumeration values
  elif tp == c_ast.Enumerator:
    nv = (' = ' + code(node.value)[0]) if node.value else ''
    return (node.name + nv, set())

  # EnumeratorList: [enumerators**]
  # a list of enumerators
  elif tp == c_ast.EnumeratorList:
    if node.enumerators:
      es = [code(e)[0] for e in node.enumerators]
      return (', '.join(es), set())
    return ('', set())

  # ExprList: [exprs**]
  #  a list of comma-separated expressions
  elif tp == c_ast.ExprList:
    if node.exprs:

      es = [code(e, offset=offset, end_statement=end_statement, base_struct=base_struct, rename_func=rename_func)[0] for e in node.exprs]
      tt = set()
      for e in node.exprs:
        tt |= code(e, offset=offset, end_statement=end_statement, base_struct=base_struct)[1] 

      return (', '.join(es), tt)

    return ('', set())

  # For: [init*, cond*, next*, stmt*]
  # for (init; cond; next) stmt
  elif tp == c_ast.For:
    c1, t1 = code(node.init, rename_func=rename_func) if node.init else ('', set())
    c2, t2 = code(node.cond, rename_func=rename_func) if node.cond else ('', set())
    c3, t3 = code(node.next, rename_func=rename_func) if node.next else ('', set())
    c4, t4 = code(node.stmt, offset=offset+2, rename_func=rename_func) if node.stmt else ('', set())

    ic = lead + 'for(' + c1 + '; '
    cc = c2 + '; '
    nc = c3 + ')\n'
    tc = c4

    return (ic + cc + nc + tc + '\n',
            t1 | t2 | t3)

  # FuncCall: [name*, args*]
  # - name: Id
  # - args: ExprList
  elif tp == c_ast.FuncCall:
    fn = code(node.name, rename_func=rename_func)[0]
    a1, tc = code(node.args, rename_func=rename_func) if node.args else ('', set())
    ac = '(' + a1 + ')'
    return (lead + fn + ac + semicolon,
            tc)

  # Goto: [name]
  elif tp == c_ast.Goto:
    return (lead + (('goto ' + node.name) if node.name else '') + semicolon,
            set())

  # ID: [name]
  elif tp == c_ast.ID:
    nc = node.name

    if node.name in fixed_vars:
      hexaddr = fixed_vars[node.name] # a string
      decaddr = int(hexaddr, 16)
      nc = MSP430_map[hexaddr]

      print 'DEREF at %s/%s\t @%s\t with fixed-address variable %s' \
            %(decaddr, hexaddr, get_memory_section(decaddr), node.name)

    tainted = set()
    if cdata:
      for i in cdata.keys():
        if nc in cdata[i]:
          tainted.add(i)

    if rename_func and cfunc and (nc in cfunc):
      nc = 'SIG_' + nc

    #print 'ID \"%s\" taints: %s' %(nc, tainted)

    return (nc + semicolon, tainted)

  # If: [cond*, iftrue*, iffalse*]
  elif tp == c_ast.If:
    c1, t1 = code(node.cond, rename_func=rename_func) if node.cond else ('', set())
    c2, t2 = code(node.iftrue, offset=offset+2, rename_func=rename_func) if node.iftrue else ('', set())
    c3, t3 = code(node.iffalse, offset=offset+2, rename_func=rename_func) if node.iffalse else ('', set())

    cc = lead + 'if(' + c1 + ')\n'
    tc = c2
    fc = ('\n' + lead + 'else \n' + c3) if node.iffalse else ''

    return (cc + tc + fc, t1)

  # While: [cond*, stmt*]
  elif tp == c_ast.While:
    c2, t2 = code(node.cond, rename_func=rename_func) if node.cond else ('', set())
    c4, t4 = code(node.stmt, offset=offset+2, rename_func=rename_func) if node.stmt else ('', set())

    return (lead + 'while(' + c2 + ')\n' + (c4 or (lead+semicolon)),
            t2)

  # DoWhile: [cond*, stmt*]
  elif tp == c_ast.DoWhile:
    c2, t2 = code(node.cond, rename_func=rename_func) if node.cond else ('', set())
    c4, t4 = code(node.stmt, offset=offset+2, rename_func=rename_func) if node.stmt else ('', set())

    return (lead + 'do\n' + c4 + '\n' + lead + 'while(' + c2 + ')' + semicolon,
            t2)

  # Label: [name, stmt*]
  elif tp == c_ast.Label:
    sc, st = code(node.stmt, rename_func=rename_func) if node.stmt else ('', set())

    return (node.name + ': ' + sc,
            st)

  # Return: [expr*]
  elif tp == c_ast.Return:
    ec, et = code(node.expr, rename_func=rename_func) if node.expr else ('', set())
    ne = (' ' + ec) if node.expr else ''
    return (lead + 'return' + ne + semicolon,
            et)

  # StructRef: [name*, type, field*]
  # - type: . or -> (name.field or name->field)
  # name can be missing for designated initializers
  elif tp == c_ast.StructRef:
    nc, nt = code(node.name, rename_func=rename_func) if node.name else ('', set())

    if (not node.name) and base_struct:
      nc = base_struct;

    return (nc + node.type + code(node.field, rename_func=rename_func)[0],
            nt)

  # Switch: [cond*, stmt*]
  elif tp == c_ast.Switch:
    cc, ct = code(node.cond, rename_func=rename_func)

    return ((lead + 'switch (' + cc + ') \n' + code(node.stmt, offset+2, rename_func=rename_func)[0] + '\n'),
            ct)

  # TernaryOp: [cond*, iftrue*, iffalse*]
  # cond ? iftrue : iffalse
  elif tp == c_ast.TernaryOp:
    c1, t1 = code(node.cond, rename_func=rename_func) if node.cond else ('', set())
    c2, t2 = code(node.iftrue, rename_func=rename_func) if node.iftrue else ('', set())
    c3, t3 = code(node.iffalse, rename_func=rename_func) if node.iffalse else ('', set())

    return ((c1 + '? ' + c2 + ' : ' + c3) + semicolon,
            t1 | t2 | t3)

  # Typename: [quals, type*]
  elif tp == c_ast.Typename:
    return (''.join(node.quals) + ' ' + code(node.type)[0],
            set())

  # UnaryOp: [op, expr*]
  elif tp == c_ast.UnaryOp:

    c1, t1 = code(node.expr, rename_func=rename_func) if node.expr else ('', set())

    if node.op == 'p++' or node.op == 'p--':
      uc = c1 + node.op[1:]
    elif node.op == 'sizeof':
      uc = node.op + '(' + c1 + ')'
    else:
      uc = node.op + c1

    # if it's a dereference of a constant-to-pointer cast
    if node.op == '*':
      ne = node.expr
      if type(ne) == c_ast.Cast:
        if    type(ne.to_type) == c_ast.Typename \
          and type(ne.expr) == c_ast.Constant \
          and type(ne.to_type.type) == c_ast.PtrDecl:

          # map the constant (e.g. .value '49U') to hex address
          decaddr = int(ne.expr.value[0:-1])
          hexaddr = hex(decaddr)
   
          print 'DEREF at %s/%s\t @ %s\t in line %s' \
                %(decaddr, hexaddr, get_memory_section(decaddr), uc)

          uc = c1

    return (lead + uc + semicolon, 
            set())
