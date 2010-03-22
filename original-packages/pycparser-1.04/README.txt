===============
pycparser v1.04
===============

:Author: `Eli Bendersky <http://eli.thegreenplace.net>`_


.. contents::

Introduction
============

What is pycparser?
------------------

``pycparser`` is a parser for the C language, written in pure Python. It is a module designed to be easily integrated into applications that need to parse C source code.

What is it good for?
--------------------

Anything that needs C code to be parsed. Personally, I'm using ``pycparser`` to write a  compiler from C into a proprietary assembly language. I've also used it for writing C code "comprehension helpers" - tracking assignments to global variables and other variables that depend upon them throughout a code base.

But I can imagine other interesting uses for it - writing semantic analysis tools, static checkers, experimenting with modifying C's syntax. ``pycparser`` is unique in the sense that it's written in pure Python - a very high level language that's easy to experiment with and tweak. To people familiar with Lex and Yacc, ``pycparser``'s code will be simple to understand.


Which version of C does pycparser support?
------------------------------------------

At the moment, ``pycparser`` supports ANSI/ISO C89, the language described by Kernighan and Ritchie in "The C Programming language, 2nd edition" (K&R2), with only selected extensions from C99. The currently supported C99 features are:

* Allowing a comma after the last value in an enumeration list

``pycparser`` doesn't support any GCC extensions.

What grammar does pycparser follow?
-----------------------------------

``pycparser`` very closely follows the ANSI C grammar provided in the end of K&R2. Listings of this grammar (often in Yacc syntax) can be easily found by a simple web search. Google for `ansi c grammar` to get started.


What is an AST?
---------------

`AST <http://en.wikipedia.org/wiki/Abstract_syntax_tree>`_ - Abstract Syntax Tree. It is a tree representation of the syntax of source code - a convenient hierarchical data structure that's built from the code and is readily suitable for exploration and manipulation.

How is pycparser licensed?
--------------------------

`LGPL <http://www.gnu.org/licenses/lgpl.html>`_

Contact details
---------------

Drop me an email to eliben@gmail.com for any questions regarding ``pycparser``.


Installing
==========

Prerequisites
-------------

* ``pycparser`` was tested on Python 2.5 and 2.6
* ``pycparser`` uses the PLY module for the actual lexer and parser construction. You'll also need to install PLY version 3.2 (earlier versions work at least since 2.5) from `its website <http://www.dabeaz.com/ply/>`_.
* If you want to modify ``pycparser``'s code, you'll need to install `PyYAML <http://pyyaml.org/>`_, since it's used by ``pycparser`` to store the AST configuration in a YAML file.

Installation process
--------------------

Installing ``pycparser`` is very simple. Once you download it from its `website <http://code.google.com/p/pycparser/>`_ and unzip the package, you just have to execute the standard ``python setup.py install``. The setup script will then place the ``pycparser`` module into ``site-packages`` in your Python's installation library.

It's recommended to run ``_build_tables.py`` in the ``pycparser`` code directory to make sure the parsing tables of PLY are pre-generated. This can make your code run faster.


Using
=====

Interaction with the C preprocessor
-----------------------------------

In order to be compilable, C code must be preprocessed by the C preprocessor - ``cpp``. ``cpp`` handles preprocessing directives like ``#include`` and ``#define``, removes comments, and does other minor tasks that prepare the C code for compilation.

For all but the most trivial snippets of C code, ``pycparser``, like a C compiler, must receive preprocessed C code in order to function correctly. If you import the top-level ``parse_file`` function from the ``pycparser`` package, it will interact with ``cpp`` for you, as long as it's in your PATH, or you provide a path to it. 

On the vast majority of Linux systems, ``cpp`` is installed and is in the PATH. If you're on Windows and don't have ``cpp`` somewhere, you can use the one provided in the ``utils`` directory in ``pycparser``'s distribution. This ``cpp`` executable was compiled from the `LCC distribution <http://www.cs.princeton.edu/software/lcc/>`_, and is provided under LCC's license terms.

What about the standard C library headers?
------------------------------------------

C code almost always includes various header files from the standard C library, like ``stdio.h``. While, with some effort, ``pycparser`` can be made to parse the standard headers from any C compiler, it's much simpler to use the provided "fake" standard in includes in ``utils/fake_libc_include``. These are standard C header files that contain only the bare necessities to allow valid compilation of the files that use them. As a bonus, since they're minimal, it can significantly improve the performance of parsing C files.

See the ``using_cpp_libc.py`` example for more details.

Basic usage
-----------

Take a look at the ``examples`` directory of the distribution for a few examples of using ``pycparser``. These should be enough to get you started.

Advanced usage
--------------

The public interface of ``pycparser`` is well documented with comments in ``pycparser/c_parser.py``. For a detailed overview of the various AST nodes created by the parser, see ``pycparser/_c_ast.yaml``.

In any case, you can always drop me an `email <eliben@gmail.com>`_ for help.

Modifying
=========

There are a few points to keep in mind when modifying ``pycparser``:

* The code for ``pycparser``'s AST nodes is automatically generated from a YAML configuration file - ``_c_ast.yaml``, by ``_ast_gen.py``. If you modify the AST configuration, make sure to re-generate the code.
* Make sure you understand the optimized mode of ``pycparser`` - for that you must read the docstring in the constructor of the ``CParser`` class. For development you should create the parser without optimizations, so that it will regenerate the Yacc and Lex tables when you change the grammar.
* The script ``_build_tables.py`` can be helpful - it regenerates all the tables needed by ``pycparser``, and the AST code from YAML.


Package contents
================

Once you unzip the ``pycparser`` package, you'll see the following files and directories:

README.txt/html:
  This README file.

setup.py:
  Installation script

examples/:
  A directory with some examples of using ``pycparser``

pycparser/:
  The ``pycparser`` module source code.

tests/:
  Unit tests.

utils/cpp.exe:
  A Windows executable of the C pre-processor suitable for working with pycparser

utils/fake_libc_include:
  Minimal standard C library include files that should allow to parse any C code.

utils/internal/:
  Internal utilities for my own use. You probably don't need them.


Changelog
=========

Version 1.04 (22.05.2009)
-------------------------

* Added the ``fake_libc_include`` directory to allow parsing of C code that uses standard C library include files without dependency on a real C library.
* Tested with Python 2.6 and PLY 3.2


Version 1.03 (31.01.2009)
-------------------------

* Accept enumeration lists with a comma after the last item (C99 feature).

Version 1.02 (16.01.2009)
-------------------------

* Fixed problem of parsing struct/enum/union names that were named similarly to previously defined ``typedef`` types. 

Version 1.01 (09.01.2009)
-------------------------

* Fixed subprocess invocation in the helper function parse_file - now it's more portable

Version 1.0 (15.11.2008)
------------------------

* Initial release
* Support for ANSI C89




