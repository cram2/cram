Getting Started
=====================

.. highlight:: lisp

Antik is normally installed using Quicklisp_. Once Antik is loaded, it is advisable to set the default package to ``ANTIK-USER`` which is provided to make using Antik easier; see :ref:`systems-packages` for more details. Reader macros are defined by Antik for convenience in specifying physical dimension quantities, dates and times, and grids. These can be enabled with ``set-reader``. This also sets the default floating point read type to ``double-float``::

   (ql:quickload :antik)
   (in-package :antik-user)
   (set-reader) 
  
As a reassurance that the system is installed and working correctly, run the defined regression tests::

    > (asdf:test-system :antik)

     Unit Test Summary
      | 93 assertions total
      | 93 passed
      | 0 failed
      | 0 execution errors
      | 0 missing tests

     Unit Test Summary
      | 88 assertions total
      | 88 passed
      | 0 failed
      | 0 execution errors
      | 0 missing tests

     T    

The first tests definitions in the ``ANTIK`` package and the second in the ``GRID`` package. In examples given in this manual, the ``>`` at the begining of the line is the Lisp REPL_ prompt, indicated the user-typed input follows. Your prompt may differ; for example, it could be the name of the current package, e.g. ``ANTIK-USER>``.

.. _Quicklisp: http://quicklisp.org/
.. _REPL: https://en.wikipedia.org/wiki/Read%E2%80%93eval%E2%80%93print_loop
.. cl:package:: antik
.. cl:function:: set-reader
.. cl:macro:: set-reader-in-file
