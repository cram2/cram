Parameters
=================================

Parameters are named values that are defined and used in computations or in presentation.  The defparameter_ macro defines parameters (dynamic variables) but Antik has definitions that permit grouping them and including properties such as the type, which will be checked when values are assigned, and defining synonyms.  The grouping is in named categories; Antik itself defines one category, ``NF``, which is used for numerical formatting.  Each category used will become a package, and the symbols exported from that package. The value can be changed with ``setf`` :cl:function:`parameter-value` and used with :cl:function:`parameter-value`, or changed locally (analogous to let_ for CL variables) with :cl:function:`with-parameters`. It is also permissible to assign as a normal variable (e.g., with let and setf), but no type checking is performed.

Define some parameters::

    (define-parameter kepler foo
      :value 122
      :type fixnum
      :documentation "A fixnum parameter of kepler.")
    (define-parameter kepler bar
      :value "hi"
      :type string
      :documentation "A string parameter of kepler.")

Get their values::

    (parameter-value kepler bar)
    "hi"
    (parameter-value kepler foo)
    122

    ;;; Dynamic binding
    (defun show-foo-bar ()
      (format t "~&foo: ~a, bar: ~s"
    	  (parameter-value kepler foo)
    	  (parameter-value kepler bar)))

    (show-foo-bar)
    foo: 122, bar: "hi"
    NIL

    ;;; Locally change values
    (with-parameters (kepler (foo 143) (bar "bye"))
      (show-foo-bar))
    foo: 143, bar: "bye"
    NIL

    (show-foo-bar)
    foo: 122, bar: "hi"
    NIL

Make a mistake::

    (with-parameters (kepler (foo 143) (bar -44))
      (show-foo-bar))
    Error: Value -44 is of type FIXNUM, not of the required type STRING.

Globally change values::

    (setf (parameter-value kepler bar) "a new value")
    (show-foo-bar)
    foo: 122, bar: "a new value"

Set multiple values::

    (set-parameters kepler bar "xyz" foo 1)
    (show-foo-bar)
    foo: 1, bar: "xyz"

Get information about the categories and parameters::

    (parameter-help)
    Parameter categories: KEPLER and NF.

    (parameter-help :kepler)
    Parameters in KEPLER: BAR and FOO.

    (parameter-help :kepler :bar)
    BAR: A string parameter of kepler.
    Type is STRING,
    Current value is "xyz".

.. _defparameter: http://www.lispworks.com/documentation/HyperSpec/Body/m_defpar.htm#defparameter,defparameter
.. _let: http://www.lispworks.com/documentation/HyperSpec/Body/s_let_l.htm

.. cl:package:: antik
.. cl:function:: define-parameter 
.. cl:function:: make-parameter 
.. cl:function:: make-parameters-from-table
.. cl:function:: parameter-help
.. cl:function:: parameter-value
.. cl:function:: set-parameter
.. cl:function:: set-parameters
.. cl:function:: with-parameters
