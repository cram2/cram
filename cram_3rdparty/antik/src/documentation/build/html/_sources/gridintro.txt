The grid concept
------------------

In scientific and engineering computing, data is frequently organized into a regular pattern indexed by a finite sequence of integers, and thought of having a Cartesian arrangement.  In Antik, these are called *grids*, and can be thought of as generalized arrays. One example type is a Common Lisp array_.  Other examples might be a list of lists, a C array accessible through a foreign function interface.

Each grid has a rectangular shape; that is, the range of indices permissible is independent of the other values of the index.  Each node of the grid is an *element* (or sometimes a *component*). Since the focus of Antik is on the scientific and engineering applications, the elements will generally be numbers of some type.  Most of the definitions here however do not force this to be the case.

The *rank* of a grid is the number of Cartesian axes, and the *dimensions* are a sequence of non-negative integers of length equal to the rank that give the number of index values possible along each axis. A grid of rank 1 is called a *vector* (where there would not be a confusion with the Common Lisp vector_), and a grid of rank 2 a *matrix*.  For example, a matrix representing rotations in three dimensional space would have rank 2 and dimensions (3 3).

.. _array: http://www.lispworks.com/documentation/lw50/CLHS/Body/t_array.htm
.. _vector: http://www.lispworks.com/documentation/HyperSpec/Body/t_vector.htm

Types of grids
--------------

Antik defines two types of grids, Lisp *arrays* and *foreign arrays*, which are defined in foreign memory and therefore are accessible by foreign libraries.  The argument ``grid-type`` to a function or the variable ``grid:*default-grid-type*`` should be bound to one of ``'cl:array`` or ``'grid:foreign-array``, to specify an array or
foreign-array respectively.

In order to use foreign arrays, the ``foreign-array`` system [#f1]_ must be loaded::

  (ql:quickload :foreign-array)

This will happen automatically if the ``antik`` system is loaded.

.. [#f1] :ref:`systems-packages`

Package
--------------

All symbols defined in this chapter are in the ``grid`` package and imported into packages defined with :cl:function:`make-user-package`.
