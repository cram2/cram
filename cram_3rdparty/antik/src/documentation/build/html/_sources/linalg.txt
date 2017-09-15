Linear Algebra
==============

To find the inverse and determinant of a matrix, use ``invert-matrix`` with a two-dimensional ``foreign-array``::

    > (invert-matrix
       (grid:make-foreign-array 
        'double-float :initial-contents '((1.0d0 2.0d0) (3.0d0 4.0d0))))
    #m((-2.000000000000000d0 1.000000000000000d0)
       (1.500000000000000d0 -0.500000000000000d0))
    -2.0

Since a foreign library (GSL) is used for this calculation, the matrix must be a foreign array.

.. note::

   At present, these call the LU methods in GSLL. Plans are for these functions to take arguments to allow other methods and libraries. 

.. cl:package:: antik
.. cl:function:: invert-matrix
.. cl:function:: determinant
.. cl:function:: solve-linear
