Introduction to Antik
=====================

Antik provides a foundation for scientific and engineering computation in Common Lisp.  It is designed not only to facilitate numerical computations, but to permit the use of numerical computation libraries and the interchange of data and procedures, whether foreign (non-Lisp) or Lisp libraries.  Notably, GSLL_ provides an interface to the GNU Scientific Library (GSL_) and is based on Antik.

Antik is designed and developed to provide a common foundation and interoperability between scientific, engineering and mathematical libraries, whether in Lisp or not.  Interoperability means that objects created can easily be passed to one or more libraries, and libraries can be mixed and combined to solve a problem.  It also means that names of like functions will be the same, differing only in the package.  For example, if systems ``foo`` and ``bar`` both provide an LU decomposition, that function will be ``foo:lu-decomposition`` in one and ``bar:lu-decomposition`` in the other.  Should a user wish to compare or switch libraries in such a function call, it is a simple matter of changing the package.  If a whole library should be switched, the names can be used without a package prefix and the use-package form changed.  This makes it easy to compare results, and select and mix the best libraries for a calculation.

.. _GSLL: http://common-lisp.net/project/gsll/
.. _GSL: http://www.gnu.org/software/gsl/
