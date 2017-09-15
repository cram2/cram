Antik: a Common Lisp library for computational mathematics, science, and engineering.  It is named after the Antikythera Mechanism [http://www.antikythera-mechanism.gr/], one of the oldest known devices built and used for scientific computation.

Antik defines library functions for common mathematical operations used in science and engineering including arrays and physical (dimensioned) quantities.  It is expressly designed to facilitate the interchange of data with foreign (that is, non-lisp) libraries.  For example, GSLL, the GNU Scientific Library for Lisp, uses Antik.

There are several systems that comprise Antik and included in this repository:
- antik-base, formatting (numbers, etc.)
- grid, generalized arrays
- foreign-array, grids whose content is in foreign memory
- physical-dimension, quantities with physical properties (e.g., mass, length) and relationships
- mathematics, various definitions for applied mathematics
- date-time, manipulation and computation with dates and times

In addition, there are other systems used internally.
