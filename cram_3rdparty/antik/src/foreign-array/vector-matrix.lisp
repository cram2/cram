;; Define vector and matrix classes
;; Liam Healy 2010-06-26 19:16:09EDT vector-matrix.lisp
;; Time-stamp: <2010-06-26 19:16:37EDT vector-matrix.lisp>

(in-package :grid)

;;;;****************************************************************************
;;;; Vectors and matrices
;;;;****************************************************************************

(export 'mvector)
(defclass mvector (foreign-array)
  ()
  (:documentation "Foreign vectors."))

;;; Define all supported mvector subclasses
#.(data-defclass 'vector 'mvector)

(export 'matrix)
(defclass matrix (foreign-array)
  ()
  (:documentation "Foreign matrices."))

;;; Define all supported matrix subclasses
#.(data-defclass 'matrix 'matrix)
