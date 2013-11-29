;; Exponential of a matrix
;; Liam Healy 2008-08-10 17:25:35EDT exponential.lisp
;; Time-stamp: <2010-07-07 14:25:00EDT exponential.lisp>
;;
;; Copyright 2008, 2009 Liam M. Healy
;; Distributed under the terms of the GNU General Public License
;;
;; This program is free software: you can redistribute it and/or modify
;; it under the terms of the GNU General Public License as published by
;; the Free Software Foundation, either version 3 of the License, or
;; (at your option) any later version.
;;
;; This program is distributed in the hope that it will be useful,
;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;; GNU General Public License for more details.
;;
;; You should have received a copy of the GNU General Public License
;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

(in-package :gsl)

;;; No documentation
(defmfun matrix-exponential (matrix exponential &optional (mode :double))
  "gsl_linalg_exponential_ss"
  (((mpointer matrix) :pointer) ((mpointer exponential) :pointer)
   (mode sf-mode))
  :inputs (matrix)
  :outputs (exponential)
  :documentation
  "Calculate the matrix exponential by the scaling and
   squaring method described in Moler + Van Loan,
   SIAM Rev 20, 801 (1978).  The matrix to be exponentiated
   is matrix, the returned exponential is exponential.
   The mode argument allows
   choosing an optimal strategy, from the table
   given in the paper, for a given precision.")

#|
;;; A matrix of the form [[0, 1], [-1, 0]]
;;; when exponentiated gives [[cos x,  sin x], [-sin x,  cos x]]
(let ((mat #m(0.0d0 1.0d0 ^ -1.0d0 0.0d0))
       (exp (grid:make-foreign-array 'double-float :dimensions '(2 2))))
  (grid:copy-to (matrix-exponential mat exp)))
#2A((0.5403023058681384d0 0.841470984807895d0)
    (-0.841470984807895d0 0.5403023058681384d0))
|#
