;; Tests of array functions 
;; Liam Healy 2008-10-20 22:41:48EDT array-tests.lisp
;; Time-stamp: <2010-07-07 14:25:02EDT array-tests.lisp>
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

;;; Generate each file with #'write-test-to-file, e.g.
;;; (write-test-to-file 'matrix-set-zero "../test/")

(in-package :gsl)

;;; Some of these don't work for complex because:
;;; Cannot pass complex scalars to and from GSL functions (structs passed by value).

;;;;****************************************************************************
;;;; Bulk operations
;;;;****************************************************************************

(generate-all-array-tests vector-set-all #+fsbv t #-fsbv :no-complex
 (let ((v1 (array-default 3 t)))
   (grid:copy-to (set-all v1 (scalar-default)))))

(generate-all-array-tests matrix-set-all #+fsbv t #-fsbv :no-complex
 (let ((m1 (array-default '(3 3) t)))
   (grid:copy-to (set-all m1 (scalar-default)))))

(generate-all-array-tests vector-set-zero t
 (let ((v1 (array-default 3)))
   (set-zero v1)
   (grid:copy-to v1)))

(generate-all-array-tests matrix-set-zero t
 (let ((m1 (array-default '(3 3))))
   (set-zero m1)
   (grid:copy-to m1)))

(generate-all-array-tests vector-swap t
 (let ((v1 (array-default 3))
       (v2 (array-default 3)))
   (swap v2 v1)
   (list (grid:copy-to v1) (grid:copy-to v2))))

(generate-all-array-tests matrix-swap t
 (let ((m1 (array-default '(3 3)))
       (m2 (array-default '(3 3))))
   (swap m2 m1)
   (list (grid:copy-to m1) (grid:copy-to m2))))

;;;;****************************************************************************
;;;; Arithmetic operations
;;;;****************************************************************************

;;; In GSL versions >= 1.12, :no-complex can be changed to T.

(generate-all-array-tests vector-add :no-complex
 (let ((v1 (array-default 3))
       (v2 (array-default 3)))
   (elt+ v1 v2)))

(generate-all-array-tests matrix-add :no-complex
 (let ((m1 (array-default '(3 3)))
       (m2 (array-default '(3 3))))
   (grid:copy-to (elt+ m1 m2))))

(generate-all-array-tests vector-sub :no-complex
 (let ((v1 (array-default 3))
       (v2 (array-default 3)))
   (grid:copy-to (elt- v1 v2))))

(generate-all-array-tests matrix-sub :no-complex
 (let ((m1 (array-default '(3 3)))
       (m2 (array-default '(3 3))))
   (grid:copy-to (elt- m1 m2))))

(generate-all-array-tests vector-mult :no-complex
 (let ((v1 (array-default 3))
       (v2 (array-default 3)))
   (grid:copy-to (elt* v1 v2))))

(generate-all-array-tests matrix-mult :no-complex
 (let ((m1 (array-default '(3 3)))
       (m2 (array-default '(3 3))))
   (grid:copy-to (elt* m1 m2))))

(generate-all-array-tests vector-div :no-complex
 (let ((v1 (array-default 3))
       (v2 (array-default 3)))
   (grid:copy-to (elt/ v1 v2))))

(generate-all-array-tests matrix-div :no-complex
 (let ((m1 (array-default '(3 3)))
       (m2 (array-default '(3 3))))
   (grid:copy-to (elt/ m1 m2))))

(generate-all-array-tests vector-mult-scalar :no-complex
 (let ((v1 (array-default 3)))
   (grid:copy-to (elt* v1 1.39d0))))

(generate-all-array-tests matrix-mult-scalar :no-complex
 (let ((m1 (array-default '(3 3))))
   (grid:copy-to (elt* m1 1.39d0))))

(generate-all-array-tests vector-add-scalar :no-complex
 (let ((v1 (array-default 3)))
   (grid:copy-to (elt+ v1 18.19d0))))

(generate-all-array-tests matrix-add-scalar :no-complex
 (let ((m1 (array-default '(3 3))))
   (grid:copy-to (elt+ m1 18.19d0))))

;;;;****************************************************************************
;;;; Maximum and minimum elements
;;;;****************************************************************************

(generate-all-array-tests vector-max :no-complex
 (let ((v1 (array-default 3)))
   (mmax v1)))

(generate-all-array-tests matrix-max :no-complex
 (let ((m1 (array-default '(3 3))))
   (mmax m1)))

(generate-all-array-tests vector-min :no-complex
 (let ((v1 (array-default 3)))
   (mmin v1)))

(generate-all-array-tests matrix-min :no-complex
 (let ((m1 (array-default '(3 3))))
   (mmin m1)))

(generate-all-array-tests vector-minmax :no-complex
 (let ((v1 (array-default 3)))
   (minmax v1)))

(generate-all-array-tests matrix-minmax :no-complex
 (let ((m1 (array-default '(3 3))))
   (minmax m1)))

(generate-all-array-tests vector-min-index :no-complex
 (let ((v1 (array-default 8)))
   (min-index v1)))

(generate-all-array-tests matrix-min-index :no-complex
 (let ((m1 (array-default '(3 3))))
   (min-index m1)))

(generate-all-array-tests vector-max-index :no-complex
 (let ((v1 (array-default 8)))
   (max-index v1)))

(generate-all-array-tests matrix-max-index :no-complex
 (let ((m1 (array-default '(3 3))))
   (max-index m1)))

(generate-all-array-tests vector-minmax-index :no-complex
 (let ((v1 (array-default 8)))
   (minmax-index v1)))

(generate-all-array-tests matrix-minmax-index :no-complex
 (let ((m1 (array-default '(3 3))))
   (minmax-index m1)))

;;; No test for mzerop yet.

;;;;****************************************************************************
;;;; Vector-only function definitions
;;;;****************************************************************************

(generate-all-array-tests set-basis t
 (let ((v1 (array-default 8)))
   (set-basis v1 2)
   (grid:copy-to v1)))

(generate-all-array-tests swap-elements t
 (let ((v1 (array-default 8)))
   (swap-elements v1 2 5)
   (grid:copy-to v1)))

(generate-all-array-tests vector-reverse t
 (let ((v1 (array-default 8)))
   (grid:copy-to (vector-reverse v1))))

;;;;****************************************************************************
;;;; Matrix-only function definitions
;;;;****************************************************************************

(generate-all-array-tests set-identity t
 (let ((m1 (array-default '(3 3))))
   (grid:copy-to (set-identity m1))))

(generate-all-array-tests row t
 (let ((m1 (array-default '(3 3)))
       (row (array-default 3 t)))
   (grid:copy-to (row m1 1 row))))

(generate-all-array-tests setf-row t
 (let ((m1 (array-default '(3 3)))
	(row (array-default 3)))
   (setf (row m1 2) row)
   (grid:copy-to m1)))

(generate-all-array-tests column t
 (let ((m1 (array-default '(3 3)))
       (col (array-default 3 t)))
   (grid:copy-to (column m1 1 col))))

(generate-all-array-tests setf-column t
 (let ((m1 (array-default '(3 3)))
	(col (array-default 3)))
   (setf (column m1 2) col)
   (grid:copy-to m1)))

(generate-all-array-tests swap-rows t
 (let ((m1 (array-default '(3 3))))
   (grid:copy-to (swap-rows m1 0 1))))

(generate-all-array-tests swap-columns t
 (let ((m1 (array-default '(3 3))))
   (grid:copy-to (swap-columns m1 1 2))))

(generate-all-array-tests swap-row-column t
 (let ((m1 (array-default '(3 3))))
   (grid:copy-to (swap-row-column m1 0 2))))

(generate-all-array-tests matrix-transpose* t
 (let ((m1 (array-default '(3 3))))
   (grid:copy-to (matrix-transpose* m1))))

(generate-all-array-tests matrix-transpose t
 (let ((m1 (array-default '(3 3)))
       (m2 (array-default '(3 3) t)))
   (grid:copy-to (matrix-transpose m1 m2))))
