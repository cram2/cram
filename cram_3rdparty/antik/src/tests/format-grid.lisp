;; Test grid formatting.
;; Liam Healy 2011-02-27 17:20:59EST format-grid.lisp
;; Time-stamp: <2015-12-26 10:46:19EST format-grid.lisp>

;; Copyright 2011, 2013, 2015 Liam M. Healy
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

;;; Test formatting of grids (does not require nor does it run in lisp-unit).

(in-package :antik)

(defun test-grid (foreign-array matrix pq)
  "Make a test grid for formatting.
   foreign-array: nil= CL array, T=foreign array
   matrix: nil = vector, T = matrix
   pq: nil = numerical grid, :scalar = all the same units, :mixed = different units."  
  (let ((numgrid 
	  (grid:make-grid-sequential-elements
	   :grid-type (if foreign-array 'grid:foreign-array 'array)
	   :dimensions
	   (if (eq pq :mixed)
	       '(2)
	       (if matrix '(4 4) '(5))))))
    (case pq
      (:scalar (make-pq numgrid 'km))
      (:mixed
       (if matrix
	   (grid:matrix-from-columns (make-pq numgrid 'km) (make-pq numgrid 'km/s))
	   (grid:concatenate-grids (make-pq numgrid 'km) (make-pq numgrid 'km/s))))
      (t numgrid))))

(defparameter *all-test-grids*
  (list
   (test-grid nil nil nil)
   (test-grid nil nil :scalar)
   (test-grid nil nil :mixed)
   (test-grid nil t nil)
   (test-grid nil t :scalar)
   (test-grid nil t :mixed)
   (test-grid t nil nil)
   (test-grid t nil :scalar)
   (test-grid t nil :mixed)
   (test-grid t t nil)
   (test-grid t t :scalar)
   (test-grid t t :mixed))
  "A set of grids covering all combinations of arrays and foreign-arrays, vectors and matrices, and no units, same units, and mixed (different) units.")

(defun test-grid-formatting (grid)
  "Format the 1D grid with all different combinations."
  (if (grid:dim1 grid)
      ;; Matrix formatting
      (progn
	(format t "~&===== Readably =========~&")
	(with-nf-options (:style :readable)
	  (nf grid))
	(format t "~&===== Plain =========~&")
	(with-nf-options (:style nil)
	  (nf grid))
	(format t "~&===== LaTeX, right aligned =========~&")
	(with-nf-options (:style :tex :tex-decimal-align nil)
	  (nf grid))
	(format t "~&===== LaTeX, decimal aligned =========~&")
	(with-nf-options (:style :tex :tex-decimal-align t)
	  (nf grid)))
      ;; Vector formatting
      (progn
	(format t "~&===== Plain, CUV =========~&")
	(with-nf-options
	    (:style nil :vector-format :coordinate-unit-vectors
	     :components '("a" "b" "c" "d" "e" "f"))
	  (nf grid))
	(format t "~&===== LaTeX, CUV =========~&")
	(with-nf-options
	    (:style :tex :vector-format :coordinate-unit-vectors
	     :components '("a" "b" "c" "d" "e" "f"))
	  (nf grid))
	(format t "~&===== Readably, CUV =========~&")
	(with-nf-options (:style :readable :vector-format :coordinate-unit-vectors)
	  (nf grid))
	(format t "~&===== Plain, horizontal =========~&")
	(with-nf-options
	    (:style nil :vector-format :horizontal
	     :components '("a" "b" "c" "d" "e" "f"))
	  (nf grid))
	(format t "~&===== LaTeX, horizontal =========~&")
	(with-nf-options
	    (:style :tex :vector-format :horizontal
	     :components '("a" "b" "c" "d" "e" "f"))
	  (nf grid))
	(format t "~&===== Readably, horizontal =========~&")
	(with-nf-options (:style :readable :vector-format :horizontal)
	  (nf grid))
	(format t "~&===== Plain, vertical =========~&")
	(with-nf-options
	    (:style nil :vector-format :vertical
	     :components '("a" "b" "c" "d" "e" "f"))
	  (nf grid))
	(format t "~&===== LaTeX, vertical =========~&")
	(with-nf-options
	    (:style :tex :vector-format :vertical
	     :components '("a" "b" "c" "d" "e" "f"))
	  (nf grid))
	(format t "~&===== Readably, vertical =========~&")
	(with-nf-options (:style :readable :vector-format :vertical)
	  (nf grid)))))

