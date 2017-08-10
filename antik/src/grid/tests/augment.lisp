;; Additional methods for lisp-unit
;; Liam Healy 2009-04-15 23:23:30EDT augment.lisp
;; Time-stamp: <2011-05-23 22:50:53EDT augment.lisp>
;;
;; Copyright 2009 Liam M. Healy
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

(in-package :grid)

(defun grid-numerical-equal
    (grid1 grid2
     &key (test #'lisp-unit:number-equal) (conformable-only nil))
  "Return true if the grids are numerically equal according to :TEST.
   If conformable-only is :strict or :dropped, grids need not have
   the same specification, just be conformable, either strictly or
   with singleton dimensions dropped."
  (and (if conformable-only
	   (affi:check-conformability (affi grid1) (affi grid2) conformable-only)
	   (equal (specification grid1) (specification grid2)))
       (let* ((walker1 (affi:make-walker (affi grid1)))
	      (index1 nil)
	      (walker2 (affi:make-walker (affi grid2)))
	      (index2 nil)
	      (result t))
	 (block nil
	   (tagbody
	    loop-top-nil
	      (setq index1
		    (or (funcall walker1) (go loop-end-nil))
		    index2
		    (or (funcall walker2) (go loop-end-nil)))
	      (or
	       (setq result
		     (funcall test (aref* grid1 index1) (aref* grid2 index2)))
	       (return-from nil nil))
	      (go loop-top-nil)
	    loop-end-nil)
	   result))))

(defparameter *conformability* nil
  "In tests with lisp-unit:numerical-equal, default :conformable-only
   argument.")

#|
;;; This will override the method given in lisp-unit.
(defmethod lisp-unit:numerical-equal
    ((grid1 array) (grid2 array)
     &key (test #'lisp-unit:number-equal) (conformable-only *conformability*))
  (grid-numerical-equal
   grid1 grid2 :test test :conformable-only conformable-only))
|#
