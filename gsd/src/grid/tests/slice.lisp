;; Tests of slice
;; Liam Healy 2009-12-05 20:25:35EST slice.lisp
;; Time-stamp: <2010-06-20 11:08:51EDT slice.lisp>
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

(lisp-unit:define-test slice
  (lisp-unit:assert-numerical-equal
   '((10.0d0 11.0d0 12.0d0))
   (contents
    (slice (test-grid-double-float 'array '(3 4))
	   '(1 (:range 0 2)) :drop nil)))
  (lisp-unit:assert-numerical-equal
   '(10.0d0 11.0d0 12.0d0)
   (contents
    (slice (test-grid-double-float 'array '(3 4))
	   '(1 (:range 0 2)) :drop t)))
  (lisp-unit:assert-numerical-equal
   '((10.0d0 11.0d0 12.0d0) (20.0d0 21.0d0 22.0d0))
   (contents
    (slice (test-grid-double-float 'array '(3 4))
	   '((:range 1 2) (:range 0 2)))))
  (lisp-unit:assert-numerical-equal
   '((2.0d0 3.0d0) (12.0d0 13.0d0) (22.0d0 23.0d0))
   (contents
    (slice (test-grid-double-float 'array '(3 4))
	   '(:all (:select 2 3)) :drop t)))
  (lisp-unit:assert-numerical-equal
   '((22.0d0 23.0d0) (12.0d0 13.0d0) (2.0d0 3.0d0))
   (contents
    (slice (test-grid-double-float 'array '(3 4))
	   '(:rev (:select 2 3)) :drop t)))
  (lisp-unit:assert-numerical-equal
   '((1.0d0 3.0d0) (21.0d0 23.0d0))
   (contents
    (slice (test-grid-double-float 'array '(3 4))
	   '((:select 0 2) (:select 1 3)) :drop t)))
  (lisp-unit:assert-numerical-equal
   '((10.0d0 12.0d0) (20.0d0 22.0d0))
   (contents (slice (test-grid-double-float 'array '(3 4))
		    '((:range 1 2) (:range 0 3 2))))))
