;; Test of map
;; Liam Healy 2010-01-01 12:51:51EST map.lisp
;; Time-stamp: <2010-07-18 00:33:55EDT map.lisp>
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

(lisp-unit:define-test extrude
  (lisp-unit:assert-numerical-equal
   '((0.4d0 2.4d0 4.4d0)
     (10.4d0 12.4d0 14.4d0)
     (20.4d0 22.4d0 24.4d0))
   (let ((mat1
	  (map-grid :source (offset-ifd 0.1d0)
		    :destination-specification '((array 3 3) double-float)))
	 (vec (map-grid :source (offset-ifd 0.3d0)
			:destination-specification '((array 3) double-float))))
     (contents
      (map-n-grids
       :sources
       `((,vec ,(affi:extrude (affi vec) 0 3))
	 (,mat1 nil))
       :destination-specification (specification mat1)
       :combination-function '+
       :combine-destination nil))))
  (lisp-unit:assert-numerical-equal
   '((0.3d0 3.4d0 8.5d0)
     (20.3d0 33.4d0 48.5d0)
     (40.3d0 63.4d0 88.5d0))
   (let ((mat1
	  (map-grid :source (offset-ifd 0.1d0)
		    :destination-specification '((array 3 3) double-float)))
	 (mat2 (map-grid :source (offset-ifd 0.2d0)
			 :destination-specification '((array 3 3) double-float)))
	 (vec (map-grid :source '1+
			:destination-specification '((array 3) double-float))))
     (contents
      (map-n-grids
       :sources
       `((,vec ,(affi:extrude (affi vec) 0 3))
	 (,mat1 nil)
	 (,mat2 nil))
       :destination-specification (specification mat2)
       :combination-function (lambda (a b c) (+ (* a b) c))
       :combine-destination nil)))))
 
