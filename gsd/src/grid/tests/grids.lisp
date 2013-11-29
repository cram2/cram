;; Test grids
;; Liam Healy 2009-11-28 09:47:35EST grids.lisp
;; Time-stamp: <2010-07-03 22:55:04EDT grids.lisp>
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

(defun index-fill-decadal (&rest args)
  "Element will be computed from the indices by multiplying
   by powers of 10 and summing."
  (loop with dims = (length args)
     for i below dims
     sum (* (expt 10 (- dims 1 i)) (nth i args))))

(defun offset-ifd (offset)
  (lambda (&rest args) (+ offset (apply 'index-fill-decadal args))))

(defun test-grid-double-float
    (type dimensions &optional (index-fill-function 'index-fill-decadal))
  "Make a grid of the specified type and dimensions,
   where the contents are computed by the index-fill-function."
  (map-grid :source index-fill-function
	    :source-dims dimensions
	    :destination-specification
	    (make-specification type dimensions 'double-float)))
