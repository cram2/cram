;; Polar and rectangular coordinates in three dimensions.                         
;; Liam Healy Sat Mar 17 2001 - 15:21
;; Time-stamp: <2011-05-23 22:50:56EDT polar.lisp>

;; Copyright 2011 Liam M. Healy
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

(in-package :antik)

(export '(polar-to-rectangular rectangular-to-polar))

;;;;============================================================================
;;;; Mathematical
;;;;============================================================================

;;; These should be moved to mathematics/numerical.
;;; Vallado (3-1) and (3-3)

(defun polar-to-rectangular (azimuth elevation &optional (radius 1.0) (sign1 1))
  "Convert polar coordinates to rectangular coordinates.
   Argument sign1 should be set to -1 for accomodating topocentric azimuth,
   which is measured from North instead of South."
  (with-pq ((azimuth angle) (elevation angle))
    (let ((coselev (cos elevation)))
      (grid:make-simple-grid
       :dimensions 3
       :initial-contents
       (list (antik:* sign1 radius coselev (cos azimuth))
	     (antik:* radius coselev (sin azimuth))
	     (antik:* radius (sin elevation)))))))

(defun rectangular-to-polar (vector &optional (sign1 1))
  "Convert rectangular coordinates to polar coordinates.
   Argument sign1 should be set to -1 for accomodating topocentric azimuth,
   which is measured from North instead of South.
   Returns a list of azimuth, elevation, and radius, and the
   plane distance."
  (let ((plane
	 (sqrt
	  (antik:+ (expt (grid:aref vector 0) 2)
		   (expt (grid:aref vector 1) 2)))))
    (values
      (list
       (atan (grid:aref vector 1) (antik:* sign1 (grid:aref vector 0))) ; azimuth
       (atan (grid:aref vector 2) plane) ; elevation
       ;; replace with euclidean
       (grid:norm vector))
      plane)))

