;; Tests of physical quantities
;; Liam Healy 2011-01-09 17:38:41EST tests.lisp
;; Time-stamp: <2015-01-01 12:17:55EST physical-quantities-grid.lisp>

;; Copyright 2011, 2013, 2014, 2015 Liam M. Healy
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
(named-readtables:in-readtable :antik)

;;; These are all scalar units, i.e., the same units apply to each
;;; element of the array.
(lisp-unit:define-test grid-scalar-units
  (lisp-unit:assert-true
   ;; An array with pqs as elements
   (let ((grid (vector (make-pq 1.0 'm) (make-pq 5.0 'm))))
     (and (arrayp grid)
	  (equal (grid:dimensions grid) '(2))
	  (check-pq (grid:aref grid 0) 1.0d0 'meter)
	  (check-pq (grid:aref grid 1) 5.0d0 'meter))))
  (lisp-unit:assert-true
   ;; An array with pqs as elements
   (let ((grid (make-array '(2) :initial-contents (list (make-pq 1.0 'm) (make-pq 5.0 'm)))))
     (and (arrayp grid)
	  (equal (grid:dimensions grid) '(2))
	  (check-pq (grid:aref grid 0) 1.0d0 'meter)
	  (check-pq (grid:aref grid 1) 5.0d0 'meter))))
  (lisp-unit:assert-true
   ;; A physical-quantity with a foreign-array as magnitude
   (check-pq #m(#_1.0d0_m #_5.0d0_m) #m(1.0d0 5.0d0) 'meter))
  (lisp-unit:assert-true
   ;; A physical-quantity with an array as magnitude
   (check-pq
    (grid:make-simple-grid
     :grid-type 'array
     :initial-contents (list #_1.0d0_m #_5.0d0_m))
    #(1.0d0 5.0d0) 'meter))
  (lisp-unit:assert-true
   ;; A physical-quantity with a foreign-array as magnitude
   (check-pq
    (grid:make-foreign-array
     'double-float :dimensions '(2) :initial-contents '(#_1.0d0_m #_5.0d0_m))
    (grid:make-foreign-array
     'double-float :dimensions '(2) :initial-contents '(1.0d0 5.0d0))
    'meter))
  (lisp-unit:assert-true
   ;; A physical-quantity with an array as magnitude
   (check-pq #_#(1.0d0 2.0d0)_m #(1.0d0 2.0d0) 'meter))
  (lisp-unit:assert-true
   ;; A physical-quantity with an array as magnitude
   (check-pq (make-pq #(1.0d0 2.0d0) 'meter) #(1.0d0 2.0d0) 'meter))
  (lisp-unit:assert-true
   ;; A physical-quantity with a foreign-array as magnitude
   (check-pq #_#m(1.0d0 5.0d0)_m #m(1.0d0 5.0d0) 'meter))
  (lisp-unit:assert-true
   ;; A physical-quantity with a foreign-array as magnitude
   (check-pq (make-pq #m(1.0d0 5.0d0) 'meter) #m(1.0d0 5.0d0) 'meter))
  (lisp-unit:assert-true
   ;; A physical-quantity using make-grid with :initial-element
   (check-pq
    (grid:make-grid '((grid:foreign-array 3) double-float)
		    :initial-element #_23.0_m)
    (grid:make-foreign-array 'double-float :initial-contents '(23.0d0 23.0d0 23.0d0))
    'meter))
  (lisp-unit:assert-true
   ;; A physical-quantity using make-grid with :initial-contents
   (let ((pq
	   (grid:make-grid '((grid:foreign-array 3) double-float)
			   :initial-contents '(#_1_km #_2_km #_3_km))))
     (and (scalar-dimension pq)
	  (check-pq pq
		    (grid:make-foreign-array 'double-float :initial-contents '(1.0d3 2.0d3 3.0d3))
		    'meter)))))

(lisp-unit:define-test grid-array-units
  (lisp-unit:assert-numerical-equal
   ;; see [[id:25d7e1e8-5adc-4433-b933-fb95069bc096][Search for default in sources instead of taking the first one]]
   '(#_6.000000000000000d0_m #_5.000000000000000d0_m #_4.000000000000000d0_m) 
   (grid:contents
    (grid:subgrid
     (grid:make-foreign-array
      'double-float
      :initial-contents '(#_6.0_m #_5.0_m #_4_m #_3.0_m/s #_2.0_m/s #_1.0_m/s))
     '(3) '(0))))
  (lisp-unit:assert-true
   ;; An array with pqs as elements
   (let ((grid #(#_1.0d0_m #_5.0d0_s)))
     (and (arrayp grid)
	  (equal (grid:dimensions grid) '(2))
	  (check-pq (grid:aref grid 0) 1.0d0 'meter)
	  (check-pq (grid:aref grid 1) 5.0d0 'second))))
  (lisp-unit:assert-true
   ;; An array with pqs as elements
   (let ((grid (make-array '(2) :initial-contents '(#_1.0d0_m #_5.0d0_s))))
     (and (arrayp grid)
	  (equal (grid:dimensions grid) '(2))
	  (check-pq (grid:aref grid 0) 1.0d0 'meter)
	  (check-pq (grid:aref grid 1) 5.0d0 'second))))
  ;; [[id:6e7e8d6f-cc79-44bb-a264-06e9ec160d49][This test prints out when quickloaded]]
  #+(or)
  (lisp-unit:assert-true
   ;; A physical-quantity with a foreign-array as magnitude
   (check-pq #m(#_1.0d0_m #_5.0d0_s) #m(1.0d0 5.0d0) #(meter second)))
  ;; This should work but it doesn't like #(meter) for units:
;;;(lisp-unit:assert-true
  ;; A physical-quantity with a foreign-array as magnitude, single
;;;   (check-pq #m(#_1.0d0_m) #m(1.0d0) #(meter)))
  (lisp-unit:assert-true
   ;; A vector with foreign-array magnitude and :initial-contents
   (check-pq
    (grid:make-grid '((grid:foreign-array 3) double-float)
		    :initial-contents '(#_1_km #_2_km #_3_sec))
    (grid:make-foreign-array
     'double-float :dimensions '(2) :initial-contents '(1.0d3 2.0d3 3.0d0))
    #(meter meter second)))
  (lisp-unit:assert-true
   ;; A 2D foreign-array magnitude and :initial-contents
   (check-pq
    (grid:make-grid '((grid:foreign-array 3 2) double-float)
		    :initial-contents '((#_1_km #_2_km #_3_sec) (#_1_m #_2_m #_88_sec)))
    (grid:make-foreign-array
     'double-float :dimensions '(2 3)
     :initial-contents '((1000.0d0 2000.0d0 3.0d0) (1.0d0 2.0d0 88.0d0)))
    #2A((meter meter second) (meter meter second))))
  (lisp-unit:assert-true
   ;; Concatenate grids, array
   (check-pq (grid:concatenate-grids #_#(1.0d0 2.0d0)_m #_#(3.0d0 4.0d0)_s)
	     #(1.0d0 2.0d0 3.0d0 4.0d0)
	     #(meter meter second second)))
  (lisp-unit:assert-true
   ;; Concatenate grids, foreign-array
   ;;  (grid:concatenate-grids #_#m(1.0d0 2.0d0)_m #_#m(3.0d0 4.0d0)_s)
   (check-pq (let ((grid:*default-grid-type* 'grid:foreign-array))
	       (grid:concatenate-grids
		(make-pq
		 (grid:make-grid '((grid:foreign-array 2) double-float) :initial-contents
				 '(1.0 2.0))
		 'meter t)
		(make-pq
		 (grid:make-grid '((grid:foreign-array 2) double-float) :initial-contents
				 '(3.0 4.0))
		 'second t)))
	     (grid:make-foreign-array 'double-float
				      :dimensions '(4)
				      :initial-contents '(1.0d0 2.0d0 3.0d0 4.0d0))
	     #(meter meter second second)))
  
  )

#|
;;; Things that should be made to work.
;;; Multiplication of dimensionless grid by dimensioned grid
(* #((1.0 0.0) (0.0 1.0)) #(#_1_km #_2_s))  ; works
(* #m((1.0 0.0) (0.0 1.0)) #m(#_1_km #_2_s))  ; does not work

;;; Add dimensioned
(+ #m(#_1_km #_2_s) #m(#_1_km #_2_s))

;;; Multiply dimensioned
(* #_#m((1.0 0.0) (0.0 1.0))_km #m(#_1_km #_2_s))
|#
