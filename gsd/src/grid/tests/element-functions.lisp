;; Test functions of elements
;; Liam Healy 2010-06-20 10:54:42EDT element-functions.lisp
;; Time-stamp: <2010-07-03 22:49:28EDT element-functions.lisp>

(in-package :grid)

(lisp-unit:define-test element-functions
  (lisp-unit:assert-numerical-equal
   '((0.0d0 1.0d0 1.4142135623730951d0 1.7320508075688772d0)
     (3.1622776601683795d0 3.3166247903554d0 3.4641016151377544d0 3.605551275463989d0)
     (4.47213595499958d0 4.58257569495584d0 4.69041575982343d0 4.795831523312719d0))
   ;; Elementwise application of a function
   (contents
    (map-grid :source (test-grid-double-float 'array '(3 4))
	      :element-function 'sqrt)))
  (lisp-unit:assert-numerical-equal
   '(0.0d0 1.0d0 1.4142135623730951d0 1.7320508075688772d0
     3.1622776601683795d0 3.3166247903554d0 3.4641016151377544d0
     3.605551275463989d0 4.47213595499958d0 4.58257569495584d0
     4.69041575982343d0 4.795831523312719d0)  
   ;; Elementwise application of a function, reshaping into a 1d array
   (contents
    (map-grid :source (test-grid-double-float 'array '(3 4))
	      :element-function 'sqrt
	      :destination-specification '((array 12) double-float)))))
