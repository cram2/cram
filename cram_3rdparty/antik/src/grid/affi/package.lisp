(defpackage :affi
  (:use :common-lisp)
  (:export

   ;; utility.lisp -- nothing is exported

   ;; affi.lisp

   affi get-const get-coeff get-domain rank size range make-affi
   make-affi-cm calculate-index make-walker in-affi test-walker
   check-conformability))
