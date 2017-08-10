;; Functions with integers
;; Liam Healy 2011-03-30 09:19:10EDT integers.lisp
;; Time-stamp: <2011-03-30 09:23:03EDT integers.lisp>
;; $Id: $

(in-package :antik)

(export '(prime-factors))

;;; From https://groups.google.com/d/msg/comp.lang.lisp/R1cfrqKzecI/zvAT6qDIhswJ
(defun prime-factors (n)
  "Find the prime factors of the integer n."
  (labels ((next-candidate (candidate)
             (if (= 2 candidate)
                 3
                 (+ 2 candidate)))
           (rec (n candidate factors)
             (cond ((or (= n 1) (> candidate n)) factors)
                   ((zerop (mod n candidate))
                    (rec (/ n candidate) candidate (cons candidate factors)))
                   (t (rec n (next-candidate candidate) factors)))))
    (rec n 2 nil)))
