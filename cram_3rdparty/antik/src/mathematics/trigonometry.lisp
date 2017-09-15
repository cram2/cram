;; Trigonometric functions
;; Liam Healy Wed Dec  7 2005 - 22:53
;; Time-stamp: <2011-01-18 15:46:56EST trigonometry.lisp>

(in-package :antik)

(export '(angle-law-of-cosines))

(defun angle-law-of-cosines
    (length-opp length-adj1 length-adj2)
  "Solve for the angle when lengths of a triangle are known."
  (acos (/ (+ (expt length-adj1 2) (expt length-adj2 2)
		(- (expt length-opp 2)))
	    (* 2 length-adj1 length-adj2))))
