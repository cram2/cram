;; Write output data
;; Liam Healy 2011-04-21 15:10:58EDT write.lisp
;; Time-stamp: <2011-04-21 17:23:10EDT write.lisp>

(in-package :antik)

(export 'write-data)

;;; These definitions are a combination of #'nf and output-destination.

;;; (set-nf-options :significant-figures 5)
;;;    (output (:file "freshcart.dat")            )

;;; It would be nice to have a defined-width option for nf so that we
;;; could then tab to align the columns on the right, rather than the
;;; left.
(defun write-data (list &optional (stream *standard-output*))
  "Write the formatted data, given as a list of lists."
  (with-nf-options (:style nil)
    (dolist (l list) (format stream "~{~0,12t~a~}~&" (mapcar 'nf-string l)))))
