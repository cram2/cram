(in-package :affi)


(defun copy-into-fixnum-vector (seq)
  "Copy a sequence into a vector of fixnums."
  (map '(simple-array fixnum (*)) #'identity seq))

(defun make-fixnum-vector (length)
  "Create a vector with element type fixnum."
  (make-array length :element-type 'fixnum))

;;; LMH From
;;; http://www.ic.unicamp.br/~meidanis/courses/mc336/2006s2/funcional/p21.lisp,
;;; modified to correctly count positions in the list.
(defun insert-at (elem org-list pos)
  "Insert into the list at the indicated position."
  (if (or (zerop pos) (eql org-list nil))
      (cons elem org-list)
      (cons (car org-list) (insert-at elem (cdr org-list) (- pos 1)))))
