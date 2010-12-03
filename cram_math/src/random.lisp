
(in-package :cma)

(defun sample (var-fun p-fun &optional (max-retries 10000000))
  "Draw a variable from a probability distripution.

   `var-fun' is a function that maps a value within the interval [0;1)
   to a random variable.
   
   `p-fun' is the probability distripution that maps a variable to a
   probability within the interval [0;1]."
  (when (<= max-retries 0)
    (error 'simple-error
           :format-control "Could not draw a sample. Maximal retries exeeded."))
  (let* ((t-val (random 1.0d0))
         (u-val (random 1.0d0))
         (var (funcall var-fun t-val)))
    (if (<= u-val (funcall p-fun var))
        var
        (sample var-fun p-fun (- max-retries 1)))))

(defun sample-discrete (var-probs)
  "Draws a sample from a discrete probability distribution.

  `var-probs' is a sequence with values of the form (VAR
  . PROBABILITY) with VAR being of arbitrary type and PROBABILITY
  being a probability within the interval [0;1]."
  (let* ((n-vars (length var-probs))
         (vars (make-array n-vars))
         (probs (make-hash-table :test 'equal)))
    (loop for (var . prob) in var-probs
          for i from 0 do
       (setf (aref vars i) var)
       (setf (gethash var probs) prob))
    (flet ((var-fun (x)
             (aref vars (truncate (* x n-vars))))
           (prob-fun (var)
             (gethash var probs)))
      (sample #'var-fun #'prob-fun))))
