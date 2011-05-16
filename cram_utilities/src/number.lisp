(in-package :cut)

; bool->float
(declaim (inline bool->float))
(defun bool->float (val)
  "returns 1.0 for true and 0.0 for false"
  (if val 1.0 0.0))

; random-number
(let ( (random-state (make-random-state T)) )
  (defun random-number (limit &key (lower 0))
    "returns a random number in range [lower, limit["
    (if (zerop limit)
      limit
      (+ (random (- limit lower) random-state) lower)))
)
