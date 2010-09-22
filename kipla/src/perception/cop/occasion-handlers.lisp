
(in-package :kipla-reasoning)

(defun cop-successful-pick-up (op &key ?obj ?side)
  (declare (ignore op ?obj ?side))
  nil)

(defun cop-failed-pick-up (op &key ?f ?obj ?side)
  (declare (ignore op ?f ?obj ?side))
  nil)

(register-production-handler 'object-picked-up #'cop-successful-pick-up)
(register-production-handler 'object-in-hand-failure #'cop-failed-pick-up)
