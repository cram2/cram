
(in-package :prolog)

(defun prolog-type-p (variable type)
  (alexandria:ignore-some-conditions (simple-error)
    (typep variable type)))

(defun query-var (var function query &rest args)
  "Executes `function' with `query' as its first parameter and `args'
  as additional parameters and returns the value of `var' in the first
  result."
  (let ((bdg (car (apply function (cons query args)))))
    (when bdg
      (var-value var bdg))))
