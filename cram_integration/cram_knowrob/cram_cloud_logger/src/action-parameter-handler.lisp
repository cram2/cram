(in-package :ccl)


(define-all-action-designator-parameter-logging-functions)

(defun log-action-designator-parameters-for-logged-action-designator
    (action-designator-parameters action-designator-logging-id)
  (mapcar (lambda (action-designator-parameter)
            (let ((parameter-name (first action-designator-parameter))
                  (parameter-value (second action-designator-parameter)))
              (let ((logging-function (get-logging-function-for-action-designator-parameter parameter-name))
                    (logging-args (list action-designator-logging-id parameter-value)))
                (execute-logging-function-with-arguments
                  logging-function logging-args))))
          action-designator-parameters))

(defun execute-logging-function-with-arguments
    (logging-parameter-function logging-parameter-function-arguments)
    (when logging-parameter-function 
      (apply logging-parameter-function logging-parameter-function-arguments)))
