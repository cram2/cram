
(in-package :cut)

(define-method-combination hooks (&key (hook-combination #'list))
  ((methods *))
  "Every method should be qualified by a symbol. All matching methods
   are executed and the results are combined using `hook-combination'
   which returns the final result of the method. The
   `hook-combination' function gets every method result as a
   parameter."
  `(funcall ,hook-combination
            ,@(loop for m in methods
                    collecting `(call-method ,m))))

(defmacro define-hook (fun-name lambda-list &body options)
  "Wrapper around DEFGENERIC. If `options' does not include a
   :METHOD-COMBINATION clause, the HOOKS method combination is used. Also if
   the defined function is called an no methods are applicable, NIL is
   returned instead of an error condition being signaled."
  (let ((method-combination-clause (or (assoc :method-combination options)
                                       '(:method-combination hooks)))
        (options (remove :method-combination options :key #'car)))
    `(progn (defgeneric ,fun-name ,lambda-list
              ,method-combination-clause
              ,@options)
            (defmethod no-applicable-method ((generic-function (EQL #',fun-name)) &rest args)
              (declare (ignore args generic-function))
              nil))))
