
(in-package :kipla-reasoning)

(defun make-object-cost-function (obj-loc-desig std-dev)
  (declare (type object-designator obj-loc-desig)
           (type number std-dev))
  (let* ((obj-loc-jlo (reference obj-loc-desig))
         (gauss-fun (cma:gauss
                     (grid:make-foreign-array 'float
                                              :dimensions '(2 2)
                                              :initial-contents `((,(coerce (* std-dev std-dev)
                                                                            'float) 0)
                                                                  (0 ,(coerce (* std-dev std-dev)
                                                                              'float))))
                     (grid:make-foreign-array 'float
                                              :dimensions '(2 1)
                                              :initial-contents (list (jlo:pose obj-loc-jlo 0 3)
                                                                      (jlo:pose obj-loc-jlo 1 3))))))
    (lambda (x y)
      (funcall gauss-fun (grid:make-foreign-array 'float
                                                  :dimensions '(2 1)
                                                  :initial-contents (list (coerce x 'float)
                                                                          (coerce y 'float)))))))
