(in-package :cram-tutorial-planlib)

(define-condition ambiguous-perception (simple-plan-failure)
  ((result :initarg :result :reader result :initform nil))
  (:default-initargs :format-control "amiguous-perception"))

(declare-goal perceive-object (indicator designator)
  (roslisp:ros-info (tutorial plan-lib)
                    "PERCEIVE-OBJECT ~a ~a" indicator designator))

(def-goal (perceive-object all ?obj)
  (with-designators
      ((act-perceive (action `((desig-props:to desig-props:perceive)
                               (desig-props:obj ,?obj)))))
    (perform act-perceive)))

(def-goal (perceive-object :a ?obj)
  (first (perceive-object :all ?obj)))

(def-goal (perceive-object :the ?obj)
  (let ((objects (perceive-object :all ?obj)))
    (cond ((= (length objects) 1)
           (first objects))
          (t (cpl:error 'ambiguous-perception
                        :result objects)))))
