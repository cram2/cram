
(in-package :kipla-reasoning)

(defvar *cop-feedback-pub* nil)

(defun init-cop-feedback ()
  (setf *cop-feedback-pub*
        (roslisp:advertise
         "/cop/feedback"
         "vision_msgs/cop_feedback")))
(register-ros-init-function init-cop-feedback)

;;; publish a cop_feedback message on the corresponding cop topic.
;;; The perception_primitive is received in the cop_feedback message
;;; and should be in the designator passed to the callbacks.
(defun cop-successful-pick-up (op &key ?obj ?side)
  (declare (ignore ?side))
  (when (eq op :assert)
    (let ((perceived-object (reference (kipla::newest-valid-designator ?obj))))
      (when (typep perceived-object 'kipla::cop-perceived-object)
        (roslisp:publish *cop-feedback-pub*
                         (roslisp:make-message
                          "vision_msgs/cop_feedback"
                          perception_primitive (kipla::perception-primitive
                                                perceived-object)
                          evaluation 1.0))))))

(defun cop-failed-pick-up (op &key ?f ?obj ?side)
  (declare (ignore ?side))
  (when (eq op :assert)
    (let ((perceived-object (reference (kipla::newest-valid-designator ?obj))))
      (when (typep perceived-object 'kipla::cop-perceived-object)
        (roslisp:publish *cop-feedback-pub*
                         (roslisp:make-message
                          "vision_msgs/cop_feedback"
                          perception_primitive (kipla::perception-primitive
                                                perceived-object)
                          evaluation 0.0
                          error (vector
                                 (roslisp:make-message
                                  "vision_msgs/system_error"
                                  error_id 0
                                  node_name roslisp:*ros-node-name*
                                  error_description (symbol-name ?f)))))))))

(register-production-handler 'object-picked-up #'cop-successful-pick-up)
(register-production-handler 'object-in-hand-failure #'cop-failed-pick-up)
