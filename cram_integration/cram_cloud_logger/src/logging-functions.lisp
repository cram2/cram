(in-package :ccl)

(defparameter action-designator-parameter-logging-functions (make-hash-table))

(defun define-all-action-designator-parameter-logging-functions ()
  (add-logging-function-for-action-designator-parameter 'send-effort-action-parameter :effort)
  (add-logging-function-for-action-designator-parameter 'send-position-action-parameter :position)
  (add-logging-function-for-action-designator-parameter 'send-object-action-parameter :object)
  (add-logging-function-for-action-designator-parameter 'send-arm-action-parameter :arm)
  (add-logging-function-for-action-designator-parameter 'send-gripper-action-parameter :gripper)
  (add-logging-function-for-action-designator-parameter 'send-left-pose-stamped-list-action-parameter :left-poses)
  (add-logging-function-for-action-designator-parameter 'send-right-pose-stamped-list-action-parameter :right-poses)
  (add-logging-function-for-action-designator-parameter 'send-location-action-parameter :location)
  (add-logging-function-for-action-designator-parameter 'send-target-action-parameter :target))

(defun add-logging-function-for-action-designator-parameter (logging-function action-designator-parameter)
  (setf
   (gethash action-designator-parameter action-designator-parameter-logging-functions)
   logging-function))

(defun get-logging-function-for-action-designator-parameter (action-designator-parameter)
  (gethash action-designator-parameter action-designator-parameter-logging-functions))
