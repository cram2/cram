(in-package :ccl)

(defparameter cram-action-designator-parameter-to-knowrob (make-hash-table))

(defun init-map ()
  (add-to-map :effort 'send-effort-action-parameter)
  (add-to-map :position 'send-position-action-parameter)
  (add-to-map :object 'send-object-action-parameter)
  (add-to-map :arm 'send-arm-action-parameter)
  (add-to-map :gripper 'send-gripper-action-parameter)
  (add-to-map :left-poses 'send-left-pose-stamped-list-action-parameter)
  (add-to-map :right-poses 'send-right-pose-stamped-list-action-parameter)
  (add-to-map :location 'send-location-action-parameter)
  (add-to-map :target 'send-target-action-parameter))

(defun add-to-map (cram-action-designator-property to-knowrob-function)
  (setf
   (gethash  cram-action-designator-property cram-action-designator-parameter-to-knowrob)
   to-knowrob-function))

(init-map)
(defun log-action-parameter (designator action-id)
  (mapcar (lambda (key-value-pair)
            (let ((key (first key-value-pair))
                  (value (second key-value-pair)))
              (execute-logging-parameter-function key action-id value)))
          (desig:properties designator)))

(defun execute-logging-parameter-function (cram-action-designator-property action-id value)
  (let ((logging-parameter-function
          (get-logging-parameter-function cram-action-designator-property))
        (logging-parameter-function-arguments
          (list action-id value)))
    (print logging-parameter-function)
    (when logging-parameter-function 
      (apply logging-parameter-function logging-parameter-function-arguments))))

(defun get-logging-parameter-function (cram-action-designator-property)
  (gethash cram-action-designator-property cram-action-designator-parameter-to-knowrob))
