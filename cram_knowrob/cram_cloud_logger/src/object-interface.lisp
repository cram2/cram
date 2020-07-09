(in-package :ccl)


(defmethod man-int:calculate-object-faces :around (robot-to-object-transform)
  (if *is-logging-enabled*
      (let ((pose-id (send-create-transform-pose-stamped robot-to-object-transform)))
        (let ((query-id
                (ccl::create-prolog-log-query-str
                 "calculate-object-faces"
                 (list pose-id)))
              (query-result (call-next-method)))
          (log-end-of-query query-id)
          (log-result-of-query
           query-id
           (concatenate 'string (write-to-string (car query-result)) " " (write-to-string (cadr query-result))))
          query-result))
      (call-next-method)))

(defmethod man-int:get-action-gripping-effort :around (object-type)
  (if *is-logging-enabled*
      (let ((query-id
              (ccl::create-prolog-log-query-str
               "get-object-type-gripping-effort"
               (list (write-to-string object-type))))
            (query-result (call-next-method)))
        (log-end-of-query query-id)
        query-result)
      (call-next-method)))


(defmethod man-int:get-action-grasps :around (object-type
                                              arm
                                              object-transform-in-base)
  (if *is-logging-enabled*
      (let ((query-id
              (ccl::create-prolog-log-query-str
               "get-object-type-grasps"
               (list (write-to-string object-type)
                     (write-to-string nil)
                     (write-to-string nil)
                     (write-to-string nil)
                     (write-to-string arm))))
            (query-result (call-next-method)))
        (log-end-of-query query-id)
        query-result)
      (call-next-method)))

(defmethod man-int:get-action-gripper-opening :around (object-type)
  ;;(format t "Asking for GRIPPER OPENING for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    ;;(format t "GRIPPER OPENING Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod man-int:get-object-type-wrt-base-frame-lift-transforms :around (object-type
                                                                           arm
                                                                           grasp
                                                                           location)
  ;;(format t "Asking for GRIPPER LIFT TRANSFORMATION for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    ;;(format t "GRIPPER LIFT TRANSFORMATION Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod man-int:get-object-type-to-gripper-transform :around (object-type
                                                                 object-name
                                                                 arm
                                                                 grasp)
  ;;(format t "Asking for GRIPPER TRANSFORM for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    ;;(format t "GRIPPER TRANSFORM Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod man-int:get-object-type-to-gripper-pregrasp-transforms :around (object-type
                                                                           object-name
                                                                           arm
                                                                           grasp
                                                                           location
                                                                           grasp-transform)
  ;;(format t "Asking for GRIPPER PREGRASP TRANSFORMATION for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    ;;(format t "GRIPPER PREGRASP TRANSFORMATION Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod man-int:get-object-grasping-poses :around (object-name object-type arm grasp object-transform)
  ;;(format t "Asking for GRASPING POSES for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    ;;(format t "Asking for GRASPING POSES Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))
