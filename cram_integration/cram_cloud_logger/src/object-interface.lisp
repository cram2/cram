(in-package :ccl)


(defmethod obj-int:get-object-type-gripping-effort :around (object-type)
  (let ((query-id
          (ccl::create-prolog-log-query-str
           "get-object-type-gripping-effort"
           (list (write-to-string object-type))))
        (query-result (call-next-method)))
    (log-end-of-query query-id)
      query-result))

(defmethod obj-int:get-object-type-grasps :around (object-type
                                                   facing-robot-face
                                                   bottom-face
                                                   rotatiationally-symmetric
                                                   arm)
  (let ((query-id
          (ccl::create-prolog-log-query-str
           "get-object-type-grasps"
           (list (write-to-string object-type)
                 (write-to-string facing-robot-face)
                 (write-to-string bottom-face)
                 (write-to-string rotatiationally-symmetric)
                 (write-to-string arm))))
        (query-result (call-next-method)))
    (log-end-of-query query-id)
    (print "QUERY RESULT")
    (print query-result)
    (print "--------------")
      query-result))

(defmethod obj-int:get-object-type-gripper-opening :around (object-type)
  ;;(format t "Asking for GRIPPER OPENING for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    ;;(format t "GRIPPER OPENING Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod obj-int:get-object-type-to-gripper-lift-transform :around (object-type
                                                                      object-name
                                                                      arm
                                                                      grasp
                                                                      grasp-transform)
  ;;(format t "Asking for GRIPPER LIFT TRANSFORMATION for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    ;;(format t "GRIPPER LIFT TRANSFORMATION Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod obj-int:get-object-type-to-gripper-transform :around (object-type
                                                                 object-name
                                                                 arm
                                                                 grasp)
  ;;(format t "Asking for GRIPPER TRANSFORM for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    ;;(format t "GRIPPER TRANSFORM Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod obj-int:get-object-type-to-gripper-pregrasp-transform :around (object-type
                                                                      object-name
                                                                      arm
                                                                      grasp
                                                                      grasp-transform)
  ;;(format t "Asking for GRIPPER PREGRASP TRANSFORMATION for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    ;;(format t "GRIPPER PREGRASP TRANSFORMATION Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod obj-int:get-object-type-to-gripper-2nd-pregrasp-transform :around (object-type
                                                                      object-name
                                                                      arm
                                                                      grasp
                                                                      grasp-transform)
  ;;(format t "Asking for GRIPPER 2ND PREGRASP TRANSFORMATION for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    ;;(format t "GRIPPER 2ND PREGRASP TRANSFORMATION Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod obj-int:get-object-grasping-poses :around (object-name object-type arm grasp object-transform)
  ;;(format t "Asking for GRASPING POSES for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    ;;(format t "Asking for GRASPING POSES Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))
