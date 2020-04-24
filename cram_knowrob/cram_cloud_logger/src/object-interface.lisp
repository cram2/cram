(in-package :ccl)

(defmethod cram-manipulation-interfaces:get-action-gripping-effort :around (object-type)
    (if *is-logging-enabled*
      (let ((reasoning-id (log-reasoning-task  "GetActionGrippingEffort")))
        (ccl::start-situation reasoning-id)
        (let ((query-result (call-next-method)))
          (ccl::stop-situation reasoning-id)
          query-result))        
      (call-next-method)))

(defmethod cram-manipulation-interfaces:get-action-gripper-opening :around (object-type)
  (if *is-logging-enabled*
      (let ((reasoning-id (log-reasoning-task  "GetActionGripperOpening")))
        (ccl::start-situation reasoning-id)
        (let ((query-result (call-next-method)))
          (ccl::stop-situation reasoning-id)
          query-result))        
      (call-next-method)))


(defmethod cram-manipulation-interfaces:get-action-grasps :around  (object-type arm object-transform-in-base)
  (if *is-logging-enabled*
      (let ((reasoning-id (log-reasoning-task  "GetActionGrasps")))
        (ccl::start-situation reasoning-id)
        (let ((query-result (call-next-method)))
          (ccl::stop-situation reasoning-id)
          query-result))        
      (call-next-method)))

(defmethod cram-manipulation-interfaces:get-container-closing-distance :around (container-name)
    (if *is-logging-enabled*
      (let ((reasoning-id (log-reasoning-task  "GetContainerClosingDistance")))
        (ccl::start-situation reasoning-id)
        (let ((query-result (call-next-method)))
          (ccl::stop-situation reasoning-id)
          query-result))        
      (call-next-method)))

(defmethod cram-manipulation-interfaces:get-container-opening-distance :around (container-name)
  (if *is-logging-enabled*
      (let ((reasoning-id (log-reasoning-task  "GetContainerOpeningDistance")))
        (ccl::start-situation reasoning-id)
        (let ((query-result (call-next-method)))
          (ccl::stop-situation reasoning-id)
          query-result))        
      (call-next-method)))

;;(defmethod cram-manipulation-interfaces:get-action-grasps :around  (object-type arm object-transform-in-base)
;;    (if *is-logging-enabled*
;;        (let* ((query-result (call-next-method))
;;               (query-id (log-reasoning-task "cram-manipulation-interfaces:get-action-grasps" (write-to-string object-type) (write-to-string query-result)))
;;               (pose-id (send-create-transform-pose-stamped object-transform-in-base)))
;;          (send-rdf-query query-id
;;                          "knowrob:parameter2"
;;                          (convert-to-prolog-str pose-id))
;;          query-result)
;;        (call-next-method)))

(defmethod cram-manipulation-interfaces:get-action-trajectory :around  (action-type arm grasp objects-acted-on  &key &allow-other-keys)
    (if *is-logging-enabled*
      (let ((reasoning-id (log-reasoning-task  "GetActionTrajectory")))
        (ccl::start-situation reasoning-id)
        (let ((query-result (call-next-method)))
          (ccl::stop-situation reasoning-id)
          query-result))        
      (call-next-method)))

(defmethod cram-manipulation-interfaces:get-location-poses :around (location-designator)
  (if *is-logging-enabled*
      (let ((reasoning-id (log-reasoning-task "GetLocationPoses")))
        (ccl::start-situation reasoning-id)
        (let ((query-result (call-next-method)))
          (ccl::stop-situation reasoning-id)
          query-result))        
      (call-next-method)))

(defun log-reasoning-task (predicate-name)
  (let ((reasoning-url (create-reasoning-url predicate-name)))
      (attach-event-to-situation reasoning-url (get-parent-uri))))


(defun create-reasoning-url (predicate-name)
  (concatenate 'string "'http://www.ease-crc.org/ont/EASE-ACT.owl#" predicate-name "'"))

;;(defun log-reasoning-task (predicate-name parameter reasoning-result)
;;  (let
;;      ((query-id (convert-to-prolog-str (get-value-of-json-prolog-dict
;;                      (cdaar
;;                       (send-cram-start-action
;;                        (concatenate 'string "knowrob:" (convert-to-prolog-str "ReasoningTask"))
;;                        " \\'TableSetting\\'"
;;                        (convert-to-prolog-str (get-timestamp-for-logging))
;;                        "PV"
;;                        "ActionInst"))
;;                      "ActionInst"))))
;;    (print "REASONING LOGGING")
;;    ;;(send-init-reasoning-query query-id)
;;    (send-predicate-query query-id predicate-name)
;;        (print "REASONING LOGGING 1")
;;    (send-parameter-query query-id parameter)
;;        (print "REASONING LOGGING 2")
;;    (send-link-reasoing-to-action query-id)
;;        (print "REASONING LOGGING 3")
;;    (send-result-query query-id reasoning-result)
;;       (print "REASONING LOGGING 4")
;;    query-id))


(defun send-init-reasoning-query (query-id)
  (send-prolog-query-1
   (create-query
    "cram_start_action"
    (list (concatenate 'string "knowrob:" (convert-to-prolog-str "PrologQuery"))
          "\\'TableSetting\\'"
          (get-timestamp-for-logging)
          "PV"
          query-id))))

(defun send-predicate-query (query-id predicate-name)
  (send-rdf-query query-id
                  "knowrob:predicate"
                  (convert-to-prolog-str predicate-name)))

(defun send-result-query (query-id result-query)
  (send-rdf-query query-id
                    "knowrob:result"
                    (convert-to-prolog-str result-query)))

(defun send-parameter-query (query-id parameter)
  (send-rdf-query query-id
                  "knowrob:parameter"
                  (convert-to-prolog-str parameter)))

(defun send-link-reasoing-to-action (query-id)
  (send-rdf-query (convert-to-prolog-str (car *action-parents*))
                  "knowrob:reasoningTask"
                  query-id))

(defun create-reasoning-task-query-id ()
  (convert-to-prolog-str (concatenate 'string "http://knowrob.org/kb/knowrob.owl#" "PrologQuery_" (format nil "~x" (random (expt 16 8))))))
  ;;(concatenate (concatenate 'string "knowrob:" (convert-to-prolog-str (concatenate 'string "PrologQuery_" (format nil "~x" (random (expt 16 8))))))))
