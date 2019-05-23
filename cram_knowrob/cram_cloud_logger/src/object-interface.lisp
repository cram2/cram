(in-package :ccl)

(defmethod cram-manipulation-interfaces:get-action-gripping-effort :around (object-type)
    (if *is-logging-enabled*
        (let ((query-result (call-next-method)))
          (log-reasoning-task "cram-manipulation-interfaces:get-action-gripping-effort" object-type (write-to-string query-result))
          query-result)
        (call-next-method)))

(defmethod cram-manipulation-interfaces:get-action-gripper-opening :around (object-type)
    (if *is-logging-enabled*
        (let ((query-result (call-next-method)))
          (log-reasoning-task "cram-manipulation-interfaces:get-action-gripper-opening" object-type (write-to-string query-result))
          query-result)
        (call-next-method)))

(defmethod cram-manipulation-interfaces:get-action-grasps :around  (object-type arm object-transform-in-base)
    (if *is-logging-enabled*
        (let* ((query-result (call-next-method))
               (query-id (log-reasoning-task "cram-manipulation-interfaces:get-action-grasps" object-type (write-to-string query-result)))
               (pose-id (send-create-transform-pose-stamped object-transform-in-base)))
          (send-rdf-query (convert-to-prolog-str query-id)
                          "knowrob:parameter2"
                          (convert-to-prolog-str pose-id))
          query-result)
        (call-next-method)))

(defmethod cram-manipulation-interfaces:get-action-trajectory :around  (action-type arm grasp objects-acted-on  &key &allow-other-keys)
    (if *is-logging-enabled*
        (let ((query-result (call-next-method)))
          (log-reasoning-task "cram-manipulation-interfaces:get-action-trajectory" grasp "result")
          query-result)
        (call-next-method)))

(defmethod cram-manipulation-interfaces:get-location-poses :around (location-designator)
    (if *is-logging-enabled*
        (let ((query-result (call-next-method)))
          ;;(log-reasoning-task "cram-manipulation-interfaces:get-location-pose" location-designator query-result)
          query-result)
      (call-next-method)))

(defun log-reasoning-task (predicate-name parameter reasoning-result)
  (let
      ((query-id (create-reasoning-task-query-id predicate-name)))
    (send-init-reasoning-query query-id)
    (send-predicate-query query-id predicate-name)
    (send-parameter-query query-id parameter)
    (send-link-reasoing-to-action query-id)
    (send-result-query query-id reasoning-result)
    query-id))


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
  (send-rdf-query (convert-to-prolog-str query-id)
                  "knowrob:predicate"
                  (convert-to-prolog-str predicate-name)))

(defun send-result-query (query-id result-query)
  (send-rdf-query (convert-to-prolog-str query-id)
                    "knowrob:result"
                    (convert-to-prolog-str result-query)))

(defun send-parameter-query (query-id parameter)
  (send-rdf-query (convert-to-prolog-str query-id)
                  "knowrob:parameter"
                  (convert-to-prolog-str parameter)))

(defun send-link-reasoing-to-action (query-id)
  (send-rdf-query (convert-to-prolog-str (car *action-parents*))
                  "knowrob:reasoningTask"
                  (convert-to-prolog-str query-id)))

(defun create-reasoning-task-query-id (predicate-name)
  (concatenate 'string predicate-name (format nil "~x" (random (expt 16 8)))))
