(in-package :ccl)


(defparameter *cloud-logger-client* nil)
(defparameter *is-client-connected* nil)

(defparameter *host* "'https://localhost'")
(defparameter *cert-path* "'/home/koralewski/Desktop/localhost.pem'")
;(defparameter *api-key* "'0nYZRYs5AxDeZAWhWBKYmLF1IJCtRM7gkYTqSV3Noyhl5V3yyxzSaA7Nxi8FFQsC'")
(defparameter *api-key* "'K103jdr40Rp8UX4egmRf42VbdB1b5PW7qYOOVvTDAoiNG6lcQoaDHONf5KaFcefs'")


(defclass cloud-logger-client()
  ((address :accessor get-address)
   (certificate :accessor get-certificate)
   (token :accessor get-token)
   (current-query-id :accessor get-current-query-id)
   ))

;(defgeneric connect (client))
(defgeneric get-next-solution (client))
(defgeneric perform-query (client prolog-query))

(defun connect-to-cloud-logger ()
  (if *is-client-connected*
      (print "Already connected to cloud logger")
      (handler-case (progn
                  (roslisp:start-ros-node "json_prolog_client")
                  (json-prolog:prolog-simple-1 "register_ros_package('knowrob_cloud_logger').")
                  (send-cloud-interface-query *host* *cert-path* *api-key*)
                  (json-prolog:prolog-simple-1 "start_user_container.")
                  (json-prolog:prolog-simple-1 "connect_to_user_container.")
                  (setf *is-client-connected* t)
                  (print "Client is connected to the cloud logger"))
        (ROSLISP::ROS-RPC-ERROR () (print "No JSON Prolog service is running"))
        (SIMPLE-ERROR () (print "Cannot connect to container")))))

(defmethod perform-query ((client cloud-logger-client) prolog-query)
  (let ((query-id (get-id-from-query-result (send-prolog-query-1 prolog-query))))
    (setf (slot-value client 'current-query-id) query-id)
    (send-next-solution query-id)
    (read-next-prolog-query)))

(defmethod get-next-solution ((client cloud-logger-client))
  (send-next-solution (get-current-query-id client))
  (read-next-prolog-query))


(defun init-cloud-logger-client ()
  (setf *cloud-logger-client* (make-instance 'cloud-logger-client)))

(defun send-cloud-interface-query (host cert-path api-key)
  (json-prolog:prolog-simple-1 (create-query "cloud_interface" (list host cert-path api-key))))

(defun send-prolog-query-1 (prolog-query)
  (send-next-solution
   (get-id-from-query-result
    (json-prolog:prolog-simple-1
     (concatenate 'string "send_prolog_query('" (string prolog-query) "', @(false), Id)"))))
  (read-next-prolog-query))

(defun send-prolog-query (prolog-query)
  (json-prolog:prolog-simple
   (concatenate 'string "send_prolog_query('" (string prolog-query) "', @(false), Id)")))

;(defun test-interface ()
;  (let (test-cloud-logger-client)
;    (setf test-cloud-logger-client (make-instance 'cloud-logger-client))
;    (connect test-cloud-logger-client)
;    (send-cram-start-action "knowrob:\\'CRAMAction\\'" " \\'DummyContext\\'" "1492785072" "PV" "ActionInst")
;    (export-log-to-owl "lisp-interface.owl")))

(defun get-id-from-query-result (query-result)
  (let ((x (string (cdaar query-result))))
    (subseq x 2 (- (length x) 2))))

(defun send-next-solution(id)
  (json-prolog:prolog-simple-1 (concatenate 'string "send_next_solution('" id "').")))

(defun read-next-prolog-query()
  (json-prolog:prolog-simple-1 "read_next_prolog_query(Result)."))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;Logging Specific Part;;;;;;;;;;;;;;;;;

(defun create-parameters (parameter-list)
  (if (listp parameter-list)
      (let ((result (car parameter-list)))
        (dolist (item (cdr parameter-list))
          (setq result (concatenate 'string result "," item)))
        result)))

(defun create-query (query-name parameter-list)
  (concatenate 'string query-name "(" (create-parameters parameter-list) ")."))

(defun send-cram-start-parent-action (owl-action-class task-context start-time
                               prev-action parent-task action-inst)
  (send-prolog-query-1 (create-query "cram_start_action" (list owl-action-class task-context start-time
                               prev-action parent-task action-inst))))

(defun send-cram-start-action (owl-action-class task-context start-time
                               prev-action action-inst)
  (send-prolog-query-1 (create-query "cram_start_action" (list owl-action-class task-context start-time
                               prev-action action-inst))))

(defun send-cram-finish-action (action-inst end-time)
  (send-prolog-query-1 (create-query "cram_finish_action" (list action-inst end-time))))

(defun send-cram-set-subaction (sup sub)
  (send-prolog-query-1 (create-query "cram_set_subaction" (list sup sub))))

(defun send-cram-add-image-to-event (event image-url)
  (send-prolog-query-1 (create-query "cram_add_image_to_event" (list event image-url))))

(defun send-cram-add-failure-to-action (action-inst failure-type failure-label failure-time failure-inst)
  (send-prolog-query-1 (create-query "cram_add_failure_to_action" (list action-inst failure-type failure-label failure-time failure-inst))))

(defun send-cram-create-designators (desig-type desig-inst)
  (send-prolog-query-1 (create-query "cram_create_designators" (list desig-type desig-inst))))

(defun send-cram-equate-designators (pre-designator succ-designator equation-time)
  (send-prolog-query-1 (create-query "cram_equate_designators" (list pre-designator succ-designator equation-time))))

(defun send-cram-add-designator-to-action-with-prop-id (action-inst property-identifier designator-inst)
  (send-prolog-query-1 (create-query "cram_add_desig_to_action" (list action-inst property-identifier designator-inst))))

(defun send-cram-add-designator-to-action (action-inst designator-inst)
  (send-prolog-query-1 (create-query "cram_add_desig_to_action" (list action-inst designator-inst))))

(defun send-cram-set-object-acted-on (action-inst object-inst)
  (send-prolog-query-1 (create-query "cram_set_object_acted_on" (list action-inst object-inst))))

(defun send-cram-set-detected-object (action-inst object-type object-inst)
  (send-prolog-query-1 (create-query "cram_set_detected_object" (list action-inst object-type object-inst))))

(defun send-cram-set-perception-request (action-inst req)
  (send-prolog-query-1 (create-query "cram_set_perception_request" (list action-inst req))))

(defun send-cram-set-perception-result (action-inst res)
  (send-prolog-query-1 (create-query "cram_set_perception_result" (list action-inst res))))

(defun export-log-to-owl (filename)
  (send-prolog-query-1 (create-query "rdf_save" (list (concatenate 'string "\\'/home/ros/user_data/" filename "\\'" ) "[graph(\\'LoggingGraph\\')]"))))

(defun get-value-of-json-prolog-dict (json-prolog-dict key-name)
  (let ((json-prolog-dict-str (string json-prolog-dict)))
    (let ((key-name-search-str (concatenate 'string key-name "\":\"")))
      (let ((key-name-pos (+ (search key-name-search-str (string json-prolog-dict-str)) (length key-name-search-str))))
        (let ((sub-value-str (subseq (string json-prolog-dict-str) key-name-pos)))
          (subseq  sub-value-str 0 (search "\"" sub-value-str)))))))






