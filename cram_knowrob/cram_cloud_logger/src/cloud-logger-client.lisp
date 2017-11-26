(in-package :ccl)


(defparameter *cloud-logger-client* nil)
(defparameter *is-client-connected* nil)
(defparameter *is-logging-enabled* t)

(defparameter *host* "'https://localhost'")
;;(defparameter *host* "'https://192.168.101.42'")
(defparameter *cert-path* "'/home/koralewski/Desktop/localhost.pem'")
;;(defparameter *cert-path* "'/home/ease/asil.pem'")
;;LOCAL PC
;;(defparameter *api-key* "'0nYZRYs5AxDeZAWhWBKYmLF1IJCtRM7gkYTqSV3Noyhl5V3yyxzSaA7Nxi8FFQsC'")
;;AI PC
(defparameter *api-key* "'K103jdr40Rp8UX4egmRf42VbdB1b5PW7qYOOVvTDAoiNG6lcQoaDHONf5KaFcefs'")
;;(defparameter *api-key* "'DiI6fqr5I2ObbeMyI9cDyzjoEHjfz3E48O45M3bKAZh465PUvNtOPB9v8xodMCQT'")


(defclass cloud-logger-client()
  ((address :accessor get-address)
   (certificate :accessor get-certificate)
   (token :accessor get-token)
   (current-query-id :accessor get-current-query-id)
   ))


(defun connect-to-cloud-logger ()
  (if (and *is-client-connected*)
      (print "Already connected to cloud logger")
      (handler-case
          (if *is-logging-enabled*
            (progn
                  (print "Connecting to cloud logger ...")
                 ; (roslisp:start-ros-node "json_prolog_client")
                  (json-prolog:prolog-simple-1 "register_ros_package('knowrob_cloud_logger').")
                  (send-cloud-interface-query *host* *cert-path* *api-key*)
                  (json-prolog:prolog-simple-1 "start_user_container.")
                  (json-prolog:prolog-simple-1 "connect_to_user_container.")
                  (setf *is-client-connected* t)
                  (print "Client is connected to the cloud logger")))
        (ROSLISP::ROS-RPC-ERROR () (print "No JSON Prolog service is running"))
        (SIMPLE-ERROR () (print "Cannot connect to container")))))


(defun init-cloud-logger-client ()
  (setf *cloud-logger-client* (make-instance 'cloud-logger-client)))

(defun send-cloud-interface-query (host cert-path api-key)
  (json-prolog:prolog-simple-1 (create-query "cloud_interface" (list host cert-path api-key))))

(defun send-prolog-query-1 (prolog-query)
  ;;(print prolog-query)
  (if *is-logging-enabled*
   (let ((query-id (get-id-from-query-result
                    (json-prolog:prolog-simple-1
                     (concatenate 'string "send_prolog_query('"
                                  (string prolog-query) "', @(false), Id)")))))
     (let ((query-result (send-next-solution query-id)))
       (send-finish-query query-id)
       query-result))))

(defun send-prolog-query (prolog-query)
  (json-prolog:prolog-simple
   (concatenate 'string "send_prolog_query('" (string prolog-query) "', @(false), Id)")))

(defun get-id-from-query-result (query-result)
  (let ((x (string (cdaar query-result))))
    (subseq x 2 (- (length x) 2))))

(defun send-next-solution(id)
  (json-prolog:prolog-simple-1 (concatenate 'string "send_next_solution('" id "',Result).")))

(defun read-next-prolog-query(query-id)
  (json-prolog:prolog-simple-1 (concatenate 'string "read_next_prolog_query('" query-id "',Result).")))


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

(defun export-log-to-owl (&optional (filename (concatenate 'string "default_log_" (write-to-string (truncate (cram-utilities:current-timestamp))) ".owl")))
  (send-prolog-query-1 (create-query "rdf_save" (list (concatenate 'string "\\'/home/ros/user_data/" filename "\\'" ) "[graph(\\'LoggingGraph\\')]"))))

(defun export-belief-state-to-owl (&optional (filename (concatenate 'string "default_belief_state_" (write-to-string (truncate (cram-utilities:current-timestamp))) ".owl")))
  (json-prolog:prolog-simple-1 (concatenate 'string "rdf_save(" (concatenate 'string "'/home/ease/logs/" filename "'" "," "[graph('belief_state')])"))))

(defun get-value-of-json-prolog-dict (json-prolog-dict key-name)
  (let ((json-prolog-dict-str (string json-prolog-dict)))
    (let ((key-name-search-str (concatenate 'string key-name "\":\"")))
      (let ((key-name-pos (+ (search key-name-search-str (string json-prolog-dict-str)) (length key-name-search-str))))
        (let ((sub-value-str (subseq (string json-prolog-dict-str) key-name-pos)))
          (subseq  sub-value-str 0 (search "\"" sub-value-str)))))))

(defun send-task-success (action-inst is-sucessful)
  (let ((a (convert-to-prolog-str action-inst))
        (b "knowrob:taskSuccess")
        (c (create-owl-literal "xsd:boolean" is-sucessful)))
    (send-rdf-query a b c)))

(defun send-effort-action-parameter (action-inst effort)
  (let ((a (convert-to-prolog-str action-inst))
        (b "knowrob:effort")
        (c (create-owl-literal
            (convert-to-prolog-str "http://qudt.org/vocab/unit#NewtonMeter") effort)))
    (send-rdf-query a b c)))

(defun send-rdf-query (a b c)
  (send-prolog-query-1 (create-rdf-assert-query a b c)))

(defun send-object-action-parameter (action-inst object-designator)
  (let ((object-instance-id (symbol-name (desig:desig-prop-value object-designator :NAME))))
    (send-rdf-query (convert-to-prolog-str action-inst) "knowrob:objectActedOn" (convert-to-prolog-str object-instance-id))
    object-instance-id))

(defun get-object-name (object-name)
  (if (eq (search "|" object-name) 1)
      (subseq object-name 2 (- (length object-name) 2))
      object-name))

(defun send-create-object (action-inst object-name object-type)
  (let ((object-instance-id (send-instance-from-class "object")))
    (send-rdf-query (convert-to-prolog-str object-instance-id) "knowrob:objectName" (convert-to-prolog-str object-name))
    (send-rdf-query (convert-to-prolog-str object-instance-id) "knowrob:objectType" (convert-to-prolog-str object-type))
    (send-rdf-query (convert-to-prolog-str object-instance-id) "knowrob:action" (convert-to-prolog-str action-inst))
    object-instance-id))

(defun create-owl-literal (literal-type literal-value)
  (concatenate 'string "literal(type(" literal-type "," literal-value "))"))

(defun create-rdf-assert-query (a b c)
  (concatenate 'string "rdf_assert(" a "," b "," c ", \\'LoggingGraph\\')."))

(defun convert-to-prolog-str(lisp-str)
  (if (eq 0 (search "'" lisp-str))
      (concatenate 'string "\\'" (subseq lisp-str 1 (- (length lisp-str) 1)) "\\'")
      (concatenate 'string "\\'" lisp-str "\\'")))

(defun send-instance-from-class (instance-class-name)
  (let ((instance-class-id
          (get-value-of-json-prolog-dict (cdaar
                                          (ccl::send-prolog-query-1 (concatenate 'string "rdf_instance_from_class(knowrob:\\'" instance-class-name "\\', \\'LoggingGraph\\', ActionInst)."))) "ActionInst")))
    (send-rdf-query (convert-to-prolog-str instance-class-id) "rdf:type" " owl:\\'NamedIndividual\\'")
    instance-class-id))

(defun create-float-owl-literal (value)
  (create-owl-literal "xsd:float" (format nil "~f" value)))

(defun create-string-owl-literal (value)
  (create-owl-literal "xsd:string" value))

(defun send-create-3d-vector (3d-vector)
  (let ((x (cl-transforms:x 3d-vector))
        (y (cl-transforms:y 3d-vector))
        (z (cl-transforms:z 3d-vector)))
    (concatenate 'string (format nil "~F" x) " " (format nil "~F" y) " " (format nil "~F" z))))


(defun send-create-quaternion (quaternion)
  (let ((x (cl-transforms:x quaternion))
        (y (cl-transforms:y quaternion))
        (z (cl-transforms:z quaternion))
        (w (cl-transforms:w quaternion)))
        (concatenate 'string (format nil "~F" x) " " (format nil "~F" y) " " (format nil "~F" z) " " (format nil "~F" w))))

(defun send-create-pose-stamped (pose-stamped)
  (let ((pose-stamped-instance-id (send-instance-from-class "Pose"))
        ;;(frame-id (cl-transforms-stamped:frame-id pose-stamped))
        ;;(stamp (cl-transforms-stamped:stamp pose-stamped))
        (origin (cl-transforms-stamped:origin pose-stamped))
        (orientation (cl-transforms-stamped:orientation pose-stamped)))
    (let ((3d-vector-id (send-create-3d-vector origin))
          (quaternion-id (send-create-quaternion orientation)))
      ;;(send-rdf-query (convert-to-prolog-str pose-stamped-instance-id) "knowrob:frameId" (create-string-owl-literal frame-id))
      ;;(send-rdf-query (convert-to-prolog-str pose-stamped-instance-id) "knowrob:stamp" (create-float-owl-literal stamp))
      (send-rdf-query (convert-to-prolog-str pose-stamped-instance-id) "knowrob:translation" (create-string-owl-literal (convert-to-prolog-str 3d-vector-id)))
      (send-rdf-query (convert-to-prolog-str pose-stamped-instance-id) "knowrob:quaternion" (create-string-owl-literal (convert-to-prolog-str quaternion-id))))
    pose-stamped-instance-id))

(defun send-pose-stamped-list-action-parameter (action-inst list-name pose-stamped-list)
  (let ((counter 0))
    (dolist (pose-stamp pose-stamped-list)
      (if pose-stamp (progn 
      (send-rdf-query (convert-to-prolog-str action-inst) (concatenate 'string "knowrob:" list-name "_" (write-to-string counter)) (convert-to-prolog-str (send-create-pose-stamped pose-stamp)))
      (setf counter (+ 1 counter)))))))

(defun send-arm-action-parameter (action-inst arm-value)
  (let((arm-value-str (write-to-string arm-value)))
    (cond ((string-equal ":RIGHT" arm-value-str) (send-rdf-query (convert-to-prolog-str action-inst) "knowrob:arm" (convert-to-prolog-str "http://knowrob.org/kb/PR2.owl#pr2_right_arm")))
          ((string-equal ":LEFT" arm-value-str) (send-rdf-query (convert-to-prolog-str action-inst) "knowrob:arm" (convert-to-prolog-str "http://knowrob.org/kb/PR2.owl#pr2_left_arm"))))))

(defun send-gripper-action-parameter (action-inst gripper-value)
  (let((gripper-value-str (write-to-string gripper-value)))
    (cond ((string-equal ":RIGHT" gripper-value-str) (send-rdf-query (convert-to-prolog-str action-inst) "knowrob:bodyPartsUsed" (convert-to-prolog-str "http://knowrob.org/kb/PR2.owl#pr2_right_gripper")))
          ((string-equal ":LEFT" gripper-value-str) (send-rdf-query (convert-to-prolog-str action-inst) "knowrob:bodyPartsUsed" (convert-to-prolog-str "http://knowrob.org/kb/PR2.owl#pr2_left_gripper"))))))

(defun send-location-action-parameter (action-inst location-designator)
  (print action-inst)
  (print location-designator)
  (print "LOCATION"))

(defun send-target-action-parameter (action-inst location-designator)
  (if (desig::desig-prop-value location-designator :POSE)
      (let (
            (pose-id (send-create-pose-stamped
                      (desig::desig-prop-value location-designator :POSE))))
          (send-rdf-query (convert-to-prolog-str action-inst)
                          "knowrob:goalLocation"
                          (convert-to-prolog-str pose-id))))
  
  (let ((location (desig::desig-prop-value location-designator :LOCATION)))
    (if location
        (let ((pose-id (send-create-pose-stamped (desig::desig-prop-value location :POSE))))
          (send-rdf-query (convert-to-prolog-str action-inst)
                          "knowrob:goalLocation"
                          (convert-to-prolog-str pose-id))
          (cond ((desig::desig-prop-value location-designator :REACHABLE-FOR)
                 (send-rdf-query (convert-to-prolog-str pose-id)
                          "knowrob:inReachOf"
                          (convert-to-prolog-str "http://knowrob.org/kb/PR2.owl#PR2Robot1")))
                ((desig::desig-prop-value location-designator :VISIBLE-FOR)
                 (send-rdf-query (convert-to-prolog-str pose-id)
                          "knowrob:inFieldOfView"
                          (convert-to-prolog-str "http://knowrob.org/kb/PR2.owl#PR2Robot1"))))))))

(defun send-finish-query(id)
  (json-prolog:prolog-simple-1 (concatenate 'string "send_finish_query('" id "').")))
