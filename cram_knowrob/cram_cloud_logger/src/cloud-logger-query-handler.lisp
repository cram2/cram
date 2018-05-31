(in-package :ccl)

(defun send-cram-start-parent-action (owl-action-class task-context start-time
                               prev-action parent-task action-inst)
  (send-prolog-query-1
   (create-query
    "cram_start_action"
    (list owl-action-class task-context start-time
          prev-action parent-task action-inst))))

(defun send-cram-start-action (owl-action-class task-context start-time
                               prev-action action-inst)
  (send-prolog-query-1
   (create-query
    "cram_start_action"
    (list owl-action-class task-context start-time
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

(defun reset-logged-owl ()
  (send-prolog-query-1 (create-query "rdf_retractall" (list "A" "B" "C" "\\'LoggingGraph\\'"))))

(defun export-belief-state-to-owl (&optional (filename (concatenate 'string "default_belief_state_" (write-to-string (truncate (cram-utilities:current-timestamp))) ".owl")))
  (json-prolog:prolog-simple-1 (concatenate 'string "rdf_save(" (concatenate 'string "'/home/ease/logs/" filename "'" "," "[graph('belief_state')])"))))

(defun send-task-success (action-inst is-sucessful)
  (let ((a (convert-to-prolog-str action-inst))
        (b "knowrob:taskSuccess")
        (c (create-owl-literal "xsd:boolean" is-sucessful)))
    (send-rdf-query a b c)))

(defun send-performed-in-projection (action-inst is-performed-in-projection)
  (let ((a (convert-to-prolog-str action-inst))
        (b "knowrob:performedInProjection")
        (c (create-owl-literal "xsd:boolean" is-performed-in-projection)))
    (send-rdf-query a b c)))

(defun send-effort-action-parameter (action-inst effort)
  (let ((a (convert-to-prolog-str action-inst))
        (b "knowrob:effort")
        (c (create-owl-literal
            (convert-to-prolog-str "http://qudt.org/vocab/unit#NewtonMeter") effort)))
    (send-rdf-query a b c)))

(defun send-position-action-parameter (action-inst position)
  (let ((a (convert-to-prolog-str action-inst))
        (b "knowrob:position")
        (c (create-float-owl-literal position)))
    (send-rdf-query a b c)))

(defun send-rdf-query (a b c)
  (send-prolog-query-1 (create-rdf-assert-query a b c)))

(defun send-object-action-parameter (action-inst object-designator)
  (let ((object-instance-id (symbol-name (desig:desig-prop-value object-designator :NAME))))
    (when (not (string-equal object-instance-id "nil"))
      (progn
        (send-rdf-query
         (convert-to-prolog-str action-inst)
         "knowrob:objectActedOn"
         (convert-to-prolog-str object-instance-id)))))

  (let ((object-type (symbol-name (desig:desig-prop-value object-designator :TYPE))))
    (when (not (string-equal object-type "nil"))
      (progn
        (send-rdf-query
         (convert-to-prolog-str action-inst)
         "knowrob:objectType"
         (convert-to-prolog-str object-type))))))


(defun send-create-object (action-inst object-name object-type)
  (let ((object-instance-id (send-instance-from-class "object")))
    (send-rdf-query (convert-to-prolog-str object-instance-id) "knowrob:objectName" (convert-to-prolog-str object-name))
    (send-rdf-query (convert-to-prolog-str object-instance-id) "knowrob:objectType" (convert-to-prolog-str object-type))
    (send-rdf-query (convert-to-prolog-str object-instance-id) "knowrob:action" (convert-to-prolog-str action-inst))
    object-instance-id))

(defun send-cram-next-action (current-action-name next-action-name)
  (send-rdf-query current-action-name "knowrob:nextAction" next-action-name))

(defun send-cram-previous-action (current-action-name previous-action-name)
      (send-rdf-query current-action-name "knowrob:previousAction" previous-action-name))

(defun send-instance-from-class (instance-class-name)
  (let ((instance-class-id
          (get-value-of-json-prolog-dict (cdaar
                                          (ccl::send-prolog-query-1 (concatenate 'string "rdf_instance_from_class(knowrob:\\'" instance-class-name "\\', \\'LoggingGraph\\', ActionInst)."))) "ActionInst")))
    (send-rdf-query (convert-to-prolog-str instance-class-id) "rdf:type" " owl:\\'NamedIndividual\\'")
    instance-class-id))

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
        (frame-id (cl-transforms-stamped:frame-id pose-stamped))
        ;;(stamp (cl-transforms-stamped:stamp pose-stamped))
        (origin (cl-transforms-stamped:origin pose-stamped))
        (orientation (cl-transforms-stamped:orientation pose-stamped)))
    (let ((3d-vector-id (send-create-3d-vector origin))
          (quaternion-id (send-create-quaternion orientation)))
      (if (string-equal frame-id "base_footprint")
          (send-rdf-query (convert-to-prolog-str pose-stamped-instance-id) "knowrob:relativeTo" (convert-to-prolog-str "http://knowrob.org/kb/PR2.owl#pr2_base_footprint"))
          (unless (string-equal frame-id "map")
            (format t "~%~%~%FRAME ~a NOT KNOWN.~%~%~%" frame-id)))
      ;;(send-rdf-query (convert-to-prolog-str pose-stamped-instance-id) "knowrob:stamp" (create-float-owl-literal stamp))
      (send-rdf-query (convert-to-prolog-str pose-stamped-instance-id) "knowrob:translation" (create-string-owl-literal (convert-to-prolog-str 3d-vector-id)))
      (send-rdf-query (convert-to-prolog-str pose-stamped-instance-id) "knowrob:quaternion" (create-string-owl-literal (convert-to-prolog-str quaternion-id))))
    pose-stamped-instance-id))

(defun send-pose-stamped-list-action-parameter (action-inst list-name pose-stamped-list)
  (let ((pose-stamp (get-last-element-in-list pose-stamped-list)))
    (if pose-stamp (progn 
                     (send-rdf-query (convert-to-prolog-str action-inst)
                                     "knowrob:goalLocation" (convert-to-prolog-str (send-create-pose-stamped pose-stamp)))
                     (if (string-equal" left" list-name)
                         (send-gripper-action-parameter action-inst :left)
                         (send-gripper-action-parameter action-inst :right))))))

(defun send-arm-action-parameter (action-inst arm-value)
  (let((arm-value-str (write-to-string arm-value)))
    (cond ((string-equal ":RIGHT" arm-value-str) (send-rdf-query (convert-to-prolog-str action-inst) "knowrob:arm" (convert-to-prolog-str "http://knowrob.org/kb/PR2.owl#pr2_right_arm")))
          ((string-equal ":LEFT" arm-value-str) (send-rdf-query (convert-to-prolog-str action-inst) "knowrob:arm" (convert-to-prolog-str "http://knowrob.org/kb/PR2.owl#pr2_left_arm"))))))

(defun send-gripper-action-parameter (action-inst gripper-value)
  (let((gripper-value-str (write-to-string gripper-value)))
    (cond ((string-equal ":RIGHT" gripper-value-str) (send-rdf-query (convert-to-prolog-str action-inst) "knowrob:bodyPartsUsed" (convert-to-prolog-str "http://knowrob.org/kb/PR2.owl#pr2_right_gripper")))
          ((string-equal ":LEFT" gripper-value-str) (send-rdf-query (convert-to-prolog-str action-inst) "knowrob:bodyPartsUsed" (convert-to-prolog-str "http://knowrob.org/kb/PR2.owl#pr2_left_gripper"))))))

(defun send-location-action-parameter (action-inst location-designator)
  (let ((location-id (send-instance-from-class "ConnectedSpaceRegion"))
        (location-describtion
          (cond ((desig::desig-prop-value location-designator :ON)
                 "onPhysical"))))
    (send-rdf-query (convert-to-prolog-str location-id)
                    (concatenate 'string "knowrob:" location-describtion)
                    (convert-to-prolog-str
                     (concatenate 'string "http://knowrob.org/kb/iai-kitchen.owl#"
                                  (desig::desig-prop-value location-designator :name))))
    (send-rdf-query (convert-to-prolog-str action-inst)
                    "knowrob:goalLocation"
                    (convert-to-prolog-str location-id))))

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
