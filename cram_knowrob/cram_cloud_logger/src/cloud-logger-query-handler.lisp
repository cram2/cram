;;;
;;; Copyright (c) 2017-2022, Sebastian Koralewski <seba@cs.uni-bremen.de>
;;;
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :ccl)

(defparameter *environment-owl* "'package://iai_semantic_maps/owl/kitchen.owl'")
(defparameter *environment-owl-individual-name*
  "'http://knowrob.org/kb/IAI-kitchen.owl#iai_kitchen_room_link'")
(defparameter *environment-urdf* "'package://iai_kitchen/urdf_obj/kitchen.urdf'")
(defparameter *environment-urdf-prefix* "'iai_kitchen/'")

(defparameter *agent-owl* "'package://knowrob/owl/robots/PR2.owl'")
(defparameter *agent-owl-individual-name* "'http://knowrob.org/kb/PR2.owl#PR2_0'")
(defparameter *agent-urdf* "'package://knowrob/urdf/pr2.urdf'")
(defparameter *neem-interface-path* "")

(defun get-parent-folder-path()
  (namestring (physics-utils:parse-uri "package://cram_cloud_logger/src")))

(defun send-load-neem-generation-interface ()
  (if (string= *neem-interface-path* "")
      (let ((path-to-interface-file
             (concatenate 'string "'" (get-parent-folder-path) "/neem-interface.pl'")))
        (ccl::send-query-1-without-result "ensure_loaded" path-to-interface-file))
      (ccl::send-query-1-without-result "ensure_loaded" *neem-interface-path*)
    ))

(defun init-logging ()
  (send-load-neem-generation-interface))

(defun finish-logging ()
  (print "Finished"))
;;(ccl::send-query-1-without-result "ros_logger_stop" ""))

(defun get-grasp-type-lookup-table()
  (let ((lookup-table (make-hash-table :test 'equal)))
    (setf (gethash ":TOP" lookup-table) "TopGrasp")
    (setf (gethash ":TOP-FRONT" lookup-table) "TopFrontGrasp")
    (setf (gethash ":TOP-LEFT" lookup-table) "TopLeftGrasp")
    (setf (gethash ":TOP-RIGHT" lookup-table) "TopRightGrasp")
    (setf (gethash ":BOTTOM" lookup-table) "BottomGrasp")
    (setf (gethash ":LEFT" lookup-table) "LeftGrasp")
    (setf (gethash ":LEFT-SIDE" lookup-table) "LeftSideGrasp")
    (setf (gethash ":RIGHT-SIDE" lookup-table) "RightSideGrasp")
    (setf (gethash ":RIGHT" lookup-table) "RightGrasp")
    (setf (gethash ":FRONT" lookup-table) "FrontGrasp")
    (setf (gethash ":BACK" lookup-table) "BackGrasp")
    lookup-table))


(defun get-object-parameter-role-lookup-table()
  (let ((lookup-table (make-hash-table :test 'equal)))
    (setf (gethash "Lowering" lookup-table) "AlteredObject")
    (setf (gethash "Perceiving" lookup-table) "DetectedObject")
    (setf (gethash "Reaching" lookup-table) "Destination")
    lookup-table))

(defparameter *grasp-type-lookup-table* (get-grasp-type-lookup-table))
(defparameter *object-parameter-role-lookup-table* (get-object-parameter-role-lookup-table))

(defun start-situation (situation-uri)
  (attach-time-to-situation "mem_event_begin" situation-uri))

(defun stop-situation (situation-uri)
  (attach-time-to-situation "mem_event_end" situation-uri))

(defun attach-time-to-situation (predicate-name situation-uri)
  (send-query-1-without-result predicate-name situation-uri))

(defun attach-event-to-situation (event-prolog-url situation-prolog-url)
  (get-url-from-send-query-1 "SubAction"
                             "add_subaction_with_task"
                             situation-prolog-url
                             "SubAction"
                             event-prolog-url))

(defun send-belief-perceived-at (object-type transform rotation object-id)
  (if transform
      (send-query-1-without-result "belief_perceived_at" object-type transform rotation object-id)
      (send-query-1-without-result "belief_perceived_at" object-type object-id))
  object-id)

(defun send-belief-new-object-query (object-type)
  (get-url-from-send-query-1 "Object" "belief_new_object" object-type "Object"))

(defun set-event-status-to-succeeded (event-prolog-url)
  (send-query-1-without-result "mem_event_set_succeeded" event-prolog-url))

(defun set-event-status-to-failed (event-prolog-url)
  (send-query-1-without-result "mem_event_set_failed" event-prolog-url))

(defun send-attach-object-as-parameter-to-situation (object-url parameter-type-url event-prolog-url)
  (send-query-1-without-result "add_participant_with_role"
                               event-prolog-url
                               object-url parameter-type-url))

(defun set-event-diagnosis (event-prolog-url diagnosis-url)
  (send-query-1-without-result "mem_event_add_diagnosis" event-prolog-url diagnosis-url))

(defun start-episode ()
  (setf ccl::*is-logging-enabled* t)
  (ccl::init-logging)
  (when *episode-name*
    (progn
      (roslisp:ros-info (ccl start-episode)
                        "Previous episode recording is still running. Stopping the recording ...")
      (stop-episode)))
  (ccl::clear-detected-objects)
  (setf ccl::*episode-name*
        (get-url-from-send-query-1
         "RootAction"
         "mem_episode_start"
         "RootAction"
         *environment-owl*
         *environment-owl-individual-name*
         *environment-urdf*
         *environment-urdf-prefix*
         *agent-owl*
         *agent-owl-individual-name*
         *agent-urdf*))
  (ccl::start-situation *episode-name*))

(defun stop-episode ()
  (ccl::stop-situation *episode-name*)
  ;;(send-query-1-without-result "delete_episode_name" *episode-name*)
  (send-query-1-without-result "mem_episode_stop"
                               (concatenate 'string "'" (uiop:getenv "KNOWROB_MEMORY_DIR") "'"))
  (setf ccl::*episode-name* nil)
  (setf ccl::*is-logging-enabled* nil))

(defun send-query-1-without-result (query-name &rest query-parameters)
  (let* ((query (create-query query-name query-parameters)))
    (send-query-1 query)))

(defun send-query-1 (query)
  (roslisp:ros-info (ccl send-query-1) "~a" query)
  (json-prolog:prolog-simple-1 query))

(defun get-url-from-send-query-1 (url-parameter query-name &rest query-parameters)
  (let* ((query (create-query query-name query-parameters))
         (query-result (send-query-1 query)))
    (when (eq query-result nil)
      ;(break) TODO remove?
      (print "query result was nil. ignoring")
      )
    (ccl::get-url-variable-result-as-str-from-json-prolog-result url-parameter query-result)))

(defun send-comment (action-inst comment)
  (send-query-1-without-result "add_comment" action-inst (concatenate 'string "'"comment"'")))

(defun send-container-object-action-parameter (action-inst action-type object-designator)
  (declare (ignore action-type))
  (format t "IN CONTAINER OBJ with obj-desig: ~a~%" object-designator)

  (let* ((owl-name (get-designator-property-value-str object-designator :URDF-NAME))
        (soma-name (get-ease-object-id-of-cram-object-by-name owl-name))); was OWL-name
    (when owl-name
      (send-query-1-without-result "add_participant_with_role"
                                   action-inst
                                   soma-name
                                   "'http://www.ease-crc.org/ont/SOMA.owl#AlteredObject'"))))

(defun send-object-action-parameter (action-inst action-type object-designator)
  (when object-designator
   (let* ((role (gethash action-type *object-parameter-role-lookup-table*))
          (object-name (get-designator-property-value-str object-designator :NAME))
          (object-ease-id (get-ease-object-id-of-detected-object-by-name object-name))
          (soma-name (get-ease-object-id-of-cram-object-by-name object-name)))
     (roslisp:ros-info (ccl send-object-action-parameter)
                       "object-name:~%~a" object-name)
     (when (not object-ease-id)
       (let ((owl-name (get-designator-property-value-str object-designator :OWL-NAME))) ;;was OWL-NAME
         (when owl-name
           (roslisp:ros-info (ccl send-object-action-parameter)
                             "owl-name:~%~a" owl-name)
           (progn
             (format t "+++ OBJ-NAME +++: ~a" soma-name)     
                  (send-query-1-without-result "add_participant_with_role"
                                        action-inst
                                        soma-name ;;owl-name
                                        "'http://www.ease-crc.org/ont/SOMA.owl#AlteredObject'")
                  (format t "+++ PARTICIPANT+++: ~a" object-name)))))
     
     (when object-ease-id
       (roslisp:ros-info (ccl send-object-action-parameter)
                         "object-ease-id:~%~a" object-ease-id)
       (if (and (not role) object-ease-id)
           (send-query-1-without-result "add_participant_with_role"
                                        action-inst
                                        soma-name ;; was object-ease-id
                                        "'http://www.ease-crc.org/ont/SOMA.owl#Item'")
           (send-query-1-without-result "add_participant_with_role"
                                        action-inst
                                        soma-name;; was object-ease-id
                                        (concatenate 'string
                                                     "'http://www.ease-crc.org/ont/SOMA.owl#"
                                                     role "'")))))))

(defun send-object-name-action-parameter (action-inst object-name)
  (let* ((object-ease-id (get-ease-object-id-of-detected-object-by-name object-name)))
    (when object-ease-id
      (send-query-1-without-result "add_participant_with_role"
                                   action-inst
                                   object-ease-id
                                   "'http://www.ease-crc.org/ont/SOMA.owl#AffectedObject'"))
    (when (not object-ease-id)
      (send-query-1-without-result "add_participant_with_role"
                                   action-inst
                                   (get-ease-object-id-of-cram-object-by-name object-name)
                                   "'http://www.ease-crc.org/ont/SOMA.owl#AffectedObject'"))))

(defun send-grasp-action-parameter (action-inst action-type grasp)
  (declare (ignore action-type))
  (let ((grasp-type (gethash (write-to-string grasp) *grasp-type-lookup-table*)))
    (send-parameter action-inst grasp-type)))

(defun send-parameter(action-inst region-type)
  (let ((region-type-url
          (concatenate 'string "'" "http://www.ease-crc.org/ont/SOMA.owl#" region-type "'")))
    (send-query-1-without-result "add_grasping_parameter" action-inst region-type-url)))








;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; CLOUD LOGGER RELATED STUFF (OLD) ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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

(defun send-cram-set-submotion (sup sub)
  (let ((a  sup)
        (b "knowrob:subMotion")
        (c  sub))
    (send-rdf-query a b c)))

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
            (convert-to-prolog-str "http://qudt.org/vocab/unit#NewtonMeter") (write-to-string effort))))
    (send-rdf-query a b c)))

(defun send-position-action-parameter (action-inst position)
  (let ((a (convert-to-prolog-str action-inst))
        (b "knowrob:position")
        (c (create-float-owl-literal position)))
    (send-rdf-query a b c)))

(defun send-rdf-query (a b c)
  (send-prolog-query-1 (create-rdf-assert-query a b c)))

;;(defun send-object-action-parameter (action-inst object-designator)
;;  (let ((object-instance-id (symbol-name (desig:desig-prop-value object-designator :NAME))))
;;    (when (not (string-equal object-instance-id "nil"))
;;      (progn
;;        (send-rdf-query
;;         (convert-to-prolog-str action-inst)
;;         "knowrob:objectActedOn"
;;         (convert-to-prolog-str object-instance-id)))))

;;  (let ((object-type (symbol-name (desig:desig-prop-value object-designator :TYPE))))
;;    (when (not (string-equal object-type "nil"))
;;      (progn
;;        (send-rdf-query
;;         (convert-to-prolog-str action-inst)
;;         "knowrob:objectType"
;;         (convert-to-prolog-str object-type))))))




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

(defun send-cram-next-motion (current-motion-name next-motion-name)
  (send-rdf-query current-motion-name "knowrob:nextMotion" next-motion-name))

(defun send-cram-previous-motion (current-motion-name previous-motion-name)
  (send-rdf-query current-motion-name "knowrob:previousMotion" previous-motion-name))

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
    (concatenate 'string "["(format nil "~F" x) "," (format nil "~F" y) "," (format nil "~F" z)"]")))


(defun send-create-quaternion (quaternion)
  (let ((x (cl-transforms:x quaternion))
        (y (cl-transforms:y quaternion))
        (z (cl-transforms:z quaternion))
        (w (cl-transforms:w quaternion)))
        (concatenate 'string "["(format nil "~F" x) "," (format nil "~F" y) "," (format nil "~F" z) "," (format nil "~F" w) "]")))

(defun send-create-transform-pose-stamped (transform-pose)
  (let ((pose-stamped-instance-id (send-instance-from-class "Pose"))
        (frame-id (cl-transforms-stamped:frame-id transform-pose))
        (3d-vector (cl-transforms:translation transform-pose))
        (orientation (cl-transforms:rotation transform-pose))
        (child-frame (cl-transforms-stamped:child-frame-id transform-pose))
        )
    (let ((3d-vector-id (send-create-3d-vector 3d-vector))
          (quaternion-id (send-create-quaternion orientation)))
      (send-rdf-query (convert-to-prolog-str pose-stamped-instance-id) "knowrob:relativeTo" (convert-to-prolog-str "http://knowrob.org/kb/PR2.owl#pr2_base_footprint"))
      (send-rdf-query (convert-to-prolog-str pose-stamped-instance-id) "knowrob:translation" (create-string-owl-literal (convert-to-prolog-str 3d-vector-id)))
      (send-rdf-query (convert-to-prolog-str pose-stamped-instance-id) "knowrob:quaternion" (create-string-owl-literal (convert-to-prolog-str quaternion-id)))
      (send-rdf-query (convert-to-prolog-str pose-stamped-instance-id) "knowrob:child-id" (create-string-owl-literal (convert-to-prolog-str child-frame)))
      pose-stamped-instance-id)))

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

(defun send-left-pose-stamped-list-action-parameter (action-inst pose-stamped-list)
  (send-pose-stamped-list-action-parameter action-inst "left" pose-stamped-list))

(defun send-right-pose-stamped-list-action-parameter (action-inst pose-stamped-list)
  (send-pose-stamped-list-action-parameter action-inst "right" pose-stamped-list))

(defun send-pose-stamped-list-action-parameter (action-inst list-name pose-stamped-list)
  (let ((pose-stamp (get-last-element-in-list pose-stamped-list)))
    (if pose-stamp (progn
                     (send-rdf-query (convert-to-prolog-str action-inst)
                                     "knowrob:goalLocation" (convert-to-prolog-str (send-create-pose-stamped pose-stamp)))
                     (if (string-equal "left" list-name)
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
   (send-location-designator action-inst location-designator "knowrob:goalLocation"))

(defun send-target-action-parameter (action-inst location-designator)
  (send-location-designator action-inst location-designator "knowrob:targetLocation"))

(defun send-finish-query(id)
  (json-prolog:prolog-simple-1 (concatenate 'string "send_finish_query('" id "').")))

(defun send-location-designator (action-id designator predicate-name)
  (if (desig::desig-prop-value designator :POSE)
      (let ((pose-id (send-create-pose-stamped
                      (desig::desig-prop-value designator :POSE))))
          (send-rdf-query (convert-to-prolog-str action-id)
                          "knowrob:goalLocation"
                          (convert-to-prolog-str pose-id)))
      (let ((location-id (send-instance-from-class "ConnectedSpaceRegion")))
        (mapcar (lambda (key-value-pair)
                  (let ((key (first key-value-pair))
                        (value (second key-value-pair)))
                    (cond ((eq key :on)
                           (send-rdf-query (convert-to-prolog-str location-id)
                                           "knowrob:onPhysical"
                                           (convert-to-prolog-str (get-object-name value))))
                          ((eq key :in)
                           (send-rdf-query (convert-to-prolog-str location-id)
                                           "knowrob:in"
                                           (convert-to-prolog-str (get-object-name value))))
                          ((eq key :pose)
                           (let ((pose-id
                                   (send-create-pose-stamped value)))
                             (send-rdf-query (convert-to-prolog-str location-id)
                                             "knowrob:goalLocation"
                                             (convert-to-prolog-str pose-id))))
                          ((eq key :visible-for)
                           (send-rdf-query (convert-to-prolog-str location-id)
                                           "knowrob:inFieldOfView"
                                           (convert-to-prolog-str "http://knowrob.org/kb/PR2.owl#PR2Robot1")))
                          ((eq key :reachable-for)
                           (send-rdf-query (convert-to-prolog-str location-id)
                                           "knowrob:inReachOf"
                                           (convert-to-prolog-str "http://knowrob.org/kb/PR2.owl#PR2Robot1")))
                          ((eq key :arm)
                           (send-arm-action-parameter location-id value))
                          ((eq key :location)
                           (send-location-designator location-id value "knowrob:goalLocation"))
                          ((eq key :right-of)
                           (send-rdf-query (convert-to-prolog-str location-id)
                                           "knowrob:rightOf"
                                           (convert-to-prolog-str (get-object-name value))))
                          ((eq key :left-of)
                           (send-rdf-query (convert-to-prolog-str location-id)
                                           "knowrob:leftOf"
                                           (convert-to-prolog-str (get-object-name value))))
                          ((eq key :behind)
                           (send-rdf-query (convert-to-prolog-str location-id)
                                           "knowrob:behind"
                                           (convert-to-prolog-str (get-object-name value))))
                          ((eq key :near)
                           (send-rdf-query (convert-to-prolog-str location-id)
                                           "knowrob:near"
                                           (convert-to-prolog-str (get-object-name value))))
                          ((eq key :far-from)
                           (send-rdf-query (convert-to-prolog-str location-id)
                                           "knowrob:farFrom"
                                           (convert-to-prolog-str (get-object-name value))))
                          ((eq key :context)
                           (print key))
                          ((eq key :for)
                           (send-rdf-query (convert-to-prolog-str location-id)
                                           "knowrob:for"
                                           (convert-to-prolog-str (get-object-name value))))
                          ((eq key :object-count)
                           (print key))
                          ((eq key :side)
                           (send-rdf-query (convert-to-prolog-str location-id)
                                           "knowrob:side"
                                           (convert-to-prolog-str (symbol-name value)))))))
                (desig:properties designator))
        (send-rdf-query (convert-to-prolog-str action-id) predicate-name (convert-to-prolog-str location-id)))))



(defun get-object-name (object-designator)
   (let ((object-instance-id (symbol-name (desig:desig-prop-value object-designator :NAME))))
     (if (not (string-equal object-instance-id "nil"))
         object-instance-id
         (symbol-name (desig:desig-prop-value object-designator :TYPE)))))
