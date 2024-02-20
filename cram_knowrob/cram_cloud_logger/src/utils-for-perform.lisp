;;;
;;; Copyright (c) 2017-2022, Sebastian Koralewski <seba@cs.uni-bremen.de>
;;;                          Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defun get-ease-object-lookup-table()
  (let ((lookup-table (make-hash-table :test 'equal)))
    (setf (gethash "BOWL" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#Bowl'")
    (setf (gethash "CUP" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#Cup'")
    (setf (gethash "DRAWER" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#Drawer'")
    (setf (gethash "MILK" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#Milk'")
    (setf (gethash "SPOON" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#Spoon'")
    (setf (gethash "BREAKFAST-CEREAL" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#Cereal'")
    ;; for popcorn demo
    (setf (gethash "IKEA-BOWL" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#IkeaBowl'")
    (setf (gethash "IKEA-BOWL-WW" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#IkeaBowlWw'")
    (setf (gethash "IKEA-PLATE" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#IkeaPlate'")
    (setf (gethash "SALT" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#Salt'")
    (setf (gethash "POPCORN-POT" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#PopcornPot'")
    (setf (gethash "POPCORN-POT-LID" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#PopcornPotLid'")
    ;;individuals
    (setf (gethash "IKEA-BOWL-1" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#IkeaBowl1'")
    (setf (gethash "IKEA-BOWL-WW-1" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#IkeaBowlWw1'")
    (setf (gethash "IKEA-PLATE-1" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#IkeaPlate1'")
    (setf (gethash "SALT-1" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#Salt1'")
    (setf (gethash "POPCORN-POT-1" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#PopcornPot1'")
    (setf (gethash "POPCORN-POT-LID-1" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#PopcornPotLid1'")
    ;;super hacky fix
    (setf (gethash "IAI-KITCHEN" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#IAI-KitchenPart'")
    (setf (gethash "IAI-POPCORN-TABLE-DRAWER-RIGHT-MAIN" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#IAIPopcornTableDrawerRightMain'")
    (setf (gethash "IAI-POPCORN-TABLE-DRAWER-LEFT-MAIN" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#IAIPopcornTableDrawerLeftMain'")
    ;;apartment demo pouring
    (setf (gethash "HANLDE-CAB1-TOP-DOOR" lookup-table) "'http://knowrob.org/kb/iai-apartment.owl#handle_cab1_top_door'")
    (setf (gethash "JEROEN-CUP" lookup-table) "'http://www.ease-crc.org/ont/SOMA.owl#Cup'")
    (setf (gethash "APARTMENT" lookup-table) "'http://knowrob.org/kb/iai-apartment.owl#apartment_root'")

    lookup-table))


(defun get-mesh-lookup-table()
  (let ((lookup-table (make-hash-table :test 'equal)))
    (setf (gethash "BOWL" lookup-table) "'package://kitchen_object_meshes/bowl.dae'")
    (setf (gethash "CUP" lookup-table) "'package://kitchen_object_meshes/cup.dae'")
    (setf (gethash "MILK" lookup-table) "'package://kitchen_object_meshes/milk.dae'")
    (setf (gethash "SPOON" lookup-table) "'package://kitchen_object_meshes/spoon.dae'")
    (setf (gethash "BREAKFAST-CEREAL" lookup-table) "'package://kitchen_object_meshes/cereal.dae'")
    ;; for popcorn demo
    (setf (gethash "IKEA-BOWL" lookup-table) "'package://cram_pr2_popcorn_demo/resource/ikea_bowl.dae'")
    (setf (gethash "IKEA-BOWL-WW" lookup-table) "'package://cram_pr2_popcorn_demo/resource/ikea_bowl_ww.dae'")
    (setf (gethash "IKEA-PLATE" lookup-table) "'package://cram_pr2_popcorn_demo/resource/ikea_plate.dae'")
    (setf (gethash "SALT" lookup-table) "'package://cram_pr2_popcorn_demo/resource/salt.dae'")
    (setf (gethash "POPCORN-POT" lookup-table) "'package://cram_pr2_popcorn_demo/resource/popcorn_pot.dae'")
    (setf (gethash "POPCORN-POT-LID" lookup-table) "'package://cram_pr2_popcorn_demo/resource/popcorn_pot_lid.dae'")
    ;;individuals
    (setf (gethash "IKEA-BOWL-1" lookup-table) "'package://cram_pr2_popcorn_demo/resource/ikea_bowl.dae'")
    (setf (gethash "IKEA-BOWL-WW-1" lookup-table) "'package://cram_pr2_popcorn_demo/resource/ikea_bowl_ww.dae'")
    (setf (gethash "IKEA-PLATE-1" lookup-table) "'package://cram_pr2_popcorn_demo/resource/ikea_plate.dae'")
    (setf (gethash "SALT-1" lookup-table) "'package://cram_pr2_popcorn_demo/resource/salt.dae'")
    (setf (gethash "POPCORN-POT-1" lookup-table) "'package://cram_pr2_popcorn_demo/resource/popcorn_pot.dae'")
    (setf (gethash "POPCORN-POT-LID-1" lookup-table) "'package://cram_pr2_popcorn_demo/resource/popcorn_pot_lid.dae'")

    (setf (gethash "JEROEN-CUP" lookup-table) "'package://cram_projection_demos/resource/household/jeroen_cup.dae'")
    lookup-table))


(defun get-rotation-lookup-table()
  (let ((lookup-table (make-hash-table :test 'equal)))
    (setf (gethash "BOWL" lookup-table) "[-1.0,0.0,0.0,1.0]")
    (setf (gethash "CUP" lookup-table) "[-1.0,0.0,0.0,1.0]")
    (setf (gethash "MILK" lookup-table) "[0.0,0.0,0.0,1.0]")
    (setf (gethash "SPOON" lookup-table) "[-1.0,0.0,0.0,1.0]")
    (setf (gethash "BREAKFAST-CEREAL" lookup-table) "[0.0,0.0,0.0,1.0]")
    ;; for popcorn demo
    (setf (gethash "IKEA-BOWL" lookup-table) "[0.0,0.0,0.0,1.0]")
    (setf (gethash "IKEA-BOWL-WW" lookup-table) "[0.0,0.0,0.0,1.0]")
    (setf (gethash "IKEA-PLATE" lookup-table) "[0.0,0.0,0.0,1.0]")
    (setf (gethash "SALT" lookup-table) "[0.0,0.0,0.0,1.0]")
    (setf (gethash "POPCORN-POT" lookup-table) "[0.0,0.0,0.0,1.0]")
    (setf (gethash "POPCORN-POT-LID" lookup-table) "[0.0,0.0,0.0,1.0]")
    ;;individuals
    (setf (gethash "IKEA-BOWL-1" lookup-table) "[0.0,0.0,0.0,1.0]")
    (setf (gethash "IKEA-BOWL-WW-1" lookup-table) "[0.0,0.0,0.0,1.0]")
    (setf (gethash "IKEA-PLATE-1" lookup-table) "[0.0,0.0,0.0,1.0]")
    (setf (gethash "SALT-1" lookup-table) "[0.0,0.0,0.0,1.0]")
    (setf (gethash "POPCORN-POT-1" lookup-table) "[0.0,0.0,0.0,1.0]")
    (setf (gethash "POPCORN-POT-LID-1" lookup-table) "[0.0,0.0,0.0,1.0]")

    (setf (gethash "JEROEN-CUP" lookup-table) "[-1.0,0.0,0.0,1.0]")
    lookup-table))

;;another hack
(defun get-detected-objects-table()
  (let ((det-obj (make-hash-table :test 'equal)))
    (setf (gethash "IKEA-BOWL" det-obj) "IKEA-BOWL-1")
    (setf (gethash "IKEA-BOWL-WW" det-obj) "IKEA-BOWL-WW-1")
    (setf (gethash "IKEA-PLATE" det-obj) "IKEA-PLATE-1")
    (setf (gethash "SALT" det-obj) "SALT-1")
    (setf (gethash "POPCORN-POT" det-obj) "POPCORN-POT-1")
    (setf (gethash "POPCORN-POT-LID" det-obj) "POPCORN-POT-LID-1")
    (setf (gethash "BOWL" det-obj) "BOWL-1")
    (setf (gethash "JEROEN-CUP" det-obj) "JEROEN-CUP-1")
    
    det-obj))
    

(cpl:define-task-variable *action-parents* '())
(defparameter *action-siblings* (make-hash-table))
(defparameter *detected-objects* (get-detected-objects-table)) ;;(make-hash-table :test 'equal))
(defparameter *episode-name* nil)
(defparameter *is-logging-enabled* nil)
(defparameter *retry-numbers* 0)
(defparameter *ease-object-lookup-table* (get-ease-object-lookup-table))
(defparameter *mesh-lookup-table* (get-mesh-lookup-table))
(defparameter *rotation-lookup-table* (get-rotation-lookup-table))


(defun clear-detected-objects ()
  (setf *detected-objects* (make-hash-table :test 'equal)))

(defun get-ease-object-id-of-detected-object-by-name (object-name)
  (gethash object-name *detected-objects*))

;;match CRAM to EASE Obj
(defun get-ease-object-id-of-cram-object-by-name (object-name)
  (format t "Object-name: ~a~%" object-name)
  (let* ((owl-name (gethash object-name *ease-object-lookup-table*)))
    owl-name))

(defun get-parent-uri()
  (if (is-action-parent)
      *episode-name*
      (car *action-parents*)))

(defun get-transform-of-detected-object (detected-object)
  (let*
      ((detected-object-transform (man-int:get-object-transform-in-map detected-object))
       (translate (cl-transforms-stamped:translation detected-object-transform))
       (quaternion (cl-transforms-stamped:rotation detected-object-transform)))
    (concatenate
     'string "['map',"
     (send-create-3d-vector translate) ","
     (send-create-quaternion quaternion)"]")))

(defun is-action-parent ()
  (if (not *action-parents*) t nil))

(defun convert-to-ease-object-type-url (object-type)
  (if (gethash object-type *ease-object-lookup-table*)
      (gethash object-type *ease-object-lookup-table*)
      "'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#DesignedArtifact'"))

(defun handle-detected-object (detected-object)
  (format t " handle detected object: ~a~%" detected-object)
  (format t "object: ~%~a~%" (desig:desig-prop-values detected-object :object))
  (let* ((object (car (desig:desig-prop-values detected-object :object)))
        (object-name (get-designator-property-value-str object :NAME))
        (detected-object-type (get-designator-property-value-str object :TYPE))
        (object-type
          (convert-to-ease-object-type-url detected-object-type)))
    (print detected-object-type)
    (print "loggin object")
    (if (gethash object-name *detected-objects*)
        (print "Object exists")
        (let ((object-id (send-belief-perceived-at object-type
                                                   (gethash detected-object-type *mesh-lookup-table*)
                                                   (gethash detected-object-type *rotation-lookup-table*)
                                                   (concatenate 'string
                                                                "'" "http://www.ease-crc.org/ont/SOMA.owl#"
                                                                (roslisp-utilities:rosify-underscores-lisp-name (make-symbol object-name)) "'"))))
          (setf (gethash object-name *detected-objects*) object-id)
          (when (string-equal object-type "'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#DesignedArtifact'")
            (send-comment object-id (concatenate 'string "Unknown Object: "(write-to-string detected-object-type))))))))

;++++++++++++MAIN WRITER+++++++++++++++++++++++++++++++++++
(defmethod exe:generic-perform :around ((designator desig:action-designator))
  (if *is-logging-enabled*
      (let ((action-id (log-perform-call designator))
            (cram-action-name (get-designator-property-value-str designator :TYPE)))
        ;;; publish designator data to topic
        ;(publisher (string-downcase (desig-to-string designator)))
        ;;; end publish designator data to topic
        (format t "[CCL] entered generic perform for action designator: ~a~%" designator)
        (cpl:with-failure-handling
            ((cpl:plan-failure (e)
               (set-event-status-to-failed action-id)
               (set-event-diagnosis action-id (ccl::get-failure-uri (subseq (write-to-string e) 2 (search " " (write-to-string e)))))
               (let ((action-designator-parameters (desig:properties (or (second (desig:reference designator)) designator))))
                 (log-action-designator-parameters-for-logged-action-designator action-designator-parameters action-id))
                (ccl::stop-situation action-id)
               (print "plan failure")))

          (push action-id *action-parents*)
          (ccl::start-situation action-id)
          (print "HERE 0")
          (multiple-value-bind (perform-result action-desig)
              (call-next-method)
            (format t "--- perform result: ~a" perform-result)
            (format t "--- action desig: ~a" action-desig)
            (let ((referenced-action-id "")
                  (action-designator-parameters (desig:properties (or action-desig designator))))
              (print "HERE 2")
              (log-action-designator-parameters-for-logged-action-designator action-designator-parameters action-id)
              (when (string-equal cram-action-name "grasping")
                (print action-designator-parameters))
              (when (string-equal cram-action-name "picking-up")
                (handle-detected-object designator))
              ;; (when (and (string-equal cram-action-name "looking") action-desig) ;;make sure it's an action not a motion
              ;;   (handle-detected-object perform-result))
              (print "HERE 3")
              (set-event-status-to-succeeded action-id)
              (print "HERE 4")
              (ccl::stop-situation action-id)
              (print "HERE 5")
              (format t "--- perform result after: ~a~% " perform-result)
              perform-result))))
       (cpl:with-failure-handling
            ((cpl:plan-failure (e)
               (setf *retry-numbers* (+ 1 *retry-numbers*))
               (print "plan failure")))
         (call-next-method))))

;;+++++ write motions ++++++
;; (defmethod exe:generic-perform :around ((designator desig:motion-designator))
;;   (if *is-logging-enabled*
;;         ;;; publish designator data to topic
;;       (publisher (string-downcase (desig-to-string designator)))
;;       (call-next-method)))


(defun equate (designator-id referenced-designator-id)
  (send-rdf-query (convert-to-prolog-str designator-id)
                    "knowrob:equate"
                    (convert-to-prolog-str referenced-designator-id)))

(defun log-perform-call (designator)
  (if *is-logging-enabled*
      (let* ((cram-action-name (get-knowrob-action-name-uri (get-designator-property-value-str designator :TYPE) designator))
             (event-name-url (attach-event-to-situation cram-action-name (get-parent-uri))))
        (when (string-equal cram-action-name "'http://www.ease-crc.org/ont/SOMA.owl#PhysicalTask'")
          (send-comment event-name-url (concatenate 'string "Unknown Action: "  (get-designator-property-value-str designator :TYPE))))
        event-name-url)
      "NOLOGGING"))

(defun log-failure (action-id failure-type)
  (let ((failure-str (write-to-string failure-type)))
    (send-rdf-query (convert-to-prolog-str action-id)
                    "knowrob:failure"
                    (convert-to-prolog-str (subseq failure-str 2 (search " " failure-str))))))

(defun log-cram-finish-action (action-id)
  (send-cram-finish-action
   (convert-to-prolog-str action-id ) (convert-to-prolog-str (get-timestamp-for-logging))))

(defun log-cram-sub-action (parent-id child-id child-knowrob-action-name)
  (if parent-id
      ;;Motion is hacked currently, we need a cleaner implementiaon
      ;;(if (is-motion child-knowrob-action-name)
      (if nil
          (progn
            (send-cram-set-submotion
             (convert-to-prolog-str parent-id)
             (convert-to-prolog-str child-id)))
          (progn
            (send-cram-set-subaction
             (convert-to-prolog-str parent-id)
             (convert-to-prolog-str child-id))))))
;; why ???
(defun is-motion (knowrob-action-name)
  (let ((motion nil))
    (cond ((string-equal knowrob-action-name "BaseMovement")
           (setf motion t))
          ((string-equal knowrob-action-name "OpeningAGripper")
           (setf motion t))
          ((string-equal knowrob-action-name "Reaching")
           (setf motion t))
          ((string-equal knowrob-action-name "SettingAGripper")
           (setf motion t))
          ((string-equal knowrob-action-name "LiftingAnArm")
           (setf motion t))
          ((string-equal knowrob-action-name "LoweringAnArm")
           (setf motion t))
          ((string-equal knowrob-action-name "ClosingAGripper")
           (setf motion t))
          ((string-equal knowrob-action-name "LookingAtLocation")
           (setf motion t))
          ((string-equal knowrob-action-name "Pushing")
           (setf motion t))
          ((string-equal knowrob-action-name "Pulling")
           (setf motion t))
          ((string-equal knowrob-action-name "Retracting")
           (setf motion t))
          ;;added for pouring demo
          ((string-equal knowrob-action-name "MovingArmJoints")
           (setf motion t))
          ((string-equal knowrob-action-name "MovingTCP")
           (setf motion t))
          ((string-equal knowrob-action-name "MovingGripperJoint")
           (setf motion t))
          ;;is also an action. Same for detecting
          ;;((string-equal knowrob-action-name "Gripping")
          ;; (setf motion t))
          )
    motion))

(defun log-cram-sibling-action (parent-id child-id child-knowrob-name)
  (let ((hash-value (gethash parent-id *action-siblings*)))
    (if hash-value
      (let ((previous-id (car (cpl:value hash-value))))
          (progn (log-cram-prev-action
                  child-id previous-id child-knowrob-name)
                 (log-cram-next-action
                  previous-id child-id child-knowrob-name)
                 (setf (cpl:value hash-value) (cons child-id (cpl:value hash-value)))
                 (setf  (gethash parent-id *action-siblings*) hash-value)))
      (setf (gethash parent-id *action-siblings*) (cpl:make-fluent :name parent-id :value (cons child-id '()))))))

(defun log-cram-prev-action (current-id previous-id current-knowrob-name)
  (if (is-motion current-knowrob-name)
 ;; (if nil
      (send-cram-previous-motion (convert-to-prolog-str current-id) (convert-to-prolog-str previous-id)) 
      (send-cram-previous-action (convert-to-prolog-str current-id) (convert-to-prolog-str previous-id))))

(defun log-cram-next-action (current-id next-id current-knowrob-name)
  (if (is-motion current-knowrob-name)
  ;; (if nil
      (send-cram-next-motion (convert-to-prolog-str current-id) (convert-to-prolog-str next-id))
      (send-cram-next-action (convert-to-prolog-str current-id) (convert-to-prolog-str next-id))))



;;; debugging queries
(defun all-nil-triples()
  (json-prolog:prolog-simple "findall([S, P, O], (triple(S, P, O), (S='NIL'; O='NIL';P='NIL')), Triples)."))

(defun all-events-without-task()
  (json-prolog:prolog-simple "findall(Evt,(is_event(Evt), not(executes_task(Evt,_))), List), list_to_set(List, Partici)."))

(defun all-used-role-types()
  (json-prolog:prolog-simple "findall(RoleType, (has_role(_, Role), has_type(Role, RoleType)), RlT), list_to_set(RlT, Roles)."))

(defun roles-mapped-to-tasks()
  (json-prolog:prolog-simple "findall((Tsk, Roles), (executes_task(Evt, Tsk), findall(Role, (has_participant(Evt, Parti), kb_call(during([triple(RoleIndi, dul:classifies,Parti)], Evt)), has_type(RoleIndi, Role)), RoleList), list_to_set(RoleList, Roles)), TskRL)."))

(defun remember(name)
  (json-prolog:prolog-simple
   (concatenate 'string "remember('/home/ahawkin/ros_ws/neems_library/pr2_pouring_simulation/" name "').")))

