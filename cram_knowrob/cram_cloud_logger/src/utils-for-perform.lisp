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
    lookup-table))


(defun get-mesh-lookup-table()
  (let ((lookup-table (make-hash-table :test 'equal)))
    (setf (gethash "BOWL" lookup-table) "'package://kitchen_object_meshes/bowl.dae'")
    (setf (gethash "CUP" lookup-table) "'package://kitchen_object_meshes/cup.dae'")
    (setf (gethash "MILK" lookup-table) "'package://kitchen_object_meshes/milk.dae'")
    (setf (gethash "SPOON" lookup-table) "'package://kitchen_object_meshes/spoon.dae'")
    (setf (gethash "BREAKFAST-CEREAL" lookup-table) "'package://kitchen_object_meshes/cereal.dae'")
    lookup-table))


(defun get-rotation-lookup-table()
  (let ((lookup-table (make-hash-table :test 'equal)))
    (setf (gethash "BOWL" lookup-table) "[-1.0,0.0,0.0,1.0]")
    (setf (gethash "CUP" lookup-table) "[-1.0,0.0,0.0,1.0]")
    (setf (gethash "MILK" lookup-table) "[0.0,0.0,0.0,1.0]")
    (setf (gethash "SPOON" lookup-table) "[-1.0,0.0,0.0,1.0]")
    (setf (gethash "BREAKFAST-CEREAL" lookup-table) "[0.0,0.0,0.0,1.0]")
    lookup-table))

(cpl:define-task-variable *action-parents* '())
(defparameter *action-siblings* (make-hash-table))
(defparameter *detected-objects* (make-hash-table :test 'equal))
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
  (let* ((object-name (get-designator-property-value-str detected-object :NAME))
        (detected-object-type (get-designator-property-value-str detected-object :TYPE))
        (object-type
          (convert-to-ease-object-type-url detected-object-type)))
    (print detected-object-type)
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


(defmethod exe:generic-perform :around ((designator desig:action-designator))
  (if *is-logging-enabled*
      (let ((action-id (log-perform-call designator))
            (cram-action-name (get-designator-property-value-str designator :TYPE)))
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
            (let ((referenced-action-id "")
                  (action-designator-parameters (desig:properties (or action-desig designator))))
              (print "HERE 2")
              (log-action-designator-parameters-for-logged-action-designator action-designator-parameters action-id)
              (when (string-equal cram-action-name "grasping")
                (print action-designator-parameters))
              (when (string-equal cram-action-name "detecting")
                (handle-detected-object perform-result))
              (print "HERE 3")
              (set-event-status-to-succeeded action-id)
              (print "HERE 4")
              (ccl::stop-situation action-id)
              (print "HERE 5")
              perform-result))))
       (cpl:with-failure-handling
            ((cpl:plan-failure (e)
               (setf *retry-numbers* (+ 1 *retry-numbers*))
               (print "plan failure")))
         (call-next-method))))

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
           (setf motion t)))
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
  ;;(if (is-motion current-knowrob-name)
  (if nil
      (send-cram-previous-motion (convert-to-prolog-str current-id) (convert-to-prolog-str previous-id))
      (send-cram-previous-action (convert-to-prolog-str current-id) (convert-to-prolog-str previous-id))))

(defun log-cram-next-action (current-id next-id current-knowrob-name)
  ;;(if (is-motion current-knowrob-name)
  (if nil
      (send-cram-next-motion (convert-to-prolog-str current-id) (convert-to-prolog-str next-id))
      (send-cram-next-action (convert-to-prolog-str current-id) (convert-to-prolog-str next-id))))
