;;;
;;; Copyright (c) 2017, Sebastian Koralewski <seba@cs.uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;       specific prior written permission
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

(cpl:define-task-variable *action-parents* '())
(defparameter *action-siblings* (make-hash-table))
(defparameter *prolog-queries* (cpl:make-fluent :value '()))


(defun get-designator-property-value-str (designator property-keyname)
  (string (cadr (assoc property-keyname (desig:properties designator)))))

(defun get-knowrob-action-name (cram-action-name)
  (let ((knowrob-action-name cram-action-name))
    (cond ((string-equal cram-action-name "reaching")
           (setf knowrob-action-name "Reaching"))
          ((string-equal cram-action-name "retracting")
           (setf knowrob-action-name "Retracting"))
          ((string-equal cram-action-name "lifting")
           (setf knowrob-action-name "LiftingAGripper"))
          ((string-equal cram-action-name "putting")
           (setf knowrob-action-name "SinkingAGripper"))
          ((string-equal cram-action-name "setting-gripper")
           (setf knowrob-action-name "SettingAGripper"))
          ((string-equal cram-action-name "opening")
           (setf knowrob-action-name "OpeningAGripper"))
          ((string-equal cram-action-name "closing")
           (setf knowrob-action-name "ClosingAGripper"))
          ((string-equal cram-action-name "detecting")
           (setf knowrob-action-name "VisualPerception"))
          ((string-equal cram-action-name "placing")
           (setf knowrob-action-name "VoluntaryBodyMovement"))
          ((string-equal cram-action-name "picking-up")
           (setf knowrob-action-name "VoluntaryBodyMovement"))
          ((string-equal cram-action-name "releasing")
           (setf knowrob-action-name "ReleasingGraspOfSomething"))
          ((string-equal cram-action-name "gripping")
           (setf knowrob-action-name "AcquireGraspOfSomething"))
          ((string-equal cram-action-name "looking")
           (setf knowrob-action-name "LookingAtLocation"))
          ((string-equal cram-action-name "going")
           (setf knowrob-action-name "MovingToLocation"))
          ((string-equal cram-action-name "navigating")
           (setf knowrob-action-name "NavigatingToLocation"))
          ((string-equal cram-action-name "searching")
           (setf knowrob-action-name "LookingForSomething"))
          ((string-equal cram-action-name "fetching")
           (setf knowrob-action-name "PickingUpAnObject"))
          ((string-equal cram-action-name "delivering")
           (setf knowrob-action-name "PuttingDownAnObject"))
          ((string-equal cram-action-name "fetching-and-delivering")
           (setf knowrob-action-name "FetchAndDeliver")))
    (concatenate 'string "knowrob:" (convert-to-prolog-str knowrob-action-name))))

(defun get-timestamp-for-logging ()
  (write-to-string (truncate (cram-utilities:current-timestamp))))

(defun get-more-specific-timestamp-for-logging ()
  (format nil "~d"  (truncate (* 1000000 (cram-utilities:current-timestamp)))))

(defun log-action-parameter (designator action-id)
  (mapcar (lambda (key-value-pair)
            (let ((key (first key-value-pair))
                  (value (second key-value-pair)))
             (cond ((eq key :effort)
                    (send-effort-action-parameter
                     action-id
                     (write-to-string value)))
                   ((eq key :position)
                    (send-position-action-parameter action-id value))
                   ((eq key :object)
                    (send-object-action-parameter action-id value))
                   ((eq key :arm)
                    (send-arm-action-parameter action-id value))
                   ((eq key :gripper)
                    (send-gripper-action-parameter action-id value))
                   ((eq key :left-poses)
                    (send-pose-stamped-list-action-parameter action-id "left" value))
                   ((eq key :right-poses)
                    (send-pose-stamped-list-action-parameter action-id "right" value))
                   ((eq key :location)
                    (send-location-action-parameter action-id value))
                   ((eq key :target)
                    (send-target-action-parameter action-id value)))))
          (desig:properties designator)))


(defun log-perform-call (designator)
  (connect-to-cloud-logger)
  (if *is-client-connected*
      (let ((result "")
            (cram-action-name (get-designator-property-value-str designator :TYPE)))
        (setf result (get-value-of-json-prolog-dict
                      (cdaar
                       (send-cram-start-action
                        (get-knowrob-action-name cram-action-name)
                        " \\'DummyContext\\'"
                        (get-timestamp-for-logging)
                        "PV"
                        "ActionInst"))
                      "ActionInst"))
        (log-action-parameter designator result)
        result)
      "NOLOGGING"))

(defun log-cram-finish-action (action-id)
  (send-cram-finish-action
   (convert-to-prolog-str action-id ) (get-timestamp-for-logging)))

(defun log-cram-sub-action (parent-id child-id)
  (if parent-id 
      (progn
        (send-cram-set-subaction
         (convert-to-prolog-str parent-id)
         (convert-to-prolog-str child-id)))))

(defun log-cram-sibling-action (parent-id child-id)
  (let ((hash-value (gethash parent-id *action-siblings*)))
    (if hash-value
      (let ((previous-id (car (cpl:value hash-value))))
          (progn (log-cram-prev-action
                  child-id previous-id)
                 (log-cram-next-action
                  previous-id child-id)
                 (setf (cpl:value hash-value) (cons child-id (cpl:value hash-value)))
                 (setf  (gethash parent-id *action-siblings*) hash-value)))
      (setf (gethash parent-id *action-siblings*) (cpl:make-fluent :name parent-id :value (cons child-id '()))))))

(defun log-cram-prev-action (current-id previous-id)
 ( send-cram-previous-action (convert-to-prolog-str current-id) (convert-to-prolog-str previous-id)))

(defun log-cram-next-action (current-id next-id)
  (send-cram-next-action (convert-to-prolog-str current-id) (convert-to-prolog-str next-id)))

(defmethod exe:generic-perform :around ((designator desig:action-designator))
  (if *is-logging-enabled*
      (let ((action-id (log-perform-call designator)) (is-parent-action nil))
        (cpl:with-failure-handling
            ((cpl:plan-failure (e)
               (log-cram-finish-action action-id)
               (send-task-success action-id "false")
               (format t "failure string: ~a" (write-to-string e))
               (if is-parent-action
                   (send-batch-query))))
          (log-cram-sub-action (car *action-parents*) action-id)
          (log-cram-sibling-action (car *action-parents*) action-id)
          (if (not *action-parents*)
              (setq is-parent-action t))
          (push action-id *action-parents*)
          (let ((perform-result (call-next-method)))
            (log-cram-finish-action action-id)
            (when (and perform-result (typep perform-result 'desig:object-designator))
              (let ((name (desig:desig-prop-value perform-result :name)))
                (when name
                  (send-object-action-parameter action-id perform-result))))
            (send-task-success action-id "true")
            (if is-parent-action
                   (send-batch-query))
            perform-result)))
      (call-next-method)))


(defmethod obj-int:get-object-type-gripping-effort :around (object-type)
  (format t "Asking for EFFORT for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    (format t "EFFORT Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod obj-int:get-object-type-gripper-opening :around (object-type)
  (format t "Asking for GRIPPER OPENING for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    (format t "GRIPPER OPENING Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod obj-int:get-object-type-to-gripper-lift-transform :around (object-type
                                                                      object-name
                                                                      arm
                                                                      grasp
                                                                      grasp-transform)
  (format t "Asking for GRIPPER LIFT TRANSFORMATION for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    (format t "GRIPPER LIFT TRANSFORMATION Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod obj-int:get-object-type-to-gripper-transform :around (object-type
                                                                 object-name
                                                                 arm
                                                                 grasp)
  (format t "Asking for GRIPPER TRANSFORM for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    (format t "GRIPPER TRANSFORM Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod obj-int:get-object-type-to-gripper-pregrasp-transform :around (object-type
                                                                      object-name
                                                                      arm
                                                                      grasp
                                                                      grasp-transform)
  (format t "Asking for GRIPPER PREGRASP TRANSFORMATION for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    (format t "GRIPPER PREGRASP TRANSFORMATION Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod obj-int:get-object-type-to-gripper-2nd-pregrasp-transform :around (object-type
                                                                      object-name
                                                                      arm
                                                                      grasp
                                                                      grasp-transform)
  (format t "Asking for GRIPPER 2ND PREGRASP TRANSFORMATION for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    (format t "GRIPPER 2ND PREGRASP TRANSFORMATION Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))

(defmethod obj-int:get-object-grasping-poses :around (object-name object-type arm grasp object-transform)
  (format t "Asking for GRASPING POSES for the object: ~a~%" object-type)
  (let ((query-result (call-next-method)))
    (format t "Asking for GRASPING POSES Result is ~a~% for the object: ~a~%" query-result object-type)
    query-result))


(defmethod prolog::prove-one :around (query binds &optional rethrow-cut)
  (if *is-logging-enabled*
      (let ((query-id (create-prolog-log-query (car query)))(result (call-next-method)))
        (if query-id 
            (setf (cpl:value *prolog-queries*)
                  (cons (concatenate 'string query-id (get-more-specific-timestamp-for-logging)  '(#\newline))
                        (cpl:value *prolog-queries*))))
        result)
      (call-next-method)))

(defun send-batch-query ()
  (print "Sending batch query ...")
  ;;Remove this if clause when the predicate white-list is done
  (if nil
      (with-open-file (str "PROLOG-CRAM-LOG.csv"
                           :direction :output
                           :if-exists :append
                           :if-does-not-exist :create)
        (format str (create-batch-query))))
  (print "Batch query is done"))

(defun create-prolog-log-query (predicate-name)
  (if (and (string/= (string-downcase (write-to-string predicate-name)) "bound")
           (string/= (string-downcase (write-to-string predicate-name)) "and")
           (string/= (string-downcase (write-to-string predicate-name)) "or")
           (string/= (string-downcase (write-to-string predicate-name)) "cram-prolog:bound")
           (string-downcase (write-to-string predicate-name))
           ) 
      (let ((query-id (concatenate 'string "PrologQuery_" (format nil "~x" (random (expt 16 8))))))
        ;;Use this block again if you will decided to log the predicates in OWL
        ;;(setf (cpl:value *prolog-queries*)
        ;;      (cons (create-rdf-assert-query query-id "rdf:type" " owl:\\'NamedIndividual\\'")
        ;;            (cpl:value *prolog-queries*)))
        ;;(setf (cpl:value *prolog-queries*)
        ;;      (cons (create-rdf-assert-query
        ;;             query-id
        ;;             "knowrob:predicate"
        ;;             (create-string-owl-literal (write-to-string predicate-name)))
        ;;            (cpl:value *prolog-queries*)))
        ;;(setf (cpl:value *prolog-queries*)
        ;;      (cons (create-query
        ;;             "cram_start_action"
        ;;             (list (convert-to-prolog-str "PrologQuery")
        ;;                   "\\'DummyContext\\'"
        ;;                   (get-timestamp-for-logging)
        ;;                   "PV"
        ;;                   query-id))
        ;;            (cpl:value *prolog-queries*)))
        (concatenate 'string (write-to-string predicate-name) ";" (get-more-specific-timestamp-for-logging) ";"))
      nil)
  )


;;Use this version of create-batch-query when you will decide to log the predicates in OWL
;;(defun create-batch-query()
;;  (let ((batch-query ""))
;;    (dolist (item (cdr (cpl:value *prolog-queries*)))
;;      (setq batch-query (concatenate 'string batch-query item)))
;;    (setf (cpl:value *prolog-queries*) '())
;;    batch-query))

(defun create-batch-query()
  (let ((batch-query "") (i 0))
    (loop while (and (cpl:value *prolog-queries*))
          do (setq batch-query
                   (concatenate 'string batch-query
                                (pop (cpl:value *prolog-queries*)))))
    batch-query))

(defun string-start-with (str prefix)
  (if (< (length str) (length prefix))
      nil
      (string= prefix (subseq str 0 (length prefix)))))
