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


(defmethod exe:generic-perform :around ((designator desig:action-designator))
  (if *is-logging-enabled*
      (let ((action-id (log-perform-call designator)))
        (cpl:with-failure-handling
            ((cpl:plan-failure (e)
               (log-cram-finish-action action-id)
               (send-task-success action-id "false")
               (format t "failure string: ~a" (write-to-string e))))
          (let ((perform-result (call-next-method)))
            (log-cram-finish-action action-id)
            (when (and perform-result (typep perform-result 'desig:object-designator))
              (let ((name (desig:desig-prop-value perform-result :name)))
                (when name
                  (send-object-action-parameter action-id perform-result))))
            (send-task-success action-id "true")
            perform-result)))
      (call-next-method)))
