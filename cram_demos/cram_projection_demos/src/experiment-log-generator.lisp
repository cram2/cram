;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :demos)

(defparameter *experiment-log-filename* "package://cram_projection_demos/experiments/log")
(defparameter *experiment-log-extension* ".csv")

(defparameter *experiment-log-detailed?* nil)

(defvar *experiment-log-current-object* nil)

(defvar *experiment-log-current-demo-run-not-heur* -1)
(defvar *experiment-log-current-demo-run-heur* -1)

(defparameter *not-heuristic* nil)

(defparameter *experiment-log-failures-to-count*
  '(common-fail:searching-failed common-fail:fetching-failed common-fail:delivering-failed
    common-fail:navigation-goal-not-reached
    common-fail:manipulation-goal-not-reached
    common-fail:gripper-low-level-failure
    ;; common-fail:perception-low-level-failure common-fail:navigation-low-level-failure
    ;; common-fail:manipulation-low-level-failure common-fail:ptu-low-level-failure
    common-fail:environment-manipulation-goal-not-reached
    common-fail:object-unreachable
    common-fail:object-nowhere-to-be-found
    common-fail:environment-unreachable))
(defvar *experiment-log-current-demo-run-object-failures* nil)

(defvar *experiment-log-current-execution-time-bowl* 0)
(defvar *experiment-log-current-execution-time-cup* 0)
(defvar *experiment-log-current-execution-time-spoon* 0)
(defvar *experiment-log-current-execution-time-breakfast-cereal* 0)
(defvar *experiment-log-current-execution-time-milk* 0)


(defun experiment-log (string &key
                                (object-type *experiment-log-current-object*)
                                (demo-run (if *not-heuristic*
                                              *experiment-log-current-demo-run-not-heur*
                                              *experiment-log-current-demo-run-heur*)))
  (let ((file-path
          (physics-utils:parse-uri
           (format nil
                   "~a_~a~a"
                   *experiment-log-filename*
                   (if *not-heuristic* "NOT_HEUR" "HEUR")
                   *experiment-log-extension*))))
    (unless (probe-file file-path)
      (with-open-file (stream file-path
                              :direction :output
                              :if-exists :error
                              :if-does-not-exist :create)
        (format stream "RUN_ID,OBJ_TYPE,FAIL_TYPE,TRANSPORT_FAIL,~
                        SEARCH_FAIL,FETCH_FAIL,DELIVER_FAIL,~
                        NAV_FAIL,MANIP_FAIL,GRIP_FAIL,ENV_MANIP_FAIL,~
                        REACH_FAIL,FIND_FAIL,ENV_REACH_FAIL,TIME_SEC~%~%~%")))
    (with-open-file (stream file-path
                            :direction :output
                            :if-exists :append
                            :if-does-not-exist :error)
      (format stream "~a,~a,~a~%" demo-run (or object-type "") string))))

(defun read-last-line (file-uri)
  (with-open-file (stream (physics-utils:parse-uri file-uri)
                          :direction :input
                          :if-does-not-exist nil)
    (when stream
      (let (last-non-empty-line)
        (do ((line (read-line stream nil 'eof)
                   (read-line stream nil 'eof)))
            ((eql line 'eof))
          (unless (string-equal
                   (string-trim
                    '(#\Space #\Newline #\Backspace #\Tab #\Linefeed #\Page #\Return #\Rubout)
                    (remove #\Space (remove #\, line)))
                   "")
            (setf last-non-empty-line line)))
        last-non-empty-line))))

(defun find-experiment-log-last-current-demo-run (&key not-heur?)
  (let ((last-line
          (read-last-line
           (format nil "~a_~a~a"
                   *experiment-log-filename*
                   (if not-heur?
                       "NOT_HEUR"
                       "HEUR")
                   *experiment-log-extension*))))
    (if last-line
        (let ((last-current-demo-run
                (read-from-string (first (split-sequence:split-sequence #\, last-line)))))
          (if (or (not (numberp last-current-demo-run)) (< last-current-demo-run 0))
              (error "YOUR LOG FILE IS CORRUPTED!")
              last-current-demo-run))
        -1)))

(defun set-experiment-log-current-demo-run ()
  (if *not-heuristic*
      (when (< *experiment-log-current-demo-run-not-heur* 0)
        (setf *experiment-log-current-demo-run-not-heur*
              (1+ (find-experiment-log-last-current-demo-run :not-heur? t))))
      (when (< *experiment-log-current-demo-run-heur* 0)
        (setf *experiment-log-current-demo-run-heur*
              (1+ (find-experiment-log-last-current-demo-run :not-heur? nil))))))

(defmethod cpl:fail :before (&rest args)
  (let ((failure-symbol
          (typecase (first args)
            (symbol (first args))
            (string 'cpl:simple-plan-failure)
            (cpl:plan-failure (type-of (first args)))
            (t 'unknown-failure---------------------))))
    (when *experiment-log-detailed?*
      (experiment-log
       (cut:replace-all
        (format nil "~a" failure-symbol)
        '(#\Newline) " ")))
    (when (and *experiment-log-current-demo-run-object-failures*
               *experiment-log-current-object*)
      (if (getf *experiment-log-current-demo-run-object-failures*
                *experiment-log-current-object*)
          (mapc (lambda (tracked-failure-symbol)
                  (when (subtypep failure-symbol tracked-failure-symbol)
                    (incf (getf (getf *experiment-log-current-demo-run-object-failures*
                                      *experiment-log-current-object*)
                                tracked-failure-symbol))))
                *experiment-log-failures-to-count*)
          (warn "Object ~a is not tracked for failures!" *experiment-log-current-object*)))))


(defun generate-empty-failure-property-list ()
  (let (new-prop-list)
    (mapc (lambda (failure-symbol)
            (setf (getf new-prop-list failure-symbol) 0))
          *experiment-log-failures-to-count*)
    new-prop-list))

(defun get-experiment-log-failures (failure-symbol &optional object-type)
  (if object-type
      (getf (getf *experiment-log-current-demo-run-object-failures*
                  object-type)
            failure-symbol)
      (+ (getf (getf *experiment-log-current-demo-run-object-failures*
                     :bowl)
               failure-symbol)
         (getf (getf *experiment-log-current-demo-run-object-failures*
                     :cup)
               failure-symbol)
         (getf (getf *experiment-log-current-demo-run-object-failures*
                     :spoon)
               failure-symbol)
         (getf (getf *experiment-log-current-demo-run-object-failures*
                     :breakfast-cereal)
               failure-symbol)
         (getf (getf *experiment-log-current-demo-run-object-failures*
                     :milk)
               failure-symbol))))

(defun get-experiment-log-transport-duration (&optional object-type)
  (if object-type
      (case object-type
        (:bowl *experiment-log-current-execution-time-bowl*)
        (:cup *experiment-log-current-execution-time-cup*)
        (:spoon *experiment-log-current-execution-time-spoon*)
        (:breakfast-cereal *experiment-log-current-execution-time-breakfast-cereal*)
        (:milk *experiment-log-current-execution-time-milk*))
      (+ *experiment-log-current-execution-time-bowl*
         *experiment-log-current-execution-time-cup*
         *experiment-log-current-execution-time-spoon*
         *experiment-log-current-execution-time-breakfast-cereal*
         *experiment-log-current-execution-time-milk*)))


(defun experiment-log-current-demo-run-failures (&optional object-type)
  (let* ((searching-failures
           (get-experiment-log-failures
            'common-fail:searching-failed object-type))
         (fetching-failures
           (get-experiment-log-failures
            'common-fail:fetching-failed object-type))
         (delivering-failures
           (get-experiment-log-failures
            'common-fail:delivering-failed object-type))
         (transporting-failed
           (+ searching-failures fetching-failures delivering-failures))
         (nav-failures
           (get-experiment-log-failures
            'common-fail:navigation-goal-not-reached object-type))
         (manip-failures
           (get-experiment-log-failures
            'common-fail:manipulation-goal-not-reached object-type))
         (grip-failures
           (get-experiment-log-failures
            'common-fail:gripper-low-level-failure object-type))
         (env-manip-failures
           (get-experiment-log-failures
            'common-fail:environment-manipulation-goal-not-reached object-type))
         (obj-reach-failures
           (get-experiment-log-failures
            'common-fail:object-unreachable object-type))
         (obj-find-failures
           (get-experiment-log-failures
            'common-fail:object-nowhere-to-be-found object-type))
         (env-reach-failures
           (get-experiment-log-failures
            'common-fail:environment-unreachable object-type))
         (duration
           (get-experiment-log-transport-duration object-type)))
    (experiment-log (format nil "SUM,~a,~a,~a,~a,~a,~a,~a,~a,~a,~a,~a,~f"
                            transporting-failed
                            searching-failures fetching-failures delivering-failures
                            nav-failures manip-failures grip-failures env-manip-failures
                            obj-reach-failures obj-find-failures env-reach-failures
                            duration))
    (when *experiment-log-detailed?*
      (experiment-log (format nil "~%")))))


(defun experiment-log-start-demo-run ()
  (set-experiment-log-current-demo-run)
  (setf *experiment-log-current-object* nil)
  (if *experiment-log-detailed?*
      (experiment-log (format nil "~%~%"))
      (experiment-log (format nil "")))

  (setf (getf *experiment-log-current-demo-run-object-failures* :bowl)
        (generate-empty-failure-property-list))
  (setf (getf *experiment-log-current-demo-run-object-failures* :cup)
        (generate-empty-failure-property-list))
  (setf (getf *experiment-log-current-demo-run-object-failures* :spoon)
        (generate-empty-failure-property-list))
  (setf (getf *experiment-log-current-demo-run-object-failures* :breakfast-cereal)
        (generate-empty-failure-property-list))
  (setf (getf *experiment-log-current-demo-run-object-failures* :milk)
        (generate-empty-failure-property-list))

  (setf *experiment-log-current-execution-time-bowl* 0
        *experiment-log-current-execution-time-cup* 0
        *experiment-log-current-execution-time-spoon* 0
        *experiment-log-current-execution-time-breakfast-cereal* 0
        *experiment-log-current-execution-time-milk* 0))

(defun experiment-log-finish-demo-run ()
  (setf *experiment-log-current-object* nil)
  (experiment-log-current-demo-run-failures)
  (if *experiment-log-detailed?*
      (experiment-log (format nil "~%~%"))
      (experiment-log (format nil "")))
  (if *not-heuristic*
      (incf *experiment-log-current-demo-run-not-heur*)
      (incf *experiment-log-current-demo-run-heur*)))

(defun experiment-log-start-object-transport (object-type)
  (setf *experiment-log-current-object* object-type)
  (case object-type
    (:bowl (setf *experiment-log-current-execution-time-bowl*
                 (roslisp:ros-time)))
    (:cup (setf *experiment-log-current-execution-time-cup*
                (roslisp:ros-time)))
    (:spoon (setf *experiment-log-current-execution-time-spoon*
                  (roslisp:ros-time)))
    (:breakfast-cereal (setf *experiment-log-current-execution-time-breakfast-cereal*
                             (roslisp:ros-time)))
    (:milk (setf *experiment-log-current-execution-time-milk*
                 (roslisp:ros-time)))))

(defun experiment-log-finish-object-transport-successful (object-type)
  (when *experiment-log-detailed?*
    (experiment-log (format nil "TRANSPORTING SUCCEEDED~%")))
  (case object-type
    (:bowl (setf *experiment-log-current-execution-time-bowl*
                 (- (roslisp:ros-time)
                    *experiment-log-current-execution-time-bowl*)))
    (:cup (setf *experiment-log-current-execution-time-cup*
                (- (roslisp:ros-time)
                   *experiment-log-current-execution-time-cup*)))
    (:spoon (setf *experiment-log-current-execution-time-spoon*
                  (- (roslisp:ros-time)
                     *experiment-log-current-execution-time-spoon*)))
    (:breakfast-cereal (setf *experiment-log-current-execution-time-breakfast-cereal*
                             (- (roslisp:ros-time)
                                *experiment-log-current-execution-time-breakfast-cereal*)))
    (:milk (setf *experiment-log-current-execution-time-milk*
                 (- (roslisp:ros-time)
                    *experiment-log-current-execution-time-milk*)))))

(defun experiment-log-finish-object-transport-failed (object-type)
  (when *experiment-log-detailed?*
    (experiment-log (format nil "TRANSPORTING FAILED~%")))
  (case object-type
    (:bowl (setf *experiment-log-current-execution-time-bowl* 0))
    (:cup (setf *experiment-log-current-execution-time-cup* 0))
    (:spoon (setf *experiment-log-current-execution-time-spoon* 0))
    (:breakfast-cereal (setf *experiment-log-current-execution-time-breakfast-cereal* 0))
    (:milk (setf *experiment-log-current-execution-time-milk* 0))))

(defun household-demo-with-logging-one-run (&key (object-list '(:bowl
                                                                :breakfast-cereal
                                                                :milk
                                                                :cup
                                                                :spoon))
                                              varied-kitchen)
  (urdf-proj:with-simulated-robot

    (setf proj-reasoning::*projection-checks-enabled* T)
    (if varied-kitchen
        (btr-belief:vary-kitchen-urdf *furniture-offsets-offset-kitchen*)
        (btr-belief:vary-kitchen-urdf *furniture-offsets-original-kitchen*))
    (if (> (cl-transforms:x
            (cl-transforms:origin
             (btr:pose
              (btr:rigid-body (btr:get-environment-object)
                              :|IAI-KITCHEN.fridge_area|))))
           0)
        ;; if the fridge is in front of robot, current kitchen is original
        (when varied-kitchen
          (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
          (btr-belief:spawn-world))
        ;; if the fridge is behind the robot, current kitchen is varied
        (unless varied-kitchen
          (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
          (btr-belief:spawn-world)))

    (experiment-log-start-demo-run)

    (initialize)
    (setf btr:*visibility-threshold* 0.7)
    (when cram-projection:*projection-environment*
      (spawn-objects-on-fixed-spots
       :object-types object-list
       :spawning-poses-relative *demo-object-spawning-poses*))
    (park-robot)

    ;; set the table
    (dolist (?object-type object-list)

      (experiment-log-start-object-transport ?object-type)

      (cpl:with-failure-handling
          ((common-fail:high-level-failure (e)
             (declare (ignore e))
             (experiment-log-finish-object-transport-failed ?object-type)
             (return)))

        (exe:perform
         (desig:an action
                   (type transporting)
                   (object (desig:an object (type ?object-type)))
                   (context table-setting)))

        (experiment-log-finish-object-transport-successful ?object-type))

      (experiment-log-current-demo-run-failures *experiment-log-current-object*))

    (experiment-log-finish-demo-run)

    ;; clean up
    ;; (when cram-projection:*projection-environment*
    ;;   (spawn-objects-on-fixed-spots
    ;;    :object-types object-list
    ;;    :spawning-poses-relative *delivery-poses-relative*))
    ;; (dolist (?object-type (reverse object-list))
    ;;   (let ((?grasps (cdr (assoc ?object-type *object-grasps*))))
    ;;     (exe:perform
    ;;      (desig:an action
    ;;                (type transporting)
    ;;                (object (desig:an object (type ?object-type)))
    ;;                (context table-cleaning)
    ;;                (grasps ?grasps)))))
    ))


(defun logging-demo ()
  ;; First, delete or rename the old file to start with a fresh log.
  ;; YOU HAVE TO DO THIS MANUALLY.
  ;; When we start with a fresh log, we should also reset the index of our rows to 0.
  ;; If you do not want to rename or delete the old file and just want to keep logging
  ;; at the bottom of the old file, you don't have to reset the index,
  ;; so comment out the following line.
  (format t "~%~%Do you want to reset index to 0 or keep the last index from the file?")
  (print "(You might have to delete or rename the old log when switching the detail level.)")
  (print "Type 0 to reset index, type -1 to keep the old index:")
  (setf *experiment-log-current-demo-run-heur* (read))
  (format nil "Set the index to ~a.~%" *experiment-log-current-demo-run-heur*)
  (print "Starting demo")

  (dotimes (n 3)
    ;; Sleep for a moment to make sure the process modules finished,
    ;; because the demo includes WITH-PROJECTED-ROBOT and that starts process modules.
    (cpl:sleep 3.0)
    (household-demo-with-logging-one-run :object-list '(:bowl :spoon))
    (format t "~%~%~%~%~% RUN FINISHED! ~%~%~%~%~%~%~%")))
