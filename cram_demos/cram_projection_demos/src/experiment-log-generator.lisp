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

(defvar *experiment-log-objects-to-consider* '(:bowl :cup :spoon :breakfast-cereal :milk))
(defvar *experiment-log-current-execution-times* nil)


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



(defun get-experiment-log-failures (failure-symbol &optional object-type)
  (if object-type
      (getf (getf *experiment-log-current-demo-run-object-failures*
                  object-type)
            failure-symbol)
      (reduce #'+
              *experiment-log-objects-to-consider*
              :key (lambda (object-type)
                     (getf (getf *experiment-log-current-demo-run-object-failures*
                                 object-type)
                           failure-symbol)))))

(defun get-experiment-log-transport-duration (&optional object-type)
  (if object-type
      (getf *experiment-log-current-execution-times* object-type)
      (reduce #'+
              *experiment-log-objects-to-consider*
              :key (lambda (object-type)
                     (getf *experiment-log-current-execution-times* object-type)))))


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

  (mapc (lambda (object-type)
          (setf (getf *experiment-log-current-demo-run-object-failures* object-type)
                (let (new-prop-list)
                  (mapc (lambda (element)
                          (setf (getf new-prop-list element) 0))
                        *experiment-log-failures-to-count*)
                  new-prop-list)))
        *experiment-log-objects-to-consider*)

  (mapc (lambda (object-type)
          (setf (getf *experiment-log-current-execution-times* object-type) 0))
        *experiment-log-objects-to-consider*))

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
  (setf (getf *experiment-log-current-execution-times* object-type) (roslisp:ros-time)))

(defun experiment-log-finish-object-transport-successful (object-type)
  (when *experiment-log-detailed?*
    (experiment-log (format nil "TRANSPORTING SUCCEEDED~%")))
  (setf (getf *experiment-log-current-execution-times* object-type)
        (- (roslisp:ros-time)
           (getf *experiment-log-current-execution-times* object-type))))

(defun experiment-log-finish-object-transport-failed (object-type)
  (when *experiment-log-detailed?*
    (experiment-log (format nil "TRANSPORTING FAILED~%")))
  (setf (getf *experiment-log-current-execution-times* object-type) 0))

(defun household-demo-with-logging-one-run (&key (object-list '(:bowl
                                                                :breakfast-cereal
                                                                :milk
                                                                :cup
                                                                :spoon))
                                              varied-kitchen)
  (setf *experiment-log-objects-to-consider* object-list)

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


(defun clearing-demo-with-logging-one-run (&key (object-list '(:bowl
                                                                :breakfast-cereal
                                                                :milk
                                                                :cup
                                                                :spoon))
                                             varied-kitchen)

  (setf *experiment-log-objects-to-consider* object-list)

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
       :spawning-poses-relative *demo-cleaning-object-spawning-poses*))
    (park-robot)

    (dolist (?object-type (reverse object-list))
      (let ((?grasps (cdr (assoc ?object-type *object-grasps*))))

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
                     (context table-cleaning)
                     (grasps ?grasps)))

          (experiment-log-finish-object-transport-successful ?object-type)))

      (experiment-log-current-demo-run-failures *experiment-log-current-object*))

    (experiment-log-finish-demo-run)))



(defun assembly-demo-with-logging-one-run ()

  (setf *experiment-log-objects-to-consider*
        '(:chassis :bottom-wing :underbody :upper-body :bolt1
          :top-wing :bolt2 :window :bolt3 :top-wing2 :propeller :bolt4))

  (when (eq (rob-int:get-robot-name) :tiago-dual)
    (let ((kitchen-island-offset -0.2))
      (setf *plate-z* (+ *plate-z* kitchen-island-offset))
      (btr-belief:vary-kitchen-urdf `(("kitchen_island_footprint_joint"
                                       ((-1.365d0 0.59d0 ,kitchen-island-offset) (0 0 0 1)))))
      (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
      (btr-belief:spawn-world)))

  (urdf-proj:with-projected-robot

    (experiment-log-start-demo-run)

    ;;(setf cram-robosherlock::*no-robosherlock-mode* t)
    (spawn-assembly-objects)
    (let ((old-visibility
            btr:*visibility-threshold*)
          (old-object-position-convergence-delta
            btr-belief::*object-position-convergence-delta*)
          (old-object-rotation-convergence-delta
            btr-belief::*object-rotation-convergence-delta*))
      (setf btr:*visibility-threshold*
            (case (rob-int:get-robot-name)
              (:iai-donbot 0.1) ; perceiving with an object in hand is hard
              (t 0.4)))
      (setf btr-belief::*object-position-convergence-delta* 0.03) ; in meters
      (setf btr-belief::*object-rotation-convergence-delta* 0.1) ; in rad
      (unwind-protect
           (let* ((?env-name
                    (rob-int:get-environment-name))
                  (wooden-plate
                    (desig:an object
                              (type big-wooden-plate)
                              (location (desig:a location
                                                 (on (desig:an object
                                                               (type counter-top)
                                                               (urdf-name
                                                                kitchen-island-surface)
                                                               (part-of ?env-name)))
                                                 ;; (side back)
                                                 (side front)
                                                 (range 0.3))))))
             ;; 1
             (experiment-log-start-object-transport :chassis)
             (transport :chassis '(:side :left) :holder-plane-horizontal '(:range 0.3)
                        :horizontal-attachment
                        wooden-plate)
             (experiment-log-finish-object-transport-successful :chassis)
             (experiment-log-current-demo-run-failures *experiment-log-current-object*)
             ;; 2
             (experiment-log-start-object-transport :bottom-wing)
             (transport :bottom-wing '(:side :right) :chassis '(:range 0.3)
                        :wing-attachment
                        wooden-plate)
             (experiment-log-finish-object-transport-successful :bottom-wing)
             (experiment-log-current-demo-run-failures *experiment-log-current-object*)
             ;; 3
             (experiment-log-start-object-transport :underbody)
             (transport :underbody '(:side :right) :bottom-wing '(:range 0.3)
                        :body-attachment
                        wooden-plate)
             (experiment-log-finish-object-transport-successful :underbody)
             (experiment-log-current-demo-run-failures *experiment-log-current-object*)

             ;; we put the underbody on the bottom-wing but by doing that
             ;; we also put it on the rear-wing.
             ;; as there is no explicit placing action,
             ;; the attachment that we get is loose,
             ;; so we have to attach them manually unfortunately.
             ;; this is required for later moving the whole plane onto another holder
             (btr:attach-object :underbody :rear-wing)

             ;; 4
             (experiment-log-start-object-transport :upper-body)
             (transport :upper-body '(:side :right) :underbody '(:range 0.3)
                        :body-on-body
                        wooden-plate)
             (experiment-log-finish-object-transport-successful :upper-body)
             (experiment-log-current-demo-run-failures *experiment-log-current-object*)
             ;; 5
             (experiment-log-start-object-transport :bolt1)
             (transport :bolt '(:side :right) :upper-body '(:range 0.3)
                        :rear-thread
                        wooden-plate)
             (experiment-log-finish-object-transport-successful :bolt1)
             (experiment-log-current-demo-run-failures *experiment-log-current-object*)
             ;; 6
             (experiment-log-start-object-transport :top-wing)
             (transport :top-wing '(:side :left) :upper-body '(:range 0.3)
                        :wing-attachment
                        wooden-plate)
             (experiment-log-finish-object-transport-successful :top-wing)
             (experiment-log-current-demo-run-failures *experiment-log-current-object*)
             ;; 7
             (experiment-log-start-object-transport :bolt2)
             (transport :bolt :bolt :top-wing '(:range 0.3)
                        :middle-thread
                        wooden-plate)
             (experiment-log-finish-object-transport-successful :bolt2)
             (experiment-log-current-demo-run-failures *experiment-log-current-object*)
             ;; 8
             (experiment-log-start-object-transport :window)
             (transport :window '(:side :left) :top-wing '(:range 0.3)
                        :window-attachment
                        wooden-plate)
             (experiment-log-finish-object-transport-successful :window)
             (experiment-log-current-demo-run-failures *experiment-log-current-object*)
             ;; 9
             (experiment-log-start-object-transport :bolt3)
             (transport :bolt :bolt :window '(:range 0.3)
                        :window-thread
                        wooden-plate)
             (experiment-log-finish-object-transport-successful :bolt3)
             (experiment-log-current-demo-run-failures *experiment-log-current-object*)

             ;; 10
             (experiment-log-start-object-transport :top-wing2)
             (transport :top-wing  '(:range 0.3) :holder-plane-vertical '(:side :left)
                        :vertical-attachment
                        wooden-plate)
             (experiment-log-finish-object-transport-successful :top-wing2)
             (experiment-log-current-demo-run-failures *experiment-log-current-object*)

             ;; 11
             (experiment-log-start-object-transport :propeller)
             (transport :propeller '(:side :left) :motor-grill '(:side :left)
                        :propeller-attachment
                        wooden-plate)
             (experiment-log-finish-object-transport-successful :propeller)
             (experiment-log-current-demo-run-failures *experiment-log-current-object*)

             ;; 12
             (experiment-log-start-object-transport :bolt4)
             (transport :bolt :bolt :propeller '(:side :left)
                        :propeller-thread
                        wooden-plate)
             (experiment-log-finish-object-transport-successful :bolt4)
             (experiment-log-current-demo-run-failures *experiment-log-current-object*))

        (setf *plate-z* *original-plate-z*)
        (setf btr:*visibility-threshold* old-visibility)
        (setf btr-belief::*object-position-convergence-delta*
              old-object-position-convergence-delta)
        (setf btr-belief::*object-rotation-convergence-delta*
              old-object-rotation-convergence-delta)
        (experiment-log-finish-demo-run)))))



(defun retail-demo-with-logging-one-run ()

  (setf *experiment-log-objects-to-consider*
        '(:dish-washer-tabs1 :balea-bottle :dish-washer-tabs2))

  ;; (setf cram-tf:*tf-broadcasting-enabled* t)
  ;; (roslisp-utilities:startup-ros)
  (urdf-proj:with-simulated-robot

    (experiment-log-start-demo-run)

    (if (eql (rob-int:get-robot-name) :kmr-iiwa)
        (setf btr:*visibility-threshold* 0.7)
        (setf btr:*visibility-threshold* 0.5))
    (kill-and-detach-all)
    (let ((?pose (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms-stamped:make-3d-vector 2 0 0.0d0)
                  (cl-transforms:make-quaternion 0 0 1 0))))
      (exe:perform
       (desig:a motion
                (type going)
                (pose ?pose))))
    (if (eql (rob-int:get-environment-name) :store)
        (spawn-objects-on-real-small-shelf)
        (progn
          (spawn-objects-on-small-shelf 0.6)
          (spawn-objects-on-big-shelf 0.6)))
    (unless (member (rob-int:get-robot-name) '(:iai-donbot :kmr-iiwa))
      (spawn-basket))

    (let* ((?source-shelf-base-urdf-name
             (if (eql (rob-int:get-environment-name) :store)
                 :|DMShelfW100_EVZDYXFU|
                 :shelf-2-base))
           (?source-shelf-base-level
             (if (eql (rob-int:get-environment-name) :store)
                 4                      ;3
                 4))
           (?target-shelf-level-urdf-name
             (if (eql (rob-int:get-environment-name) :store)
                 :|DMFloorT6W100_YVLKGJSB| ; :|DMFloorT6W100_KYINFGDM|
                 :shelf-1-level-2-link))
           (?target-shelf-dishwasher-attachments
             (if (eql (rob-int:get-environment-name) :store)
                 '(;; :dish-washer-tabs-real-shelf-1-front
                   :dish-washer-tabs-real-shelf-1-back)
                 '(:dish-washer-tabs-shelf-1-front
                   :dish-washer-tabs-shelf-1-back)))
           (?target-shelf-balea-attachments
             (if (eql (rob-int:get-environment-name) :store)
                 '(;; :balea-bottle-real-shelf-1-front
                   :balea-bottle-real-shelf-1-back)
                 '(:balea-bottle-shelf-1-front
                   :balea-bottle-shelf-1-back)))
           (?environment-name
             (rob-int:get-environment-name))
           (?robot-name
             (rob-int:get-robot-name))
           (?search-location
             (desig:a location
                      (on (desig:an object
                                    (type shelf)
                                    (urdf-name ?source-shelf-base-urdf-name)
                                    (part-of ?environment-name)
                                    (level ?source-shelf-base-level)))
                      (side left)
                      (range 0.2)))

           (?dish-washer-tabs-desig
             (desig:an object
                       (type dish-washer-tabs)
                       (location ?search-location)))
           (?balea-bottle-desig
             (desig:an object
                       (type balea-bottle)
                       (location ?search-location)))
           (?target-location-shelf-dish-washer-tabs
             (desig:a location
                      (on (desig:an object
                                    (type environment)
                                    (name ?environment-name)
                                    (part-of ?environment-name)
                                    (urdf-name ?target-shelf-level-urdf-name)))
                      (for ?dish-washer-tabs-desig)
                      (attachments ?target-shelf-dishwasher-attachments)))
           (?target-location-shelf-balea-bottle
             (desig:a location
                      (on (desig:an object
                                    (type environment)
                                    (name ?environment-name)
                                    (part-of ?environment-name)
                                    (urdf-name ?target-shelf-level-urdf-name)))
                      (for ?balea-bottle-desig)
                      (attachments ?target-shelf-balea-attachments)))
           (?target-location-donbot-tray-dish-washer-tabs
             (desig:a location
                      (on (desig:an object
                                    (type robot)
                                    (name ?robot-name)
                                    (part-of ?robot-name)
                                    (owl-name "donbot_tray")
                                    (urdf-name plate)))
                      (for ?dish-washer-tabs-desig)
                      (attachments (donbot-tray-back donbot-tray-front))))
           (?target-location-kukabot-tray-dish-washer-tabs
             (desig:a location
                      (on (desig:an object
                                    (type robot)
                                    (name ?robot-name)
                                    (part-of ?robot-name)
                                    (owl-name "kukabot_tray")
                                    (urdf-name base-link)))
                      (for ?dish-washer-tabs-desig)
                      (attachments (kukabot-tray-back kukabot-tray-front))))
           (?target-location-basket-dish-washer-tabs
             (desig:a location
                      (on (desig:an object
                                    (type basket)
                                    (name b)))
                      (for ?dish-washer-tabs-desig)
                      (attachments (in-basket-back
                                    in-basket-front
                                    in-basket-other-back
                                    in-basket-other-front))))
           (?target-location-robot-dish-washer-tabs
             (case ?robot-name
               (:iai-donbot
                ?target-location-donbot-tray-dish-washer-tabs)
               (:kmr-iiwa
                ?target-location-kukabot-tray-dish-washer-tabs)
               (t
                ?target-location-basket-dish-washer-tabs))))

      (cpl:with-failure-handling
          ((cpl:simple-plan-failure (e)
             (roslisp:ros-warn (demos retail) "Putting tabs onto Donbot failed: ~a~%~
                                               Ignoring." e)
             (experiment-log-finish-object-transport-failed :dish-washer-tabs1)
             (return)))
        (experiment-log-start-object-transport :dish-washer-tabs1)
        (exe:perform
         (desig:an action
                   (type transporting)
                   (object ?dish-washer-tabs-desig)
                   (target ?target-location-robot-dish-washer-tabs)
                   ;; (grasps (back))
                   ))
        (experiment-log-finish-object-transport-successful :dish-washer-tabs1))
      (experiment-log-current-demo-run-failures *experiment-log-current-object*)
      (cpl:with-failure-handling
          ((cpl:simple-plan-failure (e)
             (roslisp:ros-warn (demos retail) "Transporting balea bottle failed: ~a~%~
                                               Ignoring." e)
             (experiment-log-finish-object-transport-failed :balea-bottle)
             (return)))
        (experiment-log-start-object-transport :balea-bottle)
        (exe:perform
         (desig:an action
                   (type transporting)
                   (object ?balea-bottle-desig)
                   (target ?target-location-shelf-balea-bottle)
                   ;; (grasps (back))
                   ))
        (experiment-log-finish-object-transport-successful :balea-bottle))
      (experiment-log-current-demo-run-failures *experiment-log-current-object*)
      (cpl:with-failure-handling
          ((cpl:simple-plan-failure (e)
             (roslisp:ros-warn (demos retail) "Putting tabs onto shelf failed: ~a~%~
                                               Ignoring." e)
             (experiment-log-finish-object-transport-failed :dish-washer-tabs2)
             (return)))
        (experiment-log-start-object-transport :dish-washer-tabs2)
        (exe:perform
         (desig:an action
                   (type transporting)
                   (object ?dish-washer-tabs-desig)
                   (target ?target-location-shelf-dish-washer-tabs)
                   ;; vvv donbot tries to grasp through itself otherwise
                   (grasps (back))))
        (experiment-log-finish-object-transport-successful :dish-washer-tabs2))
      (experiment-log-current-demo-run-failures *experiment-log-current-object*)

      ;; look at separators
      ;; (exe:perform
      ;;  (desig:an action
      ;;            (type looking)
      ;;            (direction right-separators)))
      ;; (cpl:sleep 5.0)

      (experiment-log-finish-demo-run))))


(defun make-2nd-of-object (object-type)
  (intern (concatenate 'string (symbol-name object-type) "2") :keyword))

(defun storage-demo-with-logging-one-run (&key new-objects)

  (let ((objects-list (if new-objects
                          '(:mug :pot :bowl-round :fork :spatula
                            :denkmit-entkalker :heitmann-citronensaeure :kuehne-essig-essenz
                            :domestos-allzweckreiniger :shoe)
                          '(:dish-washer-tabs :balea-bottle
                            :chassis :front-wheel :propeller
                            :bowl :cup :spoon :milk :breakfast-cereal)))
        stuff-failed)

    (setf *experiment-log-objects-to-consider*
          (concatenate 'list
                       objects-list
                       (mapcar #'make-2nd-of-object objects-list)))

    (when (eq (rob-int:get-robot-name) :tiago-dual)
      (btr-belief:vary-kitchen-urdf
       `(("left_leg_to_storage_origin"
          ((2.0d0 -0.7d0 -0.1d0)
           (0.0d0 0.0d0 0.24740395925452294d0 0.9689124217106447d0)))))
      (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
      (btr-belief:spawn-world))

    (urdf-proj:with-simulated-robot

      (experiment-log-start-demo-run)

      (spawn-storage-objects :objects-list objects-list)
      (let ((old-visibility btr:*visibility-threshold*))
        (setf btr:*visibility-threshold*
              (case (rob-int:get-robot-name)
                ;; perceiving with an object in hand is hard, and perceiving the pot
                ;; even without objects in hand is also too hard
                ((:iai-donbot :kmr-iiwa) 0.4)
                (t 0.7)))
        (unwind-protect

             (let ((?environment-name
                     (rob-int:get-environment-name)))

               ;; bring objects to table
               (dolist (object-type-and-urdf-name
                        (apply
                         (alexandria:curry #'concatenate 'list)
                         (mapcar (lambda (urdf-name-and-stuff)
                                   (mapcar (lambda (type-and-pose)
                                             (cons (car type-and-pose)
                                                   (car urdf-name-and-stuff)))
                                           (cdr urdf-name-and-stuff)))
                                 *storage-poses*)))

                 (let ((?object-type (car object-type-and-urdf-name))
                       (?urdf-name (cdr object-type-and-urdf-name)))
                   (when (member ?object-type objects-list)

                     (cpl:with-failure-handling
                         ((cpl:simple-plan-failure (e)
                            (declare (ignore e))

                            (experiment-log-finish-object-transport-failed ?object-type)

                            (setf stuff-failed t)

                            (return)))

                       (experiment-log-start-object-transport ?object-type)

                       (exe:perform
                        (desig:an action
                                  (type transporting)
                                  (object (desig:an object
                                                    (type ?object-type)
                                                    (location (desig:a location
                                                                       (on (desig:an object
                                                                                     (type shelf)
                                                                                     (urdf-name ?urdf-name)
                                                                                     (part-of ?environment-name)))
                                                                       (side left)))))
                                  (target (desig:a location
                                                   (on (desig:an object
                                                                 (type table)
                                                                 (urdf-name top)
                                                                 (part-of ?environment-name)))
                                                   (for (desig:an object
                                                                  (type ?object-type)))))))

                       (experiment-log-finish-object-transport-successful ?object-type))

                     (experiment-log-current-demo-run-failures *experiment-log-current-object*)

                     (when stuff-failed
                       (return)))))

               ;; put objects back
               (unless stuff-failed
                 (dolist (object-type-and-urdf-name
                          (reverse
                           (apply
                            (alexandria:curry #'concatenate 'list)
                            (mapcar (lambda (urdf-name-and-stuff)
                                      (mapcar (lambda (type-and-pose)
                                                (cons (car type-and-pose)
                                                      (if (string= (car urdf-name-and-stuff)
                                                                   "board0")
                                                          "board2"
                                                          "board0")))
                                              (cdr urdf-name-and-stuff)))
                                    *storage-poses*))))

                   (let* ((?object-type (car object-type-and-urdf-name))
                          (?urdf-name (cdr object-type-and-urdf-name))
                          (logging-type (make-2nd-of-object ?object-type))
                          (?grasps (case ?object-type
                                     (:bowl '(:top-left-tilted :top-right-tilted))
                                     (:dish-washer-tabs '(:back :front))
                                     (:balea-bottle '(:back :front))
                                     ((:shoe :mug) '(:back :front :left-side :right-side))
                                     ;; (:pot '(:left-side :right-side))
                                     (t (cdr (assoc ?object-type *object-grasps*))))))
                     (when (member ?object-type objects-list)

                       (cpl:with-failure-handling
                           ((cpl:simple-plan-failure (e)
                              (declare (ignore e))

                              (experiment-log-finish-object-transport-failed logging-type)

                              (setf stuff-failed t)

                              (return)))

                         (experiment-log-start-object-transport logging-type)

                         (exe:perform
                          (desig:an action
                                    (type transporting)
                                    (object (desig:an object
                                                      (type ?object-type)
                                                      (location (desig:a location
                                                                         (on (desig:an object
                                                                                       (type table)
                                                                                       (urdf-name top)
                                                                                       (part-of ?environment-name)))))))
                                    (target (desig:a location
                                                     (on (desig:an object
                                                                   (type shelf)
                                                                   (urdf-name ?urdf-name)
                                                                   (part-of ?environment-name)))
                                                     (side left)
                                                     (for (desig:an object
                                                                    (type ?object-type)))))
                                    (grasps ?grasps)))

                         (experiment-log-finish-object-transport-successful logging-type))

                       (experiment-log-current-demo-run-failures *experiment-log-current-object*)
                       (when stuff-failed
                         (return)))))))

          (setf btr:*visibility-threshold* old-visibility)

          (experiment-log-finish-demo-run))))))


(defun apartment-demo-with-logging-one-run ()

  (setf *experiment-log-objects-to-consider*
        '(:jeroen-cup1 :jeroen-cup2 :jeroen-cup3 :jeroen-cup4))

  (urdf-proj:with-simulated-robot

    (experiment-log-start-demo-run)

    (setf proj-reasoning::*projection-checks-enabled* t)
    (setf btr:*visibility-threshold* 0.7)

    (initialize-apartment)

    (when (eq (rob-int:get-robot-name) :boxy-description)
      (btr-belief:vary-kitchen-urdf
       `(("frame_B_joint"
          ((3.41d0 3.7d0 0.11d0)
           (0.0d0 0.0d0 1 0)))))
      (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
      (btr-belief:spawn-world))

    (when (eq (rob-int:get-robot-name) :tiago-dual)
      (btr-belief:vary-kitchen-urdf
       `(("frame_B_joint"
          ((2.91d0 3.7d0 0.06d0)
           (0.0d0 0.0d0 1 0)))))
      (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
      (btr-belief:spawn-world))

    (when cram-projection:*projection-environment*
      (spawn-objects-on-fixed-spots
       :object-types '(:jeroen-cup :cup)
       :spawning-poses-relative *apartment-object-spawning-poses*))
    (park-robot (cl-transforms-stamped:make-pose-stamped
                 cram-tf:*fixed-frame*
                 0.0
                 (cl-transforms:make-3d-vector 1.5 1.5 0.0)
                 (cl-transforms:make-quaternion 0 0 0.5 0.5)))

    (cpl:with-failure-handling
        ((cpl:simple-plan-failure (e)
           (roslisp:ros-warn (demos apartment) "some transport failed, aborting~%~a~%" e)
           (experiment-log-finish-object-transport-failed *experiment-log-current-object*)
           (experiment-log-current-demo-run-failures *experiment-log-current-object*)
           (return)))
      (let* ((?object
               (an object
                   (type jeroen-cup)
                   (name jeroen-cup-1)))
             (?location-in-cupboard
               (a location
                  (on (an object
                          (type shelf)
                          (urdf-name cabinet1-coloksu-level4)
                          (part-of apartment)
                          (location (a location
                                       (in (an object
                                               (type cupboard)
                                               (urdf-name cabinet1-door-top-left)
                                               (part-of apartment)))))))
                  (side (back left))
                  (range-invert 0.2)
                  (range 0.25)
                  (orientation upside-down)
                  (for ?object)
                  (attachments (jeroen-cup-on-shelf))))
             (?location-on-island
               (a location
                  (above (an object
                             (type surface)
                             (urdf-name island-countertop)
                             (part-of apartment)))
                  (side back)
                  (range 0.4)
                  (range-invert 0.3)
                  (z-offset 0.07)
                  (for ?object)))
             (?location-in-dishwasher
               (a location
                  (above (an object
                             (type drawer)
                             (urdf-name dishwasher-drawer-middle)
                             (part-of apartment)
                             (location (a location
                                          (in (an object
                                                  (type dishwasher)
                                                  (urdf-name cabinet7)
                                                  (part-of apartment)))))))
                  (for (desig:an object
                                 (type jeroen-cup)
                                 (name jeroen-cup-1)))
                  (attachments (jeroen-cup-in-dishwasher-1 jeroen-cup-in-dishwasher-2))))
             (?location-on-island-upside-down
               (a location
                  (on (an object
                          (type surface)
                          (urdf-name island-countertop)
                          (part-of apartment)))
                  (side (right back))
                  (range 0.5)
                  (orientation upside-down)
                  (for (an object
                           (type jeroen-cup)
                           (name jeroen-cup-1)))))

             (?on-counter-top-cup-pose
               (cl-transforms-stamped:make-pose-stamped
                "map"
                0.0
                (cl-transforms:make-3d-vector 2.43 2.6 1.0426)
                (cl-transforms:make-quaternion 0 0 1 0)))
             (?on-counter-top-cup-upsidedown-pose
               (cl-transforms-stamped:make-pose-stamped
                "map"
                0.0
                (cl-transforms:make-3d-vector 2.43 2.6 1.0426)
                (cl-transforms:make-quaternion 0 1 0 0))))

        (when (eq (rob-int:get-robot-name) :boxy-description)
          (setf ?on-counter-top-cup-pose
                (cram-tf:translate-pose ?on-counter-top-cup-pose :x 0.50))
          (setf ?on-counter-top-cup-upsidedown-pose
                (cram-tf:translate-pose ?on-counter-top-cup-upsidedown-pose :x 0.50)))

        (when (eq (rob-int:get-robot-name) :tiago-dual)
          (setf ?on-counter-top-cup-pose
                (cram-tf:translate-pose ?on-counter-top-cup-pose :z -0.05))
          (setf ?on-counter-top-cup-upsidedown-pose
                (cram-tf:translate-pose ?on-counter-top-cup-upsidedown-pose :z -0.05)))

        ;; bring cup from cupboard to table
        (let ((?goal `(and (cpoe:object-at-location ,(an object
                                                         (type jeroen-cup)
                                                         (name jeroen-cup-1))
                                                    ,(a location
                                                        (pose ?on-counter-top-cup-pose)))
                           (cpoe:location-reset ,?location-in-cupboard))))

          (experiment-log-start-object-transport :jeroen-cup1)

          (exe:perform
           (an action
               (type transporting)
               (object (an object
                           (type jeroen-cup)
                           (name jeroen-cup-1)
                           (location ?location-in-cupboard)))
               ;; (access-search-outer-robot-location (a location
               ;;                                        (poses ?accessing-cupboard-door-robot-poses)))
               (access-seal-search-outer-arms (left))
               (access-search-outer-grasps (back))
               ;; (search-robot-location (a location
               ;;                           (pose ?detecting-cupboard-robot-pose)))
               ;; (fetch-robot-location (a location
               ;;                          (pose ?detecting-cupboard-robot-pose)))
               (arms (left))
               (grasps (front))
               (target (a location
                          (pose ?on-counter-top-cup-pose))
                       ;; ?location-on-island
                       )
               ;; (deliver-robot-location (a location
               ;;                            (pose ?delivering-counter-top-robot-pose)))
               ;; (seal-search-outer-robot-location (a location
               ;;                                      (pose ?sealing-cupboard-door-robot-pose)))
               (seal-search-outer-grasps (back))
               (goal ?goal)))

          (experiment-log-finish-object-transport-successful :jeroen-cup1))
        (experiment-log-current-demo-run-failures *experiment-log-current-object*)

        ;; put cup from island into dishwasher
        (experiment-log-start-object-transport :jeroen-cup2)

        (exe:perform
         (an action
             (type transporting)
             (object (an object
                         (type jeroen-cup)
                         (name jeroen-cup-1)
                         (location ?location-on-island)))
             (target ?location-in-dishwasher)

             ;; (access-deliver-robot-location (a location
             ;;                                   (pose ?accessing-dishwasher-drawer-robot-pose)))
             ;; (seal-deliver-robot-location (a location
             ;;                                 (pose ?sealing-dishwasher-drawer-robot-pose)))
             ;; (access-seal-deliver-arms (left))
             (access-seal-deliver-grasps (back))

             ;; (access-deliver-outer-robot-location (a location
             ;;                                         (pose ?accessing-dishwasher-door-robot-pose)))
             ;; (seal-deliver-outer-robot-location (a location
             ;;                                       (pose ?accessing-dishwasher-door-robot-pose)))
             ;; (access-seal-deliver-outer-arms (left))
             (access-seal-deliver-outer-grasps (back))

             ;; (search-robot-location (a location
             ;;                           (pose ?fetching-counter-top-robot-pose)))
             ;; (fetch-robot-location (a location
             ;;                          (pose ?fetching-counter-top-robot-pose)))
             ;; (deliver-robot-location (a location
             ;;                            (pose ?deliver-dishwasher-robot-pose)))
             ;; (arms (right))
             (grasps (front))))

        (experiment-log-finish-object-transport-successful :jeroen-cup2)
        (experiment-log-current-demo-run-failures *experiment-log-current-object*)

        ;; put cup from dishwasher onto table upside-down
        ;; let ((?goal `(cpoe:object-at-location ,?object ,?location-in-hand)))
        (experiment-log-start-object-transport :jeroen-cup3)

        (exe:perform
         (an action
             (type transporting)
             (object (an object
                         (type jeroen-cup)
                         (name jeroen-cup-1)
                         (location ?location-in-dishwasher)))
             (grasps (bottom))
             ;; (arms (right))

             ;; (deliver-robot-location (a location
             ;;                            (pose ?delivering-counter-top-robot-pose)))
             ;; (target ?location-on-island-upside-down)
             (target (a location
                        (pose ?on-counter-top-cup-upsidedown-pose)))

             ;; (access-search-robot-location (a location
             ;;                                  (pose ?accessing-dishwasher-drawer-robot-pose)))
             ;; (seal-search-robot-location (a location
             ;;                                (pose ?sealing-dishwasher-drawer-robot-pose)))
             ;; (access-seal-search-arms (left))
             (access-seal-search-grasps (back))

             ;; (access-search-outer-robot-location (a location
             ;;                                        (pose ?accessing-dishwasher-door-robot-pose)))
             ;; (seal-search-outer-robot-location (a location
             ;;                                      (pose ?accessing-dishwasher-door-robot-pose)))
             ;; (access-seal-search-outer-arms (left))
             (access-search-outer-grasps (back))
             (seal-search-outer-grasps (back))))

        (experiment-log-finish-object-transport-successful :jeroen-cup3)
        (experiment-log-current-demo-run-failures *experiment-log-current-object*)

        ;; bring cup to cupboard
        (experiment-log-start-object-transport :jeroen-cup4)

        (exe:perform
         (an action
             (type transporting)
             (object (an object
                         (type jeroen-cup)
                         (name jeroen-cup-1)
                         (location ?location-on-island-upside-down)
                         ;; (location ?location-on-island)
                         ))

             ;; (access-deliver-outer-robot-location (a location
             ;;                                         (poses ?accessing-cupboard-door-robot-poses)))
             ;; (seal-deliver-outer-robot-location (a location
             ;;                                       (pose ?sealing-cupboard-door-robot-pose)))
             ;; (access-seal-deliver-outer-arms (left))
             (access-seal-deliver-outer-grasps (back))

             ;; (search-robot-location (a location
             ;;                           (pose ?delivering-counter-top-robot-pose)))
             ;; (fetch-robot-location (a location
             ;;                          (pose ?delivering-counter-top-robot-pose)))
             ;; (arms (left))
             (grasps (front))

             (target ?location-in-cupboard)
             ;; (deliver-robot-location (a location
             ;;                            (pose ?detecting-cupboard-robot-pose)))
             ))

        (experiment-log-finish-object-transport-successful :jeroen-cup4)
        (experiment-log-current-demo-run-failures *experiment-log-current-object*)))

    (finalize)

    (experiment-log-finish-demo-run)))


(defun logging-demo (&key (n 5) (demo :setting))
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

  (dotimes (i n)
    ;; Sleep for a moment to make sure the process modules finished,
    ;; because the demo includes WITH-PROJECTED-ROBOT and that starts process modules.
    (cpl:sleep 3.0)
    (ecase demo
      (:setting (household-demo-with-logging-one-run))
      (:setting-varied (household-demo-with-logging-one-run :varied-kitchen t))
      (:clearing (clearing-demo-with-logging-one-run))
      (:clearing-varied (clearing-demo-with-logging-one-run :varied-kitchen t))
      (:assembling (assembly-demo-with-logging-one-run))
      (:retail (retail-demo-with-logging-one-run))
      (:storage (storage-demo-with-logging-one-run))
      (:storage-mixed (storage-demo-with-logging-one-run :new-objects t))
      (:apartment (apartment-demo-with-logging-one-run)))
    (format t "~%~%~%~%~% RUN FINISHED! ~%~%~%~%~%~%~%")))










;;;;;;;;;;;;;;;;;;;;;;;;;;; SCREENSHOTS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defparameter *bullet-world-camera-poses*
  '((:common
     (:from-back ; this is the identity orientation for the bullet window camera
      (-10 0 2)
      (0 0 0 1))
     #+commented-out
     (cl-transforms:matrix->quaternion
      (make-array '(3 3)
       :initial-contents
       '((1 0 0)
         (0 1 0)
         (0 0 1))))
     (:from-top
      (0 0 10)
      (0.0d0 0.7071067690849304d0 0.0d0 0.7071067690849304d0))
     #+commented-out
     (cl-transforms:matrix->quaternion
      (make-array '(3 3)
       :initial-contents
       '(( 0  0  1)
         ( 0  1  0)
         (-1  0  0)))))

    (:household
     (:household-1-from-top
      (-0.8 0.5 7)
      (0.0d0 0.7071067690849304d0 0.0d0 0.7071067690849304d0))
     (:household-fridge
      (-1.2427089999111585d0 -2.0707653186311266d0 2.7255709456344333d0)
      (-0.1098881594453126d0 0.3376193136478424d0 0.2893334180747689d0 0.8889454337940679d0))
     (:household-back-from-left
      (0.4428486640382607d0 3.255725267750199d0 3.0227247507123556d0)
      (0.2427800187194928d0 0.27562352250424743d0 -0.6147813052831542d0 0.6979494845812823d0))
     (:household-back-from-right
      (0.318532181646342d0 -2.619235885895883d0 3.1404374930742116d0)
      (-0.20445121090688112d0 0.27346435884679615d0 0.5628009132292084d0 0.7527762605738654d0))
     (:household-dining-table-from-right
      (-4.422181090284331d0 -2.22875805799581d0 2.570707780131465d0)
      (-0.11694121144760425d0 0.21222307184094502d0 0.4682286985213598d0 0.849734080006561d0))
     (:household-dining-table-from-left
      (-3.7327180506537d0 2.471284812984886d0 2.633909846654068d0)
      (0.19423304957324591d0 0.2672440257455537d0 -0.5549103510082705d0 0.763497646034821d0))
     (:household-island-from-left
      (0.9451072793857807d0 3.0289612110845314d0 2.4136920252508105d0)
      (0.2538445372316369d0 0.13243053247855438d0 -0.8494829491983876d0 0.4431747104770046d0))
     (:household-island-from-right
      (0.25111390834113817d0 -1.2008875561695385d0 2.5226294997868566d0)
      (0.26038016960056626d0 -0.19364361709310945d0 -0.7590010708041067d0 -0.5644658460493146d0)))

    (:varied-household
     (:from-top
      (-0.8 0.5 7)
      (0.0d0 0.7071067690849304d0 0.0d0 0.7071067690849304d0))
     (:oven-cupboard-from-right
      (-1.776009382338736d0 -1.9071893044769581d0 1.8351254390764025d0)
      (-0.08791219941054991d0 0.18021240272171826d0 0.4295343934429912d0 0.8805084358875885d0))
     (:oven-cupboard-from-left
      (1.9031088096608866d0 3.587146774223393d0 3.6631867225215093d0)
      (0.3348553845685379d0 0.18525246857052524d0 -0.8084124179982971d0 0.4472389992119254d0))
     (:sink-from-right
      (1.6253203939305423d0 -0.48389016694862885d0 3.9735274667116465d0)
      (-0.337249331979391d0 0.24294558387222573d0 0.7379819644792344d0 0.5316229087416765d0))
     (:sink-from-left
      (-1.718450221207771d0 3.0995976331358683d0 3.5760953571340104d0)
      (0.0779887763578891d0 0.3377487306827321d0 -0.2110382981467034d0 0.9139509549140471d0))
     (:dining-table
      (-2.0975788396622272d0 -2.8757867228786704d0 2.9815319066408956d0)
      (-0.22971949880851397d0 0.2301074116408284d0 0.6681207435500727d0 0.6692489585806346d0))
     (:fridge-from-left
      (-3.0397592072738604d0 2.615173330388307d0 3.192392385644361d0)
      (0.07413797799713262d0 0.42867649243017664d0 -0.15344480202197253d0 0.8872399243274285d0))
     (:fridge-from-right
      (1.2259728942150327d0 2.017266205946312d0 3.7072241113921223d0)
      (-0.4312988994038774d0 0.0014553858673927798d0 0.9022027699567103d0 0.0030444333656652945d0))
     (:cup-drawer
      (-4.0292066415691d0 1.0523431828373842d0 3.433302160207545d0)
      (0.002894665403296085d0 0.4901802700232356d0 -0.005147084370784529d0 0.8716010535511527d0)))

    (:assembly
     (:assembly-1-from-top
      (-1.0772886704344242d0 1.590225575491786d0 4.5)
      (0.0d0 0.7071067690849304d0 0.0d0 0.7071067690849304d0))
     (:assembly-from-back-right
      (-2.7386713125000837d0 -0.3189121687976038d0 2.579088527883641d0)
      (0.16722380157626549d0 -0.28945206265482204d0 -0.47146556902338504d0 -0.8160722644800014d0))
     (:assembly-from-back-left
      (-2.2776262578064617d0 3.2469236905768293d0 3.242337784596514d0)
      (-0.2581576762102843d0 -0.39296420374403046d0 0.48458964001546934d0 -0.7376358516618592d0))
     (:assembly-from-front-right
      (-0.02926912309476193d0 0.14161985312638764d0 1.8409605972630283d0)
      (-0.21759808034879735d0 0.09544734134914214d0 0.8895458877134728d0 0.390190997157677d0))
     (:assembly-from-front-left
      (-0.282579528262209d0 3.2635092544060518d0 1.7837899348225619d0)
      (-0.2182911959025824d0 -0.10671608931075438d0 0.8714673359387284d0 -0.42603442657417506d0))
     (:assembly-from-back
      (-3.5247553706076733d0 1.6571616396918318d0 2.264123269142305d0)
      (7.43026050848939d-4 0.29358999795611973d0 -0.002419287921067227d0 0.9559280693804744d0))
     (:assembly-from-front
      (0.33078638112129644d0 1.6052210150081259d0 2.4600495209705233d0)
      (-0.34663541124352165d0 -5.84849146482799d-4 0.9379984065193399d0 -0.0015826054732459489d0))
     (:assembly-from-right
      (-1.5329638425164958d0 -1.04514927681949d0 2.3609445911094564d0)
      (-0.1890215412185938d0 0.19518025622561175d0 0.6695110540753263d0 0.6913251323662517d0))
     (:assembly-from-left
      (-1.566938066022157d0 3.0852072736432854d0 2.1179539182757456d0)
      (-0.26504550207938166d0 -0.2579853596075655d0 0.6657679306531092d0 -0.6480335362050649d0)))

    (:retail
     (:retail-1-from-top
      (1 -2 10)
      (0.5 0.5 -0.5 0.5))
     (:retail-shelf2-from-right
      (-0.13827877599551863d0 -0.14328316251620477d0 2.485410069235185d0)
      (0.05562285521544261d0 0.3316061510536849d0 -0.1557949577842715d0 0.9288010495742004d0))
     (:retail-shelf2-from-left
      (3.6425097268597892d0 0.4416477896164101d0 2.8234127181938544d0)
      (0.32621064119582827d0 0.08672001936499962d0 -0.9097143340592685d0 0.2418389675365802d0))
     (:retail-shelf2-from-back
      (1.743705656516918d0 1.7732643772955823d0 2.5954441970488835d0)
      (0.2095994533817507d0 0.1989136564800995d0 -0.6944119566403303d0 0.6590094543197862d0))
     (:retail-shelf1-from-left
      (1.928939280215045d0 0.5726228143454393d0 2.901527722073049d0)
      (0.1296509234729901d0 0.3915922606671766d0 -0.2863211962676132d0 0.8647926410395883d0))
     (:retail-shelf1-from-right
      (3.4200451938214425d0 -3.0910339797608497d0 1.793782317361786d0)
      (-0.13548428849689068d0 0.14037179672168865d0 0.6811255141231662d0 0.7056966772390644d0))
     (:retail-shelf1-from-back
      (1.077228146495525d0 -0.2809209009675754d0 2.5076855239898492d0)
      (7.430253752629423d-4 0.2935900029671916d0 -0.002419288182018408d0 0.9559280857449026d0)))

    (:apartment
     (:from-top
      (1.4 2.4 6)
      (0.5 -0.5 -0.5 -0.5))
     (:cupboard-from-right
      (3.1273820397301093d0 3.66734729127666d0 2.7388373383359337d0)
      (-0.21053481765476212d0 -0.0862714757399377d0 0.9010563834862576d0 -0.3692285427853724d0))
     (:cupboard-from-left
      (3.9671618361617904d0 0.6464906801758687d0 3.3350838597553776d0)
      (-0.2848280028396511d0 0.05248328032803591d0 0.9412943612980798d0 0.17344578251674983d0))
     (:island-from-front
      (4.593165504061147d0 2.4995573232133137d0 3.4264915070137243d0)
      (0.3479321010076878d0 -5.870365035834359d-4 -0.9375182165695267d0 -0.001581795454659893d0))
     (:dishwasher-from-right
      (1.641998036324047d0 1.042015470355715d0 2.3723833721863046d0)
      (0.17788517938934023d0 -0.18461369732734415d0 -0.6706726246135689d0 -0.6960408582164983d0))
     (:dishwasher-from-left
      (0.1506968651188889d0 4.917878361924585d0 2.458057488176855d0)
      (-0.07899315287587753d0 -0.2772661887236034d0 0.26236326341853483d0 -0.9208957923662574d0))
     (:dishwasher-from-front
      (3.322129065174484d0 3.54089719476555d0 3.6465642550985113d0)
      (-0.5170382182657953d0 -0.005234303838160734d0 0.855902449734962d0 -0.008664840082351032d0)))

    (:storage
     (:from-top
      (0 0 8)
      (0 0.707 0 0.707))
     (:shelf-from-left
      (3.089913073808897d0 1.8064414601807626d0 2.0023901092833665d0)
      (0.20442051824749968d0 0.09360214595258728d0 -0.8859393282757176d0 0.40566291007050237d0))
     (:shelf-from-right
      (-2.976121238394364d0 -1.640470334096036d0 2.1925748258024678d0)
      (-0.022806614586410475d0 0.20874290204497312d0 0.10618911554068781d0 0.9719208460145522d0))
     (:table-from-back
      (-1.5514367991101214d0 -2.3893752587792276d0 3.4969029690950166d0)
      (-0.24677303691133443d0 0.24802528564438114d0 0.660736148481942d0 0.6640890512753949d0))
     (:table-from-side
      (3.185713298243096d0 0.6717307416444517d0 4.029221856999055d0)
      (-0.31333999152779807d0 -0.0029077775500558583d0 0.9495956717286282d0 -0.008812194582716646d0))
     (:table-from-corner
      (-3.2019461846409474d0 2.0504615388182215d0 3.9024824219912637d0)
      (-0.16115806785050535d0 -0.42138876233448247d0 0.31879287022787695d0 -0.8335650508729466d0)))))

(defun camera-pose-name->pose (pose-category pose-name)
  (let ((camera-info (assoc pose-name
                            (rest (assoc pose-category
                                         *bullet-world-camera-poses*)))))
    (cl-transforms-stamped:make-pose
     (apply 'cl-transforms:make-3d-vector (second camera-info))
     (apply 'cl-transforms:make-quaternion (third camera-info)))))

(defun take-screenshot (camera-pose-category camera-pose-name &optional suffix)
  (let* ((bullet-world-camera-pose
           (camera-pose-name->pose camera-pose-category camera-pose-name))
         (opengl-camera-pose
           (cl-transforms:transform->pose
            (transform-bullet-world-camera-into-opengl-camera
             (cl-transforms:pose->transform bullet-world-camera-pose)))))
    (ensure-directories-exist "screenshots/")
    (format nil "~a~a"
            *DEFAULT-PATHNAME-DEFAULTS*
            (btr:png-from-camera-view :robot-camera-view nil
                                      :camera-pose opengl-camera-pose
                                      :png-path (format nil "screenshots/~a-~a-~a.png"
                                                        (roslisp-utils:unix-time)
                                                        camera-pose-name
                                                        (or suffix ""))))))

(defun take-screenshots (&key camera-poses-category (suffix ""))
  (unless camera-poses-category
    (setf camera-poses-category
          (case (rob-int:get-environment-name)
            (:iai-kitchen (if (btr:object btr:*current-bullet-world* :big-wooden-plate)
                              :assembly
                              (if (> (cl-transforms:x
                                      (cl-transforms:origin
                                       (btr:pose
                                        (btr:rigid-body (btr:get-environment-object)
                                                        :|IAI-KITCHEN.fridge_area|))))
                                     0)
                                  :household
                                  :varied-household)))
            (:dm-room :retail)
            (t (rob-int:get-environment-name)))))
  (mapcar (lambda (camera-info-entry)
            (take-screenshot camera-poses-category (car camera-info-entry) suffix))
          (rest (assoc camera-poses-category *bullet-world-camera-poses*))))

(defun get-current-bullet-world-camera-transform ()
  (slot-value btr:*debug-window* 'btr:camera-transform))

(defun test-bullet-world-camera-transform (bullet-world-camera-transform)
  (setf (slot-value btr:*debug-window* 'btr:camera-transform)
        bullet-world-camera-transform))

(defun test-bullet-world-camera-pose-by-name (category name)
  (test-bullet-world-camera-transform
   (cl-transforms:pose->transform
    (camera-pose-name->pose category name))))

(defparameter *bullet-world-camera-R-opengl-camera*
  '(( 0  0  1)
    (-1  0  0)
    ( 0 -1  0)))

(defun transform-bullet-world-camera-into-opengl-camera (bt-camera-transform)
  (cl-transforms:transform*
   bt-camera-transform
   (cl-transforms:make-transform
    (cl-transforms:make-identity-vector)
    (cl-transforms:matrix->quaternion
     (make-array '(3 3)
                 :initial-contents *bullet-world-camera-r-opengl-camera*)))))

(defun transform-opengl-camera-into-bullet-world-camera (opengl-camera-transform)
  (cl-transforms:transform*
   opengl-camera-transform
   (cl-transforms:make-transform
    (cl-transforms:make-identity-vector)
    (cl-transforms:q-inv
     (cl-transforms:matrix->quaternion
      (make-array '(3 3)
                  :initial-contents *bullet-world-camera-r-opengl-camera*))))))

(defun test-opengl-camera-transform (opengl-camera-transform)
  (setf (slot-value btr:*debug-window* 'btr:camera-transform)
        (transform-opengl-camera-into-bullet-world-camera opengl-camera-transform)))

(defparameter *take-screenshots* nil)
(defmethod exe:generic-perform :after ((designator desig:action-designator))
  (when *take-screenshots*
    (let ((action-type (desig:desig-prop-value designator :type))
          ;; (object-type (desig:desig-prop-value
          ;;               (print (desig:desig-prop-value designator :object))
          ;;               :type))
          )
      (when (member action-type '(:gripping :putting))
        (take-screenshots :suffix action-type)))))


(defun unfreeze-bullet-window ()
  (btr:add-debug-window btr:*current-bullet-world*))


(defun start-profiling ()
  (sb-profile:profile ik:call-ik-service  urdf-proj::detect urdf-proj::get-ik-joint-positions)
  (sb-profile:reset))
(defun report-profile ()
  (sb-profile:report))
