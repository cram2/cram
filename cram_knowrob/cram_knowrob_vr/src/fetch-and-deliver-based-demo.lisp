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

(in-package :kvr)

(defparameter *object-spawning-poses*
  '((:bowl . ((1.6 0.5 0.87) (0 0 0.4 0.6)))
    (:cup . ((1.3 0.1 0.9) (0 0 -0.7 0.7)))
    (:spoon . ((1.43 0.4 0.85) (0 0 0.3 0.7)))
    ;; (:breakfast-cereal . ((1.4 0.4 0.85) (0 0 0 1)))
    ;; (:milk . ((1.4 0.62 0.95) (0 0 1 0)))
    ))

(defparameter *object-delivering-poses*
  '((bowl . ((-0.7846 1.38127 0.89953) (0.09 0.038 0.995 -0.02)))
    (cup . ((-0.888 1.60885 0.9) (0 0 0.99 0.07213)))
    (spoon . ((-0.7573 1.787 0.86835) (0.0 -0.0 0.999 0.036)))
    ;; (breakfast-cereal . ((1.4 0.4 0.85) (0 0 0 1)))
    ;; (milk . ((1.4 0.62 0.95) (0 0 1 0)))
    ))

(defparameter *object-delivering-poses-varied-kitchen*
  '((bowl . ((-0.7846 0.3 0.89953) (0 0 0.1 0.9)))
    (cup . ((-0.95 0.2 0.9) (0 0 0.99 0.07213)))
    (spoon . ((-1.0573 0.35 0.86835) (0.0 -0.0 -0.5 0.5)))))

;; (defparameter *object-delivering-poses*
;;   '((breakfast-cereal . ((1.4 0.4 0.85) (0 0 0 1)))
;;     (cup . ((-0.888 1.207885 0.9) (0 0 0.99 0.07213)))
;;     (bowl . ((-0.78 1.07885 0.893) (0 0 0.99 0.07213)))
;;     (spoon . ((-0.7473 1.287 0.86835) (0.0 -0.0 0.995 -0.066)))
;;     (milk . ((1.4 0.62 0.95) (0 0 1 0)))))

(defparameter *object-grasping-arms*
  '(;; (:breakfast-cereal . :right)
    ;; (:cup . :left)
    ;; (:bowl . :right)
    ;; (:spoon . :right)
    ;; (:milk . :right)
    ))

(defparameter *object-cad-models*
  '(;; (:cup . "cup_eco_orange")
    ;; (:bowl . "edeka_red_bowl")
    ))

(defparameter *object-colors*
  '((:spoon . "blue")))

(defun spawn-objects-on-sink-counter ()
  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (btr:detach-all-objects (btr:get-robot-object))
  (let* ((varied-kitchen? (> (cl-transforms:y
                              (cl-transforms:origin
                               (btr:pose
                                (btr:rigid-body
                                 (btr:get-environment-object)
                                 :|KITCHEN.sink_area|))))
                             1.0))
         (object-types '(:cup :bowl :spoon))
         (delta-alpha (* 2 pi))
         (delta-y 0.3)
         (x0 (if varied-kitchen? 1.25 1.35))
         (y0 (if varied-kitchen? 1.0 (- 0.2)))
         (y-bucket-padding 0.2)
         (y-bucket-length 0.3)
         (y-buckets (alexandria:shuffle '(0 1 2))))
    ;; spawn objects at random poses
    (let ((objects (mapcar (lambda (object-type)
                             (let* ((delta-x (ecase object-type
                                               (:spoon 0.2)
                                               (:bowl 0.15)
                                               (:cup 0.1)))
                                    (x (+ x0 (random delta-x)))
                                    (y-bucket (ecase object-type
                                                (:spoon (first y-buckets))
                                                (:bowl (second y-buckets))
                                                (:cup (third y-buckets))))
                                    (y (+ y0
                                          (* (+ y-bucket-padding y-bucket-length) y-bucket)
                                          (random delta-y)))
                                    (z (ecase object-type
                                         (:spoon 0.87)
                                         (:bowl 0.89)
                                         (:cup 0.9)))
                                    (alpha (cl-transforms:normalize-angle (random delta-alpha)))
                                    (pose (cl-transforms:make-pose
                                           (cl-transforms:make-3d-vector x y z)
                                           (cl-transforms:axis-angle->quaternion
                                            (cl-transforms:make-3d-vector 0 0 1)
                                            alpha))))
                               (btr:add-vis-axis-object pose)
                               (btr-utils:spawn-object
                                (intern (format nil "~a" object-type) :keyword)
                                object-type
                                :pose (cram-tf:pose->list pose))))
                           object-types)))
      ;; stabilize world
      (btr:simulate btr:*current-bullet-world* 100)
      objects)))

;; (defmethod exe:generic-perform :before (designator)
;;   (roslisp:ros-info (demo perform) "~%~A~%~%" designator))

(cpl:def-cram-function park-robot ()
  (cpl:with-failure-handling
      ((cpl:plan-failure (e)
         (declare (ignore e))
         (return)))
    (cpl:par
      (exe:perform
       (desig:an action
                 (type positioning-arm)
                 (left-configuration park)
                 (right-configuration park)))
      (let ((?pose (cl-transforms-stamped:make-pose-stamped
                    cram-tf:*fixed-frame*
                    0.0
                    (cl-transforms:make-identity-vector)
                    (cl-transforms:make-identity-rotation))))
        (exe:perform
         (desig:an action
                   (type going)
                   (target (desig:a location
                                    (pose ?pose))))))
      (exe:perform (desig:an action (type opening-gripper) (gripper (left right))))
      (exe:perform (desig:an action (type looking) (direction forward))))))

(defun initialize ()
  (sb-ext:gc :full t)

  ;;(when ccl::*is-logging-enabled*
  ;;    (setf ccl::*is-client-connected* nil)
  ;;    (ccl::connect-to-cloud-logger)
  ;;    (ccl::reset-logged-owl))

  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (setf proj-reasoning::*projection-reasoning-enabled* nil)
  (setf proj-reasoning::*projection-checks-enabled* t)

  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:object btr:*current-bullet-world* :kitchen))
  (btr-utils:kill-all-objects)
  (setf (btr:joint-state (btr:object btr:*current-bullet-world* :kitchen)
                         "sink_area_left_upper_drawer_main_joint")
        0.0)
  (btr-belief::publish-environment-joint-state
   (btr:joint-states (btr:object btr:*current-bullet-world* :kitchen)))

  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))

  (unless cram-projection:*projection-environment*
    (json-prolog:prolog-simple "rdf_retractall(A,B,C,belief_state).")
    ;; (cram-occasions-events:clear-belief) ; to clear giskard environment
    )

  ;; (setf cram-robot-pose-guassian-costmap::*orientation-samples* 3)
  )

(defun finalize ()
  ;; (setf proj-reasoning::*projection-reasoning-enabled* nil)

  ;;(when ccl::*is-logging-enabled*
  ;;  (ccl::export-log-to-owl "ease_milestone_2018.owl")
  ;;  (ccl::export-belief-state-to-owl "ease_milestone_2018_belief.owl"))
  (sb-ext:gc :full t))


;; (defun logger ()
;;   (setf ccl::*is-logging-enabled* t)
;;   (setf ccl::*is-client-connected* nil)
;;   (ccl::connect-to-cloud-logger)
;;   (ccl::reset-logged-owl))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; EXPERIMENT LOG STUFF ;;;;;;;;;;;;;;;;;;;;;

(defparameter *experiment-log-filename* "package://cram_knowrob_vr/experiments/failures")
(defparameter *experiment-log-extension* ".csv")

(defparameter *experiment-log-detailed?* nil)

(defvar *experiment-log-current-object* nil)

(defvar *experiment-log-current-demo-run-vr* -1)
(defvar *experiment-log-current-demo-run-heur* -1)

(defparameter *experiment-log-failures-to-count*
  '(common-fail:searching-failed common-fail:fetching-failed common-fail:delivering-failed
    common-fail:perception-low-level-failure common-fail:navigation-low-level-failure
    common-fail:manipulation-low-level-failure common-fail:ptu-low-level-failure))
(defvar *experiment-log-current-demo-run-object-failures* nil)

(defvar *experiment-log-current-execution-time-bowl* 0)
(defvar *experiment-log-current-execution-time-cup* 0)
(defvar *experiment-log-current-execution-time-spoon* 0)


(defun experiment-log (string &key
                                (object-type *experiment-log-current-object*)
                                (demo-run (if *kvr-enabled*
                                              *experiment-log-current-demo-run-vr*
                                              *experiment-log-current-demo-run-heur*)))
  (let ((file-path
          (physics-utils:parse-uri
           (format nil
                   "~a_~a~a"
                   *experiment-log-filename*
                   (if *kvr-enabled* "VR" "HEUR")
                   *experiment-log-extension*))))
    (unless (probe-file file-path)
      (with-open-file (stream file-path
                              :direction :output
                              :if-exists :error
                              :if-does-not-exist :create)
        (format stream "RUN_ID,OBJ_TYPE,FAIL_TYPE,TRANSPORT_FAIL,~
                        SEARCH_FAIL,FETCH_FAIL,DELIVER_FAIL,~
                        PERCEPT_FAIL,NAV_FAIL,MANIP_FAIL,TIME_SEC~%~%~%")))
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

(defun find-experiment-log-last-current-demo-run (&key vr?)
  (let ((last-line
          (read-last-line
           (format nil "~a_~a~a"
                   *experiment-log-filename*
                   (if vr?
                       "VR"
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
  (if *kvr-enabled*
      (when (< *experiment-log-current-demo-run-vr* 0)
        (setf *experiment-log-current-demo-run-vr*
              (1+ (find-experiment-log-last-current-demo-run :vr? t))))
      (when (< *experiment-log-current-demo-run-heur* 0)
        (setf *experiment-log-current-demo-run-heur*
              (1+ (find-experiment-log-last-current-demo-run :vr? nil))))))

(defmethod cpl:fail :before (&rest args)
  (let ((failure-symbol
          (typecase (first args)
            (symbol (first args))
            (string 'cpl:simple-plan-failure)
            (cpl:plan-failure (type-of (first args)))
            (t 'unknown-failure---------------------))))
    (when *experiment-log-detailed?*
      (experiment-log
       (btr-belief::replace-all
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
  (let ((failure-num
          (if object-type
              (getf (getf *experiment-log-current-demo-run-object-failures*
                          object-type)
                    failure-symbol)
              (+ (getf (getf *experiment-log-current-demo-run-object-failures*
                             'bowl)
                       failure-symbol)
                 (getf (getf *experiment-log-current-demo-run-object-failures*
                             'cup)
                       failure-symbol)
                 (getf (getf *experiment-log-current-demo-run-object-failures*
                             'spoon)
                       failure-symbol)))))
    (if (eql failure-symbol 'common-fail:perception-low-level-failure)
        (/ failure-num 5)
        failure-num)))

(defun get-experiment-log-transport-duration (&optional object-type)
  (if object-type
      (case object-type
        (bowl *experiment-log-current-execution-time-bowl*)
        (cup *experiment-log-current-execution-time-cup*)
        (spoon *experiment-log-current-execution-time-spoon*))
      (+ *experiment-log-current-execution-time-bowl*
         *experiment-log-current-execution-time-cup*
         *experiment-log-current-execution-time-spoon*)))


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
         (navigation-failures
           (get-experiment-log-failures
            'common-fail:navigation-low-level-failure object-type))
         (manipulation-failures
           (get-experiment-log-failures
            'common-fail:manipulation-low-level-failure object-type))
         (perception-failures
           (get-experiment-log-failures
            'common-fail:perception-low-level-failure object-type))
         (duration
           (get-experiment-log-transport-duration object-type)))
    (experiment-log (format nil "SUM,~a,~a,~a,~a,~a,~a,~a,~f"
                            transporting-failed
                            searching-failures fetching-failures delivering-failures
                            perception-failures navigation-failures manipulation-failures
                            duration))
    (when *experiment-log-detailed?*
      (experiment-log (format nil "~%")))))


(defun experiment-log-start-demo-run ()
  (set-experiment-log-current-demo-run)
  (setf *experiment-log-current-object* nil)
  (if *experiment-log-detailed?*
      (experiment-log (format nil "~%~%"))
      (experiment-log (format nil "")))

  (setf (getf *experiment-log-current-demo-run-object-failures* 'bowl)
        (generate-empty-failure-property-list))
  (setf (getf *experiment-log-current-demo-run-object-failures* 'cup)
        (generate-empty-failure-property-list))
  (setf (getf *experiment-log-current-demo-run-object-failures* 'spoon)
        (generate-empty-failure-property-list))

  (setf *experiment-log-current-execution-time-bowl* 0
        *experiment-log-current-execution-time-cup* 0
        *experiment-log-current-execution-time-spoon* 0))

(defun experiment-log-finish-demo-run ()
  (setf *experiment-log-current-object* nil)
  (experiment-log-current-demo-run-failures)
  (if *experiment-log-detailed?*
      (experiment-log (format nil "~%~%"))
      (experiment-log (format nil "")))
  (if *kvr-enabled*
      (incf *experiment-log-current-demo-run-vr*)
      (incf *experiment-log-current-demo-run-heur*)))

(defun experiment-log-start-object-transport (object-type)
  (setf *experiment-log-current-object* object-type)
  (case object-type
    (bowl (setf *experiment-log-current-execution-time-bowl* (roslisp:ros-time)))
    (cup (setf *experiment-log-current-execution-time-cup* (roslisp:ros-time)))
    (spoon (setf *experiment-log-current-execution-time-spoon* (roslisp:ros-time)))))

(defun experiment-log-finish-object-transport-successful (object-type)
  (when *experiment-log-detailed?*
    (experiment-log (format nil "TRANSPORTING SUCCEEDED~%")))
  (case object-type
    (bowl (setf *experiment-log-current-execution-time-bowl*
                (- (roslisp:ros-time) *experiment-log-current-execution-time-bowl*)))
    (cup (setf *experiment-log-current-execution-time-cup*
               (- (roslisp:ros-time) *experiment-log-current-execution-time-cup*)))
    (spoon (setf *experiment-log-current-execution-time-spoon*
                 (- (roslisp:ros-time) *experiment-log-current-execution-time-spoon*)))))

(defun experiment-log-finish-object-transport-failed (object-type)
  (when *experiment-log-detailed?*
    (experiment-log (format nil "TRANSPORTING FAILED~%")))
  (case object-type
    (bowl (setf *experiment-log-current-execution-time-bowl* 0))
    (cup (setf *experiment-log-current-execution-time-cup* 0))
    (spoon (setf *experiment-log-current-execution-time-spoon* 0))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; END OF EXPERIMENT LOG STUFF ;;;;;;;;;;;;;;;


(defun demo (&optional
               (list-of-objects
                '(bowl
                  cup
                  spoon)))

  (experiment-log-start-demo-run)

  (initialize)
  (when cram-projection:*projection-environment*
    (spawn-objects-on-sink-counter))
  (park-robot)

  (dolist (type list-of-objects)

    (experiment-log-start-object-transport type)

    (cpl:with-failure-handling
        ((common-fail:high-level-failure (e)
           (declare (ignore e))
           (experiment-log-finish-object-transport-failed type)
           (return)))

      (if *kvr-enabled*

          (let* ((?bullet-type
                   (object-type-filter-bullet type))
                 (?search-poses
                   (alexandria:shuffle
                    (cut:force-ll (look-poses-ll-for-searching type))))
                 (?grasps
                   (alexandria:shuffle
                    (ecase ?bullet-type
                      (:bowl '(:top))
                      (:cup '(::RIGHT-SIDE :FRONT :LEFT-SIDE :TOP :BACK))
                      (:spoon '(:top)))
                    ;; (cut:force-ll (object-grasped-faces-ll-from-kvr-type type))
                    ))
                 (?arms
                   (alexandria:shuffle
                    '(:left :right) ;; (cut:force-ll (arms-for-fetching-ll type))
                    ))
                 (bowl-pose
                   (cl-transforms-stamped:pose->pose-stamped
                    cram-tf:*fixed-frame* 0.0
                    (cram-tf:list->pose
                     (cdr (assoc 'bowl
                                 (if (> (cl-transforms:y
                                         (cl-transforms:origin
                                          (btr:pose
                                           (btr:rigid-body
                                            (btr:get-environment-object)
                                            :|KITCHEN.sink_area|))))
                                        1.0)
                                     *object-delivering-poses-varied-kitchen*
                                     *object-delivering-poses*))))))
                 (bowl-transform
                   (cram-tf:pose-stamped->transform-stamped bowl-pose "bowl"))
                 (?delivering-poses
                   (case ?bullet-type
                     (:bowl (list bowl-pose))
                     (t
                      (list (cl-transforms-stamped:pose->pose-stamped
                          cram-tf:*fixed-frame* 0.0
                          (cram-tf:list->pose
                           (cdr (assoc type
                                       (if (> (cl-transforms:y
                                               (cl-transforms:origin
                                                (btr:pose
                                                 (btr:rigid-body
                                                  (btr:get-environment-object)
                                                  :|KITCHEN.sink_area|))))
                                              1.0)
                                           *object-delivering-poses-varied-kitchen*
                                           *object-delivering-poses*))))))
                      ;; (alexandria:shuffle
                      ;;  (cut:force-ll (object-poses-ll-for-placing type bowl-transform)))
                      ))))

            (exe:perform
             (desig:an action
                       (type transporting)
                       (object (desig:an object (type ?bullet-type)))
                       (location (desig:a location (poses ?search-poses)))
                       (arms ?arms)
                       (grasps ?grasps)
                       (target (desig:a location (poses ?delivering-poses))))))

          (let ((?bullet-type
                  (object-type-filter-bullet type))
                ;; (?arms
                ;;   (alexandria:shuffle '(:left :right)))
                (?delivering-poses
                   (list (cl-transforms-stamped:pose->pose-stamped
                          cram-tf:*fixed-frame* 0.0
                          (cram-tf:list->pose
                           (cdr (assoc type
                                       (if (> (cl-transforms:y
                                               (cl-transforms:origin
                                                (btr:pose
                                                 (btr:rigid-body
                                                  (btr:get-environment-object)
                                                  :|KITCHEN.sink_area|))))
                                              1.0)
                                           *object-delivering-poses-varied-kitchen*
                                           *object-delivering-poses*))))))))

            (exe:perform
             (desig:an action
                       (type transporting)
                       (object (desig:an object (type ?bullet-type)))
                       (location (desig:a location
                                          (on (desig:an object
                                                        (type counter-top)
                                                        (urdf-name sink-area-surface)
                                                        (part-of kitchen)))
                                          (side front)))
                       (target (desig:a location (poses ?delivering-poses)))))))

      (experiment-log-finish-object-transport-successful type))

    (experiment-log-current-demo-run-failures *experiment-log-current-object*))

  (experiment-log-finish-demo-run)

  (park-robot)

  (finalize)

  cpl:*current-path*)


