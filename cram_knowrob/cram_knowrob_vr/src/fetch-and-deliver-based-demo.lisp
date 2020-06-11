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

;;; These are not used, as the objects are spawned randomly on the sink
;;; in randomly chosen 'buckets'
(defparameter *object-spawning-poses*
  '((:bowl . ((1.6 0.5 0.87) (0 0 0.4 0.6)))
    (:cup . ((1.3 0.1 0.9) (0 0 -0.7 0.7)))
    (:spoon . ((1.43 0.4 0.85) (0 0 0.3 0.7)))
    ;;     (:breakfast-cereal . ((1.4 0.4 0.85) (0 0 0 1)))
    ;;     (:milk . ((1.4 0.62 0.95) (0 0 1 0)))))
    ))

(defparameter *object-delivering-poses*
  '((bowl . ((-0.7846 1.38127 0.89953) (0.09 0.038 0.995 -0.02)))
    (cup . ((-0.888 1.60885 0.9) (0 0 0.99 0.07213)))
    (spoon . ((-0.7573 1.787 0.86835) (0.0 -0.0 0.999 0.036)))
    ;; (breakfast-cereal . ((1.4 0.4 0.85) (0 0 0 1)))
    ;; (milk . ((1.4 0.62 0.95) (0 0 1 0)))
    ))

(defparameter *object-delivering-poses-varied-kitchen*
  '((bowl . ((-0.68 0.95 0.89953) (0 0 0.1 0.9)))
    (cup . ((-0.85 0.85 0.9) (0 0 0.99 0.07213)))
    (spoon . ((-0.9573 1.0 0.86835) (0.0 -0.0 -0.5 0.5)))))

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
  (let* ((sink-area-y (cl-transforms:y
                       (cl-transforms:origin
                        (btr:pose
                         (btr:rigid-body
                          (btr:get-environment-object)
                          :|ENVIRONMENT.sink_area|)))))
         (object-types '(:cup :bowl :spoon))
         (delta-alpha (* 2 pi))
         (delta-y 0.3)
         (x0 1.35)
         (y0 (- sink-area-y 0.3))
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
                 (type parking-arms)))
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
  (btr:detach-all-objects (btr:get-environment-object))
  (btr-utils:kill-all-objects)
  (setf (btr:joint-state (btr:get-environment-object)
                         "sink_area_left_upper_drawer_main_joint")
        0.0)
  (btr-belief::publish-environment-joint-state
   (btr:joint-states (btr:get-environment-object)))

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

  (unwind-protect
       (dolist (type list-of-objects)

         (experiment-log-start-object-transport type)

         (cpl:with-failure-handling
             ((common-fail:high-level-failure (e)
                (declare (ignore e))
                (experiment-log-finish-object-transport-failed type)
                (return)))

           (let* ((?delivering-poses
                    (list (cl-transforms-stamped:pose->pose-stamped
                           cram-tf:*fixed-frame* 0.0
                           (cram-tf:list->pose
                            (cdr
                             (assoc type
                                    (if (> (cl-transforms:y
                                            (cl-transforms:origin
                                             (btr:pose
                                              (btr:rigid-body
                                               (btr:get-environment-object)
                                               :|ENVIRONMENT.sink_area|))))
                                           1.0)
                                        *object-delivering-poses-varied-kitchen*
                                        *object-delivering-poses*)))))))

                  (?bullet-type
                    (object-type-filter-bullet type)))

             (if *kvr-enabled*

                 (let* ((?search-poses
                          (alexandria:shuffle
                           (cut:force-ll (look-poses-ll-for-searching type))))
                        (?arms
                          (alexandria:shuffle
                           ;; (cut:force-ll (arms-for-fetching-ll type))
                           '(:left :right)))
                        (?grasps
                          (alexandria:shuffle
                           (ecase ?bullet-type
                             (:bowl '(:top))
                             (:cup '(::RIGHT-SIDE :FRONT :LEFT-SIDE :TOP :BACK))
                             (:spoon '(:top))))))
                   ;; (cut:force-ll (object-grasped-faces-ll-from-kvr-type type))

                   (exe:perform
                    (desig:an action
                              (type transporting)
                              (object (desig:an object (type ?bullet-type)))
                              (target (desig:a location (poses ?delivering-poses)))

                              (location (desig:a location (poses ?search-poses)))
                              (arms ?arms)
                              (grasps ?grasps))))

                 (exe:perform
                  (desig:an action
                            (type transporting)
                            (object (desig:an object (type ?bullet-type)))
                            (target (desig:a location (poses ?delivering-poses)))

                            (location (desig:a location
                                               (on (desig:an object
                                                             (type counter-top)
                                                             (urdf-name sink-area-surface)
                                                             (part-of kitchen)))
                                               (side front)))))))

           (experiment-log-finish-object-transport-successful type))

         (experiment-log-current-demo-run-failures *experiment-log-current-object*))

    (experiment-log-finish-demo-run)

    (park-robot)

    (finalize)

    cpl:*current-path*))


