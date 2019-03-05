;;;
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
  '((:breakfast-cereal . ((1.4 0.4 0.85) (0 0 0 1)))
    (:cup . ((1.3 0.1 0.9) (0 0 -0.7 0.7)))
    (:bowl . ((1.6 0.5 0.87) (0 0 0.4 0.6)))
    (:spoon . ((1.43 0.4 0.85) (0 0 0.3 0.7)))
    (:milk . ((1.4 0.62 0.95) (0 0 1 0)))))

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

(defun spawn-objects-on-sink-counter (&optional (spawning-poses *object-spawning-poses*))
  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (btr:detach-all-objects (btr:get-robot-object))
  (let ((object-types '(;; :breakfast-cereal
                        :cup
                        :bowl ;; :milk
                        :spoon
                        )))
    ;; spawn objects at default poses
    (let ((objects (mapcar (lambda (object-type)
                             (btr-utils:spawn-object
                              (intern (format nil "~a" object-type) :keyword)
                              object-type
                              :pose (let* ((x (+ 1.5 (- (random 0.4) 0.2)))
                                           (y (+ 0.5 (- (random 1.0) 0.5)))
                                           (pi-number (- (random 2.0) 1.0))
                                           (pi-other (- 1.0 (abs pi-number)))
                                           (pose `((,x ,y 0.87) (0 0 ,pi-number ,pi-other))))
                                      pose)))
                           object-types)))
      ;; stabilize world
      (btr:simulate btr:*current-bullet-world* 100)
      objects)))

(defmethod exe:generic-perform :before (designator)
  (roslisp:ros-info (demo perform) "~%~A~%~%" designator))

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
  (setf pr2-proj-reasoning::*projection-reasoning-enabled* nil)
  (setf pr2-proj-reasoning::*projection-checks-enabled* t)

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
    (btr-belief::call-giskard-environment-service :kill-all "attached")
    (cram-bullet-reasoning-belief-state::call-giskard-environment-service
     :add-kitchen
     "kitchen"
     (cl-transforms-stamped:make-pose-stamped
      "map"
      0.0
      (cl-transforms:make-identity-vector)
      (cl-transforms:make-identity-rotation))))

  ;; (setf cram-robot-pose-guassian-costmap::*orientation-samples* 3)
  )

(defun finalize ()
  ;; (setf pr2-proj-reasoning::*projection-reasoning-enabled* nil)

  ;;(when ccl::*is-logging-enabled*
  ;;  (ccl::export-log-to-owl "ease_milestone_2018.owl")
  ;;  (ccl::export-belief-state-to-owl "ease_milestone_2018_belief.owl"))
  (sb-ext:gc :full t))


;; (defun logger ()
;;   (setf ccl::*is-logging-enabled* t)
;;   (setf ccl::*is-client-connected* nil)
;;   (ccl::connect-to-cloud-logger)
;;   (ccl::reset-logged-owl))

(cpl:def-cram-function demo (&optional
                             (list-of-objects
                              '(bowl
                                spoon
                                cup)))

  (initialize)
  (when cram-projection:*projection-environment*
    (spawn-objects-on-sink-counter))

  (park-robot)

  (dolist (type list-of-objects)
    (cpl:with-failure-handling
        ((common-fail:high-level-failure (e)
           (declare (ignore e))
           (return)))
      (let ((?bullet-type
              (object-type-filter-bullet type))
            (?search-poses
              (alexandria:shuffle (cut:force-ll (look-poses-ll-for-searching type))))
            (?search-base-poses
              (alexandria:shuffle (cut:force-ll (base-poses-ll-for-searching type))))
            (?fetch-base-poses
              (alexandria:shuffle (cut:force-ll (base-poses-ll-for-searching type)))
              ;; (base-poses-ll-for-fetching-based-on-object-desig
              ;;  object-designator)
              )
            (?grasps
              (alexandria:shuffle (cut:force-ll (object-grasped-faces-ll-from-kvr-type type))))
            (?arms
              (alexandria:shuffle '(:left :right) ;; (cut:force-ll (arms-for-fetching-ll type))
                                  ))
            (?delivering-poses
              (alexandria:shuffle (cut:force-ll (object-poses-ll-for-placing type))))
            (?delivering-base-poses
              (alexandria:shuffle (cut:force-ll (base-poses-ll-for-placing type)))))
        (exe:perform
         (desig:an action
                   (type transporting)
                   (object (desig:an object (type ?bullet-type)))
                   (location (desig:a location (poses ?search-poses)))
                   (search-robot-location (desig:a location (poses ?search-base-poses)))
                   (fetch-robot-location (desig:a location (poses ?fetch-base-poses)))
                   (arms ?arms)
                   (grasps ?grasps)
                   (target (desig:a location (poses ?delivering-poses)))
                   (deliver-robot-location (desig:a location (poses ?delivering-base-poses))))))))

  (park-robot)

  (finalize)

  cpl:*current-path*)
