;;; Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;     * Neither the name of University of Bremen nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :pr2-manipulation-process-module)

(defgeneric call-action (action &rest params))
(defgeneric display-object-handles (object))

(defmethod call-action ((action-sym t) &rest params)
  (ros-info (pr2 manip-pm)
   "Unimplemented operation `~a' with parameters ~a. Doing nothing."
   action-sym params)
  (sleep 0.5))

(defmethod call-action :around (action-sym &rest params)
  (ros-info (pr2 manip-pm)
                    "Executing manipulation action ~a ~a."
                    action-sym params)
  (prog1 (call-next-method)
    (ros-info (pr2 manip-pm) "Manipulation action done.")))

(defmacro def-action-handler (name args &body body)
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
       (destructuring-bind ,args ,params ,@body))))

(defmacro lazy-try-until (variable-name lazy-slot-name lazy-values &body body)
  `(block try-block
     (flet ((success ()
              (return-from try-block t)))
       (loop while (lazy-car ,lazy-values)
             do (let ((,variable-name (var-value ',lazy-slot-name
                                                 (lazy-car ,lazy-values))))
                  ,@body)
                (setf ,lazy-values (lazy-cdr ,lazy-values))))))

(def-action-handler park-object (object grasp-assignments goal-spec)
  (declare (ignore object))
  (ros-info (pr2 manip-pm) "Parking object")
  ;; TODO(winkler): Differentiate here between objects held with one
  ;; arm (simple, default park pose), and multiple arms (keep
  ;; transformation between the grippers as they are all attached to
  ;; the same rigid object somewhere)
  (execute-parks
   (mapcar (lambda (grasp-assignment)
             (make-instance
              'park-parameters
              :arm (side grasp-assignment)
              :max-collisions-tolerance 3
              :park-pose
              (cond ((eql (grasp-type grasp-assignment)
                          :top-slide-down)
                     (ecase (side grasp-assignment)
                       (:left *park-pose-left-top-slide-down*)
                       (:right *park-pose-right-top-slide-down*)))
                    (t
                     (ecase (side grasp-assignment)
                       (:left *park-pose-left-default*)
                       (:right *park-pose-right-default*))))))
           grasp-assignments)
   goal-spec))

(def-action-handler park-arms (arms goal-spec)
  (ros-info (pr2 manip-pm) "Parking free arms: ~a" arms)
  (execute-parks
   (mapcar (lambda (arm)
             (make-instance
              'park-parameters
              :arm arm
              :max-collisions-tolerance 3
              :park-pose
              (ecase arm
                (:left *park-pose-left-default*)
                (:right *park-pose-right-default*))
              :blindly t))
           arms)
   goal-spec))

(def-action-handler park (arms obj goal-spec &optional obstacles)
  (declare (ignore obstacles))
  (let ((arms (force-ll arms)))
    (ros-info (pr2 park) "Park arms ~a" arms)
    (when (> (length arms) 1)
      (let* ((arm-pose-goals (mapcar (lambda (arm)
                                       (let* ((frame-id (link-name arm))
                                              (arm-in-tll (progn
                                                            (tf:wait-for-transform *transformer*
                                                                                   :timeout *tf-default-timeout*
                                                                                   :time 0.0
                                                                                   :source-frame frame-id
                                                                                   :target-frame *robot-torso-frame*)
                                                            (cl-transforms-stamped:transform-pose-stamped *transformer*
                                                                                                          :pose (make-pose-stamped frame-id 0.0
                                                                                                                                   (cl-transforms:make-identity-vector)
                                                                                                                                   (cl-transforms:make-identity-rotation))
                                                                                                          :target-frame *robot-torso-frame*
                                                                                                          :timeout *tf-default-timeout*)))
                                              (raised (copy-pose-stamped arm-in-tll
                                                                         :origin (cl-transforms:v+ (cl-transforms:origin arm-in-tll)
                                                                                                   (cl-transforms:make-3d-vector -0.1 0 0.1)))))
                                         (list arm frame-id raised)))
                                     arms))
             (updated-goal-spec (mot-man:enriched-goal-specification goal-spec
                                                                     :keys `((:quiet t)
                                                                             (:allowed-collision-objects (,(desig-prop-value obj :name))))
                                                                     :arm-pose-goals arm-pose-goals)))
        (mot-man:execute-arm-action updated-goal-spec)))
    (unless (> (length arms) 1)
      (let ((grasp-type (desig-prop-value obj :grasp-type)))
        (cond
          ((and obj arms)
           (let* ((newest-effective (newest-effective-designator obj))
                  (object-name (or (desig-prop-value newest-effective :name)
                                   (desig-prop-value obj :name)))
                  (allowed-collision-objects
                    (append
                     (cond (object-name (list object-name))
                           (t nil))
                     ;; (list "all")
                     )))
             (dolist (arm (force-ll arms))
               (when arm
                 (let ((ignore-collisions nil))
                   (cpl:with-failure-handling
                       ((cram-plan-failures:manipulation-pose-unreachable (f)
                          (declare (ignore f))
                          (roslisp:ros-warn
                           (pr2 manip-pm)
                           "Park failed. Retrying with collisions ignored.")
                          (setf ignore-collisions t)
                          (cpl:retry))
                        (cram-plan-failures:manipulation-failed (f)
                          (declare (ignore f))
                          (roslisp:ros-warn
                           (pr2 manip-pm)
                           "Park failed. Retrying with collisions ignored.")
                          (setf ignore-collisions t)
                          (cpl:retry))
                        (moveit:planning-failed (f)
                          (declare (ignore f))
                          (cpl:fail
                           'cram-plan-failures:manipulation-pose-unreachable)))
                     (let ((carry-pose
                             (ecase arm
                               (:left (cond
                                        ((eql grasp-type :top-slide-down)
                                         (make-pose-stamped
                                          "base_link" (ros-time)
                                          (cl-transforms:make-3d-vector 0.3 0.5 1.3)
                                          (cl-transforms:euler->quaternion
                                           :ax 0 :ay (/ pi -2))))
                                        (t (make-pose-stamped
                                            *robot-torso-frame* (ros-time)
                                            (cl-transforms:make-3d-vector 0.1 0.45 0.3)
                                            (cl-transforms:euler->quaternion :ay (/ pi -2))))))
                               (:right (cond
                                         ((eql grasp-type :top-slide-down)
                                          (make-pose-stamped
                                           "base_link" (ros-time)
                                           (cl-transforms:make-3d-vector 0.3 -0.5 1.3)
                                           (cl-transforms:euler->quaternion
                                            :ax 0 :ay (/ pi -2))))
                                         (t (make-pose-stamped
                                             *robot-torso-frame* (ros-time)
                                             (cl-transforms:make-3d-vector 0.1 -0.45 0.3)
                                             (cl-transforms:euler->quaternion :ay (/ pi -2)))))))))
                       (mot-man:execute-arm-action (mot-man:enriched-goal-specification goal-spec
                                                                                        :keys `((:allowed-collision-objects ,allowed-collision-objects)
                                                                                                (:ignore-collisions ,ignore-collisions))
                                                                                        :arm-pose-goals (list (list arm carry-pose))))))))))))))))

(def-action-handler lift (obj grasp-assignments distance goal-spec)
  (declare (ignore obj))
  (let ((arms (mapcar #'side grasp-assignments)))
    (unless arms
      (error 'simple-error :format-control "No arms for lifting infered."))
    (execute-lift grasp-assignments distance goal-spec)))

(define-hook cram-language::on-handover-transition (side-old side-new object-name))
(define-hook cram-language::on-begin-grasp (obj-desig))
(define-hook cram-language::on-finish-grasp (log-id success))
(define-hook cram-language::on-grasp-decisions-complete
    (log-id grasp-description))

(defun update-action-designator (action-desig new-properties)
  (make-designator :action (update-designator-properties
                            new-properties
                            (description action-desig))
                   action-desig))

(defun min-object-grasp-effort (object)
  (let ((efforts (prolog:prolog `(cram-language::grasp-effort ,object ?effort))))
    (cond (efforts
           (apply
            #'min
            (force-ll
             (lazy-mapcar
              (lambda (bdgs)
                (with-vars-bound (?effort) bdgs
                  ?effort))
              efforts))))
          (t 100))))

(defun perform-grasps (action-desig object assignments-list goal-spec &key log-id)
  (let* ((obj (or (newest-effective-designator object) object))
         (obj-at (desig-prop-value obj :at))
         (obj-pose (when obj-at (reference obj-at)))
         (obj-name (desig-prop-value obj :name)))
    (labels ((calculate-grasp-pose (pose grasp-offset gripper-offset)
               (cl-transforms-stamped:transform-pose-stamped
                *transformer*
                :pose (relative-pose
                       (relative-pose pose grasp-offset)
                       gripper-offset
                       :time 0.0)
                :target-frame *robot-torso-frame*
                :timeout *tf-default-timeout*))
             (grasp-parameters (assignment)
               (let* ((pose (pose assignment))
                      (gripper-offset (gripper-offset assignment)))
                 (make-instance
                  'grasp-parameters
                  :pregrasp-pose (calculate-grasp-pose pose (pregrasp-offset assignment) gripper-offset)
                  :grasp-pose (calculate-grasp-pose pose (grasp-offset assignment) gripper-offset)
                  :grasp-type (grasp-type assignment)
                  :object-part (object-part assignment)
                  :arm (side assignment)
                  :close-radius (or (close-radius assignment) 0.0)
                  :safe-pose (ecase (side assignment)
                               (:left *park-pose-left-default*)
                               (:right *park-pose-right-default*))
                  :effort (min-object-grasp-effort obj)))))
      (let ((params (mapcar #'grasp-parameters assignments-list)))
        (dolist (param-set params)
          (let ((pub (roslisp:advertise "dhdhdh" "geometry_msgs/PoseStamped")))
            (roslisp:publish pub (to-msg (pregrasp-pose param-set))))
          (when (and obj-pose action-desig)
            (cram-language::on-grasp-decisions-complete
             log-id `(,@(mapcar (lambda (param-set)
                                  `(:grasp ((:arm ,(arm param-set))
                                            (:effort ,(effort param-set))
                                            (:object-name ,obj-name)
                                            (:object-pose
                                             ,(cl-transforms-stamped:transform-pose-stamped
                                               *transformer*
                                               :pose obj-pose
                                               :target-frame (frame-id (grasp-pose param-set))
                                               :timeout *tf-default-timeout*))
                                            (:grasp-type ,(grasp-type param-set))
                                            (:pregrasp-pose ,(pregrasp-pose param-set))
                                            (:grasp-pose ,(grasp-pose param-set)))))
                                params)))))
        (when action-desig
          (update-action-designator
           action-desig `(,@(mapcar (lambda (param-set)
                                      `(:grasp ((:arm ,(arm param-set))
                                                (:effort ,(effort param-set))
                                                (:object-pose
                                                 ,(cl-transforms-stamped:transform-pose-stamped
                                                   *transformer*
                                                   :pose obj-pose
                                                   :target-frame (frame-id (grasp-pose param-set))
                                                   :timeout *tf-default-timeout*))
                                                (:grasp-type ,(grasp-type param-set))
                                                (:pregrasp-pose ,(pregrasp-pose param-set))
                                                (:grasp-pose ,(grasp-pose param-set)))))
                                    params))))
        (execute-grasps obj-name params goal-spec)
        (dolist (param-set params)
          (with-vars-strictly-bound (?link-name)
              (lazy-car
               (prolog
                `(and (robot ?robot)
                      (cram-robot-interfaces:end-effector-link
                       ?robot ,(arm param-set) ?link-name))))
            (cram-occasions-events:on-event
             (make-instance 'cram-plan-occasions-events:object-attached
                            :object obj
                            :link ?link-name
                            :side (arm param-set)))))
        (when action-desig
          (let ((at (desig-prop-value (current-desig obj) :at)))
            (make-designator
             :location
             (append (description at)
                     (mapcar (lambda (param-set)
                               `((:handle ,(vector
                                            (arm param-set)
                                            (object-part param-set)))))
                             params))
             at)))))))

(defun grasp-ex (action-desig object goal-spec)
  (display-object-handles object)
  (let ((grasp-assignments (prolog:prolog `(grasp-assignments ,object ?grasp-assignments))))
    (unless
        (block object-lost-catch
          (cpl:with-failure-handling
              ((cram-plan-failures:object-lost (f)
                 (declare (ignore f))
                 (ros-warn (pr2 manip-pm) "Lost object. Canceling grasp.")
                 (return-from object-lost-catch)))
            (lazy-try-until assignments-list ?grasp-assignments grasp-assignments
              (block next-assignment-list
                (cpl:with-failure-handling
                    ((cram-plan-failures:manipulation-pose-unreachable (f)
                       (declare (ignore f))
                       (ros-warn (pr2 manip-pm) "Try next grasp assignment")
                       (return-from next-assignment-list)))
                  (let ((log-id (first (cram-language::on-begin-grasp object)))
                        (success nil))
                    (unwind-protect
                         (progn
                           (ros-info (pr2 manip-pm) "Performing grasp assignment(s):~%")
                           (dolist (assignment assignments-list)
                             (ros-info (pr2 manip-pm) " - ~a/~a"
                                       (grasp-type assignment)
                                       (side assignment)))
                           (perform-grasps action-desig object assignments-list goal-spec :log-id log-id)
                           (ros-info (pr2 manip-pm) "Successful grasp")
                           (setf success t)
                           (success))
                      (cram-language::on-finish-grasp log-id success))))))))
      (cpl:fail 'manipulation-pose-unreachable))))

(def-action-handler grasp (action-desig object goal-spec)
  "Handles the grasping of any given `object'. Calculates proper grasping poses for the object, based on physical gripper characteristics, free grippers, object grasp points (handles), grasp type for this object, and position of the object relative to the robot's grippers. `action-desig' is the action designator instance that triggered this handler's execution, and is later updated with more precise grasping information based on the actual infered action."
  (grasp-ex action-desig object goal-spec))

(defun pose-pointing-away-from-base (object-pose)
  (let ((ref-frame "base_link")
        (fin-frame "map"))
    (let* ((base-transform-map (cl-transforms-stamped:lookup-transform
                                *transformer* fin-frame ref-frame
                                :timeout *tf-default-timeout*))
           (base-pose-map (make-pose-stamped
                           (frame-id base-transform-map)
                           (stamp base-transform-map)
                           (cl-transforms:translation base-transform-map)
                           (cl-transforms:rotation base-transform-map)))
           (object-pose-map (cl-transforms-stamped:transform-pose-stamped
                             *transformer*
                             :pose object-pose
                             :target-frame fin-frame
                             :timeout *tf-default-timeout*))
           (origin1 (cl-transforms:origin base-pose-map))
           (origin2 (cl-transforms:origin object-pose-map))
           (p1 (cl-transforms:make-3d-vector (cl-transforms:x origin1)
                                             (cl-transforms:y origin1) 0.0))
           (p2 (cl-transforms:make-3d-vector (cl-transforms:x origin2)
                                             (cl-transforms:y origin2) 0.0))
           (angle (+ (* (signum (- (cl-transforms:y p2) (cl-transforms:y p1)))
                        (acos (/ (- (cl-transforms:x p2)
                                    (cl-transforms:x p1))
                                 (cl-transforms:v-dist p1 p2))))
                     (/ pi -2))))
      (make-pose-stamped fin-frame 0.0
                                         (cl-transforms:origin object-pose-map)
                                         (cl-transforms:euler->quaternion
                                          :az (+ angle (/ pi 2)))))))

(define-hook cram-language::on-begin-putdown (obj-desig loc-desig))
(define-hook cram-language::on-finish-putdown (log-id success))

(defun make-putdown-pose (putdown-location &key (z-offset 0.0))
  (let* ((putdown-pose (tf:copy-pose-stamped
                        (or (desig-prop-value putdown-location 'pose)
                            (pose-pointing-away-from-base
                             (reference putdown-location)))
                        :stamp 0.0))
         (pose-in-tll
           (progn
             (tf:wait-for-transform
              *transformer*
              :time (tf:stamp putdown-pose)
              :source-frame (tf:frame-id putdown-pose)
              :target-frame *robot-torso-frame*)
             (cl-transforms-stamped:transform-pose-stamped
              *transformer*
              :pose putdown-pose
              :target-frame *robot-torso-frame*
              :timeout *tf-default-timeout*))))
    (copy-pose-stamped
     pose-in-tll :origin (cl-transforms:v+ (cl-transforms:origin pose-in-tll)
                                           (cl-transforms:make-3d-vector 0.0 0.0 z-offset)))))

(define-hook cram-language::on-put-down-reorientation-count (object-designator))

(defun hand-poses-for-putdown (grasp-assignment putdown-pose)
  (let* ((grasp-type (grasp-type grasp-assignment))
         ;; TODO(winkler): Adapt this `pre-putdown-pose' to the
         ;; grasp-type
         (pre-putdown-offset *pre-putdown-offset*)
         (putdown-offset *putdown-offset*)
         (unhand-offset (cond ((eql grasp-type :top-slide-down)
                               *unhand-top-slide-down-offset*)
                              (t *unhand-offset*))))
    (labels ((gripper-putdown-pose (object-in-gripper-pose object-putdown-pose)
               (pose->pose-stamped
                (frame-id object-putdown-pose) 0.0
                (cl-transforms:transform->pose
                 (cl-transforms:transform*
                  (cl-transforms:pose->transform object-putdown-pose)
                  (cl-transforms:transform-inv
                   (cl-transforms:pose->transform object-in-gripper-pose))))))
             (gripper-grasp-pose (grasp-assignment pose-offset object-putdown-pose)
               (relative-pose
                (gripper-putdown-pose
                 (slot-value grasp-assignment 'pose)
                 object-putdown-pose)
                pose-offset))
             (grasp-assignment->pre-putdown-pose (grasp-assignment object-putdown-pose)
               (gripper-grasp-pose grasp-assignment pre-putdown-offset object-putdown-pose))
             (grasp-assignment->putdown-pose (grasp-assignment object-putdown-pose)
               (gripper-grasp-pose grasp-assignment putdown-offset object-putdown-pose))
             (grasp-assignment->unhand-pose (grasp-assignment object-putdown-pose)
               (gripper-grasp-pose grasp-assignment unhand-offset object-putdown-pose)))
      (let* ((side (slot-value grasp-assignment 'side))
             (pre-putdown-pose (grasp-assignment->pre-putdown-pose
                                grasp-assignment putdown-pose))
             (putdown-hand-pose (grasp-assignment->putdown-pose
                                 grasp-assignment putdown-pose))
             (unhand-pose (grasp-assignment->unhand-pose
                           grasp-assignment putdown-pose)))
        (publish-pose putdown-hand-pose "putdownhandpose")
        (publish-pose pre-putdown-pose "preputdownpose")
        (publish-pose unhand-pose "unhandpose")
        (unless (mot-man:all-ok 
                  (mot-man:execute-arm-action
                    (mot-man:make-goal-specification :ik-check-goal-specification
                                                     :arm-pose-goals (list (list side `(,pre-putdown-pose ,putdown-hand-pose ,unhand-pose))))))
          (cpl:fail 'manipulation-failure))
        (make-instance
         'putdown-parameters
         :grasp-type grasp-type
         :arm side
         :pre-putdown-pose pre-putdown-pose
         :putdown-pose putdown-hand-pose
         :unhand-pose unhand-pose)))))

(defun object-pose->hand-pose (grasp-assignment object-pose)
  (labels ((gripper-putdown-pose (object-in-gripper-pose object-putdown-pose)
             (pose->pose-stamped
              (frame-id object-putdown-pose) 0.0
              (transform->pose
               (transform*
                (pose->transform object-putdown-pose)
                (transform-inv
                 (pose->transform object-in-gripper-pose))))))
           (gripper-grasp-pose (grasp-assignment pose-offset object-putdown-pose)
             (relative-pose
              (gripper-putdown-pose
               (slot-value grasp-assignment 'pose)
               object-putdown-pose)
              pose-offset))
           (grasp-assignment->pose (grasp-assignment object-pose)
             (gripper-grasp-pose grasp-assignment (make-identity-pose) object-pose)))
    (let* ((side (slot-value grasp-assignment 'side))
           (pose (grasp-assignment->pose grasp-assignment object-pose)))
      (publish-pose pose "handpose")
      (unless (mot-man:all-ok 
                (mot-man:execute-arm-action
                  (mot-man:make-goal-specification :ik-check-goal-specification
                                                   :arm-pose-goals (list (list side `(,pose))))))
        (cpl:fail 'manipulation-failure))
      pose)))

(defun perform-putdowns (object-designator grasp-assignments putdown-pose goal-spec)
  (let ((putdown-parameter-sets
          (mapcar (lambda (grasp-assignment)
                    (hand-poses-for-putdown
                     grasp-assignment putdown-pose))
                  grasp-assignments)))
    (execute-putdowns (desig-prop-value object-designator :name)
                      putdown-parameter-sets
                      goal-spec)))

(def-action-handler handover (object grasp-assignments goal-spec)
  (let* ((grasp-assignment (first grasp-assignments))
         (target-side (case (side grasp-assignment)
                        (:left :right)
                        (:right :left)))
         (target-pose (make-pose-stamped
                         *robot-torso-frame* 0.0
                         (make-3d-vector 0.4 0.0 0.2)
                         (case (side grasp-assignment)
                           (:left (euler->quaternion :az (/ pi -2)))
                           (:right (euler->quaternion :az (/ pi 2))))))
           (hand-pose (object-pose->hand-pose grasp-assignment target-pose)))
      (block motion-block
        (cpl:with-failure-handling ((moveit:control-failed (f)
                                      (declare (ignore f))
                                      (return-from motion-block)))
          (mot-man:execute-arm-action (mot-man:enriched-goal-specification goal-spec
                                                                           :arm-pose-goals (list (list (side grasp-assignment) hand-pose))))))
      (let ((old-allowed-arms *allowed-arms*))
        (setf *allowed-arms* `(,target-side))
        (grasp-ex (make-designator :action nil) object goal-spec)
        (setf *allowed-arms* old-allowed-arms))
      (open-gripper (side grasp-assignment))
      (cram-language::on-handover-transition
       (side grasp-assignment) target-side (desig-prop-value object :name))
      (sleep 5)
      (mot-man:execute-arm-action (mot-man:enriched-goal-specification goal-spec
                                                                       :arm-pose-goals (list (list target-side
                                                                                                   (make-pose-stamped
                                                                                                     (link-name target-side)
                                                                                                     0.0
                                                                                                     (make-3d-vector -0.05 0.0 0.0)
                                                                                                     (euler->quaternion))))))
      (moveit:detach-collision-object-from-link
       (desig-prop-value object :name)
       (link-name (side grasp-assignment))
       :current-pose-stamped target-pose)
      (mot-man:execute-arm-action (mot-man:enriched-goal-specification goal-spec
                                                                       :arm-pose-goals (list (list (side grasp-assignment)
                                                                                                   (make-pose-stamped
                                                                                                     (link-name (side grasp-assignment))
                                                                                                     0.0
                                                                                                     (make-3d-vector -0.05 0.0 0.0)
                                                                                                     (euler->quaternion))))))))

(def-action-handler put-down (object-designator location grasp-assignments goal-spec)
  (unless (and object-designator location)
    (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable))
  (assert (> (length grasp-assignments) 0) ()
          "No arm/pose pairs specified during put-down.")
  (let* ((log-id (first (cram-language::on-begin-putdown object-designator location)))
         (success nil)
         (putdown-pose-pure (make-putdown-pose
                             location
                             :z-offset (or (when (desig-prop-value
                                                  object-designator
                                                  :plane-distance)
                                             (+ (desig-prop-value
                                                 object-designator
                                                 :plane-distance)
                                                0.02)) ;; Add two centimeters for good measure
                                           (when (desig-prop-value
                                                  object-designator
                                                  :dimensions)
                                             ;;(+ (/ (tf:z (desig-prop-value
                                             ;;             object-designator
                                             ;;             :dimensions))
                                             ;;      2)
                                                0.02);;)
                                           0.1)))
         (lazy-putdown-poses
           (prolog:prolog
            `(putdown-pose
              ,object-designator
              ,putdown-pose-pure
              ,(first (cram-language::on-put-down-reorientation-count
                       object-designator))
              ?putdown-pose))))
    (unwind-protect
         (unless (lazy-try-until putdown-pose ?putdown-pose lazy-putdown-poses
                   (block next-putdown-pose
                     (cpl:with-failure-handling
                         ((manipulation-failure (f)
                            (declare (ignore f))
                            (ros-info (pr2 manip-pm) "Trying next putdown-pose.")
                            (return-from next-putdown-pose)))
                       (publish-pose putdown-pose "putdownpose")
                       (perform-putdowns object-designator grasp-assignments putdown-pose goal-spec)
                       (setf success t)
                       (success))))
           (cpl:fail 'manipulation-failure)
           (dolist (grasp-assignment grasp-assignments)
             (let ((side (side grasp-assignment))
                   (grasped-object (or (car (handle-pair grasp-assignment))
                                       object-designator)))
               (with-vars-strictly-bound (?link-name)
                   (lazy-car
                    (prolog
                     `(and (robot ?robot)
                           (cram-robot-interfaces:end-effector-link
                            ?robot ,side ?link-name))))
                 (cram-occasions-events:on-event
                  (make-instance
                      'cram-plan-occasions-events:object-detached
                    :object grasped-object
                    :link ?link-name
                    :side side))))))
      (cram-language::on-finish-putdown log-id success))))

(defmethod display-object-handles ((object object-designator))
  (let* ((relative-handles (desig-prop-values object :handle))
         (reorient-object
           (var-value '?r (first (prolog:prolog `(reorient-object-globally ,object ?r)))))
         (absolute-handles
           (mapcar (lambda (handle)
                     (absolute-handle object handle :reorient reorient-object))
                   relative-handles))
         (pose-msgs
           (map 'vector
                (lambda (handle)
                  (let ((pose (reference (desig-prop-value handle :at))))
                    (to-msg (tf:pose-stamped->pose pose))))
                absolute-handles)))
    (let ((publisher (roslisp:advertise "objecthandleposes" "geometry_msgs/PoseArray")))
      (roslisp:publish publisher
                       (roslisp:make-message
                        "geometry_msgs/PoseArray"
                        (frame_id header) (frame-id
                                           (reference (desig-prop-value
                                                       (first absolute-handles) :at)))
                        (poses) pose-msgs)))))
