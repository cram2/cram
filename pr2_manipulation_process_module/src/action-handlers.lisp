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

(def-action-handler park-object (object grasp-assignments)
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
                          'desig-props:top-slide-down)
                     (ecase (side grasp-assignment)
                       (:left *park-pose-left-top-slide-down*)
                       (:right *park-pose-right-top-slide-down*)))
                    (t
                     (ecase (side grasp-assignment)
                       (:left *park-pose-left-default*)
                       (:right *park-pose-right-default*))))))
           grasp-assignments)))

(def-action-handler park-arms (arms)
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
                (:right *park-pose-right-default*))))
           arms)))

(def-action-handler park (arms obj &optional obstacles)
  (declare (ignore obstacles))
  (let ((arms (force-ll arms)))
    (ros-info (pr2 park) "Park arms ~a" arms)
    (when (> (length arms) 1)
      (let ((trajectories
              (mapcar (lambda (arm)
                        (let* ((frame-id
                                 (ecase arm
                                   (:left "l_wrist_roll_link")
                                   (:right "r_wrist_roll_link")))
                               (arm-in-tll
                                 (cl-tf2:transform-pose
                                  *tf2-buffer*
                                  :pose (cl-tf-datatypes:make-pose-stamped
                                         frame-id (ros-time)
                                         (cl-transforms:make-identity-vector)
                                         (cl-transforms:make-identity-rotation))
                                  :target-frame "/torso_lift_link"
                                  :timeout cram-roslisp-common:*tf-default-timeout*))
                               (raised
                                 (cl-tf-datatypes:copy-pose-stamped
                                  arm-in-tll
                                  :origin
                                  (cl-transforms:v+
                                   (cl-transforms:origin arm-in-tll)
                                   (cl-transforms:make-3d-vector -0.1 0 0.1)))))
                          (execute-move-arm-pose
                           arm raised :plan-only t
                                      :quiet t
                                      :allowed-collision-objects
                                      `(,(desig-prop-value obj 'desig-props::name)))))
                      arms)))
        (moveit::execute-trajectories trajectories)))
    (unless (> (length arms) 1)
      (let ((grasp-type (desig-prop-value obj 'desig-props:grasp-type)))
        (cond
          ((and obj arms)
           (let* ((newest-effective (newest-effective-designator obj))
                  (object-name (desig-prop-value newest-effective
                                                 'desig-props:name))
                  (allowed-collision-objects
                    (append
                     (cond (object-name (list object-name))
                           (t nil))
                     (list "all"))))
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
                                        ((eql grasp-type
                                              'desig-props:top-slide-down)
                                         (cl-tf-datatypes:make-pose-stamped
                                          "base_link" (ros-time)
                                          (cl-transforms:make-3d-vector 0.3 0.5 1.3)
                                          (cl-transforms:euler->quaternion
                                           :ax 0 :ay (/ pi -2))))
                                        (t (cl-tf-datatypes:make-pose-stamped
                                            "base_link" (ros-time)
                                            (cl-transforms:make-3d-vector 0.3 0.5 1.3)
                                            (cl-transforms:euler->quaternion :ax 0)))))
                               (:right (cond
                                         ((eql grasp-type
                                               'desig-props:top-slide-down)
                                          (cl-tf-datatypes:make-pose-stamped
                                           "base_link" (ros-time)
                                           (cl-transforms:make-3d-vector 0.3 -0.5 1.3)
                                           (cl-transforms:euler->quaternion
                                            :ax 0 :ay (/ pi -2))))
                                         (t (cl-tf-datatypes:make-pose-stamped
                                             "base_link" (ros-time)
                                             (cl-transforms:make-3d-vector 0.3 -0.5 1.3)
                                             (cl-transforms:euler->quaternion :ax 0))))))))
                       (execute-move-arm-pose
                        arm carry-pose
                        :allowed-collision-objects allowed-collision-objects
                        :ignore-collisions ignore-collisions)))))))))))))

(def-action-handler lift (obj grasp-assignments distance)
  (declare (ignore obj))
  (let ((arms (mapcar #'side grasp-assignments)))
    (unless arms
      (error 'simple-error :format-control "No arms for lifting infered."))
    (execute-lift grasp-assignments distance)))

(define-hook cram-language::on-begin-grasp (obj-desig))
(define-hook cram-language::on-finish-grasp (log-id success))
(define-hook cram-language::on-grasp-decisions-complete
    (log-id object-name pregrasp-pose grasp-pose side object-pose))

(defun update-action-designator (action-desig new-properties)
  (make-designator 'action (update-designator-properties
                            new-properties
                            (description action-desig))
                   action-desig))

(defun min-object-grasp-effort (object)
  (let ((efforts (crs:prolog `(cram-language::grasp-effort ,object ?effort))))
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

(defun perform-grasps (action-desig object assignments-list &key log-id)
  (let* ((obj (desig:newest-effective-designator object))
         (obj-pose (reference (desig-prop-value obj 'desig-props:at)))
         (obj-name (desig-prop-value obj 'desig-props:name)))
    (labels ((calculate-grasp-pose (pose grasp-offset gripper-offset)
               (cl-tf2:transform-pose
                *tf2-buffer*
                :pose (relative-pose
                       (relative-pose pose grasp-offset)
                       gripper-offset)
                :target-frame "/torso_lift_link"
                :timeout cram-roslisp-common:*tf-default-timeout*))
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
                               (:left *left-safe-pose*)
                               (:right *right-safe-pose*))
                  :effort (min-object-grasp-effort obj)))))
      (let ((params (mapcar #'grasp-parameters assignments-list)))
        (dolist (param-set params)
          (let ((pub (roslisp:advertise "/dhdhdh" "geometry_msgs/PoseStamped")))
            (roslisp:publish pub (cl-tf2:to-msg (pregrasp-pose param-set))))
          (cram-language::on-grasp-decisions-complete
           log-id obj-name (pregrasp-pose param-set)
           (grasp-pose param-set) (arm param-set) obj-pose))
        (update-action-designator
         action-desig `(,@(mapcar (lambda (param-set)
                                    `(grasp ((arm ,(arm param-set))
                                             (effort ,(effort param-set))
                                             (object-pose ,obj-pose)
                                             (grasp-type ,(grasp-type param-set))
                                             (pregrasp-pose ,(pregrasp-pose param-set))
                                             (grasp-pose ,(grasp-pose param-set)))))
                                  params)))
        (execute-grasps obj-name params)
        (dolist (param-set params)
          (with-vars-strictly-bound (?link-name)
              (lazy-car
               (prolog
                `(cram-manipulation-knowledge:end-effector-link
                  ,(arm param-set) ?link-name)))
            (plan-knowledge:on-event
             (make-instance 'plan-knowledge:object-attached
                            :object obj
                            :link ?link-name
                            :side (arm param-set))))
          (let ((at (desig-prop-value obj 'desig-props:at)))
            (make-designator
             'location
             (append (description at)
                     `((handle `(,(arm param-set)
                                 ,(object-part param-set)))))
             at)))))))

(def-action-handler grasp (action-desig object)
  "Handles the grasping of any given `object'. Calculates proper grasping poses for the object, based on physical gripper characteristics, free grippers, object grasp points (handles), grasp type for this object, and position of the object relative to the robot's grippers. `action-desig' is the action designator instance that triggered this handler's execution, and is later updated with more precise grasping information based on the actual infered action."
  (display-object-handles object)
  (let ((grasp-assignments (crs:prolog `(grasp-assignments ,object ?grasp-assignments)))
        (log-id (first (cram-language::on-begin-grasp object)))
        (success nil))
    (unwind-protect
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
                       (ros-info (pr2 manip-pm) "Performing grasp assignment(s):~%")
                       (dolist (assignment assignments-list)
                         (ros-info (pr2 manip-pm) " - ~a/~a"
                                   (grasp-type assignment)
                                   (side assignment)))
                       (perform-grasps action-desig object assignments-list :log-id log-id)
                       (ros-info (pr2 manip-pm) "Successful grasp")
                       (setf success t)
                       (success))))))
           (cpl:fail 'manipulation-pose-unreachable))
      (cram-language::on-finish-grasp log-id success))))

(defun pose-pointing-away-from-base (object-pose)
  (let ((ref-frame "/base_link")
        (fin-frame "/map"))
    (let* ((base-transform-map
             (cl-tf2:ensure-transform-available
              *tf2-buffer* ref-frame fin-frame))
           (base-pose-map (cl-tf-datatypes:make-pose-stamped
                           (cl-tf-datatypes:frame-id base-transform-map)
                           (cl-tf-datatypes:stamp base-transform-map)
                           (cl-transforms:translation base-transform-map)
                           (cl-transforms:rotation base-transform-map)))
           (object-pose-map (cl-tf2:transform-pose
                             *tf2-buffer*
                             :pose object-pose
                             :target-frame fin-frame
                             :timeout cram-roslisp-common:*tf-default-timeout*))
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
      (cl-tf-datatypes:make-pose-stamped fin-frame 0.0
                                         (cl-transforms:origin object-pose-map)
                                         (cl-transforms:euler->quaternion
                                          :az (+ angle (/ pi 2)))))))

(define-hook cram-language::on-begin-putdown (obj-desig loc-desig))
(define-hook cram-language::on-finish-putdown (log-id success))

(defun make-putdown-pose (putdown-location &key (z-offset 0.0))
  (let* ((putdown-pose (pose-pointing-away-from-base
                        (reference putdown-location)))
         (pose-in-tll
           (cl-tf2:transform-pose
            *tf2-buffer*
            :pose putdown-pose
            :target-frame "/torso_lift_link"
            :timeout cram-roslisp-common:*tf-default-timeout*
            :use-current-ros-time t)))
    (cl-tf-datatypes:copy-pose-stamped
     pose-in-tll :origin (cl-transforms:v+ (cl-transforms:origin pose-in-tll)
                                           (cl-transforms:make-3d-vector 0.0 0.0 z-offset)))))

(define-hook cram-language::on-put-down-reorientation-count (object-designator))

(defun hand-poses-for-putdown (grasp-assignment putdown-pose)
  (let* ((grasp-type (grasp-type grasp-assignment))
         ;; TODO(winkler): Adapt this `pre-putdown-pose' to the
         ;; grasp-type
         (pre-putdown-offset *pre-putdown-offset*)
         (putdown-offset *putdown-offset*)
         (unhand-offset (cond ((eql grasp-type 'desig-props:top-slide-down)
                               *unhand-top-slide-down-offset*)
                              (t *unhand-offset*))))
    (labels ((gripper-putdown-pose (object-in-gripper-pose object-putdown-pose)
               (cl-tf-datatypes:pose->pose-stamped
                (cl-tf-datatypes:frame-id object-putdown-pose) 0.0
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
                           grasp-assignment putdown-pose))
             (link-name
               (cut:var-value
                '?link
                (first
                 (crs:prolog
                  `(manipulator-link ,side ?link)))))
             (planning-group
               (cut:var-value
                '?group
                (first
                 (crs:prolog
                  `(planning-group ,side ?group))))))
        (publish-pose putdown-hand-pose "/putdownhandpose")
        (publish-pose pre-putdown-pose "/preputdownpose")
        (publish-pose unhand-pose "/unhandpose")
        (unless (moveit:plan-link-movements
                 link-name planning-group
                 `(,pre-putdown-pose
                   ,putdown-hand-pose
                   ,unhand-pose)
                 :destination-validity-only t)
          (cpl:fail 'manipulation-failure))
        (make-instance
         'putdown-parameters
         :grasp-type grasp-type
         :arm side
         :pre-putdown-pose pre-putdown-pose
         :putdown-pose putdown-hand-pose
         :unhand-pose unhand-pose)))))

(defun perform-putdowns (object-designator grasp-assignments putdown-pose)
  (let ((putdown-parameter-sets
          (mapcar (lambda (grasp-assignment)
                    (hand-poses-for-putdown
                     grasp-assignment putdown-pose))
                  grasp-assignments)))
    (execute-putdowns (desig-prop-value object-designator 'name)
                      putdown-parameter-sets)))

(def-action-handler put-down (object-designator location grasp-assignments)
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
                                                  'desig-props::plane-distance)
                                             (+ (desig-prop-value
                                                 object-designator
                                                 'desig-props::plane-distance)
                                                0.01)) ;; Add one centimeter for good measure
                                           0.0)))
         (lazy-putdown-poses
           (crs:prolog
            `(putdown-pose
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
                       (publish-pose putdown-pose "/putdownpose")
                       (perform-putdowns object-designator grasp-assignments putdown-pose)
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
                     `(cram-manipulation-knowledge:end-effector-link
                       ,side ?link-name)))
                 (plan-knowledge:on-event
                  (make-instance
                   'plan-knowledge:object-detached
                   :object grasped-object
                   :link ?link-name
                   :side side))))))
      (cram-language::on-finish-putdown log-id success))))

(defmethod display-object-handles ((object object-designator))
  (let* ((relative-handles (desig-prop-values object 'desig-props::handle))
         (reorient-object
           (var-value '?r (first (crs:prolog `(reorient-object-globally ,object ?r)))))
         (absolute-handles
           (mapcar (lambda (handle)
                     (absolute-handle object handle :reorient reorient-object))
                   relative-handles))
         (pose-msgs
           (map 'vector
                (lambda (handle)
                  (let ((pose (reference (desig-prop-value handle 'desig-props::at))))
                    (cl-tf2:to-msg pose)))
                absolute-handles)))
    (let ((publisher (roslisp:advertise "/objecthandleposes" "geometry_msgs/PoseArray")))
      (roslisp:publish publisher
                       (roslisp:make-message
                        "geometry_msgs/PoseArray"
                        (frame_id header) (cl-tf-datatypes:frame-id
                                           (reference (desig-prop-value
                                                       (first absolute-handles) 'desig-props::at)))
                        (poses) pose-msgs)))))
