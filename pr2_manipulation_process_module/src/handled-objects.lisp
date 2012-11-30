;;; Copyright (c) 2012, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
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

(in-package :pr2-manip-pm)

(defparameter *handle-pregrasp-offset-pose*
  (tf:make-pose
   (tf:make-3d-vector 0.20 0.0 0.0)
   (tf:euler->quaternion :az pi :ax (/ pi 2)))
  "Specifies the gripper pose relative to the respective handle
coordinate system (including it's origin and rotation) when going into
pregrasp.")
(defparameter *handle-grasp-offset-pose*
  (tf:make-pose
   (tf:make-3d-vector 0.135 0.0 0.0)
   (tf:euler->quaternion :az pi :ax (/ pi 2)))
  "Specifies the gripper pose relative to the respective handle
coordinate system (including it's origin and rotation) when going into
grasp.")

(defmacro par-loop ((var sequence) &body body)
  "Executes body in parallel for each `var' in `sequence'."
  (alexandria:with-gensyms (loop-body evaluated-sequence)
    `(labels ((,loop-body (,evaluated-sequence)
                (cpl:par
                  (let ((,var (car ,evaluated-sequence)))
                    (cpl:seq ,@body))
                  (when (cdr ,evaluated-sequence)
                    (,loop-body (cdr ,evaluated-sequence))))))
       (,loop-body ,sequence))))

(defun grab-handled-object-constraint-aware (obj arms-handles-pairs obstacles
                                             &key obj-as-obstacle)
  (clear-collision-objects)
  (dolist (obstacle (cut:force-ll obstacles))
    (register-collision-object obstacle))
  (when obj-as-obstacle
    (register-collision-object obj))
  (grab-handled-object obj arms-handles-pairs :constraint-aware t)
  (loop for (arm handle) in arms-handles-pairs
        do (attach-collision-object arm obj)))

(defun grab-handled-object (obj arms-handles-pairs
                            &key constraint-aware)
  "Grasps an object `obj' on its designated handles with the chosen
robot arm sides, both specified in `arms-handles-pairs'. The grippers
will go into a pregrasp pose (relatively defined by
`*handle-pregrasp-offset-pose*' w.r.t. the absolute position of the
handle), will open and move into a grasp pose (relatively defined by
`*handle-grasp-offset-pose*' w.r.t. the absolute position of the
handle). Afterwards, the grippers are closed (until they reach the
opening position defined by the `radius' property of the handle
objects)."
  (let ((pair-count (length arms-handles-pairs)))
    (assert (> pair-count 0) ()
            "No arm/handle pairs specified in `grab-handled-object'.")
    (roslisp:ros-info (pr2-manipulation-process-module)
                      "Going into pregrasp for ~a arm(s)" pair-count)
    (par-loop (arm-handle-pair arms-handles-pairs)
      (pregrasp-handled-object-with-relative-location
       obj (car arm-handle-pair) (cdr arm-handle-pair)
       :constraint-aware constraint-aware))
    (roslisp:ros-info (pr2-manipulation-process-module)
                      "Opening gripper(s) for ~a arm(s)" pair-count)
    (par-loop (arm-handle-pair arms-handles-pairs)
      (let* ((handle (cdr arm-handle-pair))
             (radius (or (desig-prop-value handle 'radius)
                         0.0)))
        (open-gripper (car arm-handle-pair)
                      :position (+ radius 0.02))))
    (roslisp:ros-info (pr2-manipulation-process-module)
                      "Going into grasp for ~a arm(s)" pair-count)
    (par-loop (arm-handle-pair arms-handles-pairs)
      (grasp-handled-object-with-relative-location
       obj (car arm-handle-pair) (cdr arm-handle-pair)))
    (roslisp:ros-info (pr2-manipulation-process-module)
                      "Closing gripper(s) for ~a arm(s)" pair-count)
    (par-loop (arm-handle-pair arms-handles-pairs)
      (let* ((handle (cdr arm-handle-pair))
             (radius (or (desig-prop-value handle 'radius)
                         0.0)))
        (close-gripper (car arm-handle-pair)
                       :position radius)))
    (loop for (arm . handle) in arms-handles-pairs
          for radius = (or (desig-prop-value handle 'radius) 0.0)
          do (check-valid-gripper-state arm
                                        :min-position
                                        (- radius 0.01))
             (with-vars-strictly-bound (?link-name)
                 (lazy-car (prolog `(cram-manipulation-knowledge:end-effector-link ,arm ?link-name)))
               (plan-knowledge:on-event
                (make-instance
                    'plan-knowledge:object-attached
                  :object obj
                  :link ?link-name
                  :side arm))))))

(defun taxi-handled-object (obj side handle
                            &key (relative-gripper-pose
                                  (tf:make-identity-pose))
                              constraint-aware)
  "Commutes the arm to a certain absolute pose. The target pose is
determined through the absolute object pose of object `obj', the
relative object handle location `relative-handle-loc' and the relative
gripper pose `relative-gripper-pose' w.r.t. the handle. The relative
gripper pose defaults to an identity pose."
  (let*
      ;; NOTE(winkler): We're transforming into the tf-frame
      ;; "torso_lift_link" here due to the fact that get-ik
      ;; (service node constraint_awake_ik) does not transform from
      ;; the map-frame. The constructor of the class does not
      ;; create a tf listener. Therefore, the goal has to be
      ;; specified in the ik root.
      ((absolute-pose-map (object-handle-absolute
                           obj handle
                           :handle-offset-pose relative-gripper-pose))
       (absolute-pose (tf:transform-pose *tf*
                                         :pose absolute-pose-map
                                         :target-frame "torso_lift_link"))
       (seed-state (calc-seed-state-elbow-up side))
       (move-ik (cond (constraint-aware
                       (get-constraint-aware-ik side absolute-pose
                                                :seed-state seed-state))
                      (t
                       (get-ik side absolute-pose :seed-state seed-state)))))
    (unless move-ik
      (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable))
    (let ((move-trajectory (ik->trajectory (first move-ik) :duration 5.0)))
      (unless move-trajectory
        (cpl:fail 'cram-plan-failures:manipulation-failed))
      (nth-value 1 (execute-arm-trajectory side move-trajectory)))))

(defun grasp-handled-object-with-relative-location (obj side handle
                                                    &key constraint-aware)
  "Moves the gripper side `side' into the grasp position with respect
to the object's `obj' handle `handle'."
  (taxi-handled-object
   obj side handle :relative-gripper-pose *handle-grasp-offset-pose*
                   :constraint-aware constraint-aware))

(defun pregrasp-handled-object-with-relative-location (obj side handle
                                                       &key constraint-aware)
  "Moves the gripper side `side' into the pregrasp position with
respect to the object's `obj' handle `handle'."
  (taxi-handled-object
   obj side handle :relative-gripper-pose *handle-pregrasp-offset-pose*
                   :constraint-aware constraint-aware))

(defun object-handle-absolute (obj handle
                               &key (handle-offset-pose
                                     (tf:make-identity-pose)))
  "Transforms the relative handle location `handle' of object `obj'
into the object's coordinate system and returns the appropriate
location designator. The optional parameter `handle-offset-pose' is
applied to the handle pose before the absolute object pose is
applied."
  (let* ((absolute-object-loc (desig-prop-value obj 'at))
         (absolute-object-pose-stamped (reference absolute-object-loc))
         (relative-handle-loc (desig-prop-value handle 'at))
         (relative-handle-pose (cl-transforms:transform-pose
                                (tf:pose->transform
                                 (reference relative-handle-loc))
                                handle-offset-pose)))
    (tf:pose->pose-stamped
     (tf:frame-id absolute-object-pose-stamped)
     (tf:stamp absolute-object-pose-stamped)
     (cl-transforms:transform-pose
      (tf:pose->transform
       absolute-object-pose-stamped)
      relative-handle-pose))))

(defun handle-distances (obj arms &key
                                    (pregrasp-offset (tf:make-identity-pose))
                                    (grasp-offset (tf:make-identity-pose))
                                    constraint-aware)
  "Generates a list of handles and their respective arms which can
reach them, as well as the respective distances for each."
  (let ((handles (desig-prop-values obj 'handle))
        (distances nil))
    (loop for handle in handles
          for arm-distances = (arm-handle-distances
                               obj handle arms
                               :pregrasp-offset pregrasp-offset
                               :grasp-offset grasp-offset
                               :constraint-aware constraint-aware)
          when (> (length arm-distances) 0)
            do (push (cons handle arm-distances) distances))
    distances))

(defun arm-handle-distances (obj handle arms
                             &key
                               (pregrasp-offset
                                (tf:make-identity-pose))
                               (grasp-offset
                                (tf:make-identity-pose))
                               constraint-aware)
  "Calculates the distances for each arm given in the `arms' list with
respect to the handle `handle'. Only arms that can actually reach the
handle are included. The resulting list consists of entries of the
form `((\"with-offset\" . distance-with-offset) (\"without-offset\"
. distance-without-offset)' for each arm given in `arms'. If no arm
could reach the handle, `NIL' is returned."
  (let ((distances nil))
    (loop for arm in arms
          for target-link = (ecase arm
                              (:left "l_wrist_roll_link")
                              (:right "r_wrist_roll_link"))
          for dist-with-offset = (reaching-length
                                  (object-handle-absolute
                                   obj handle
                                   :handle-offset-pose
                                   pregrasp-offset)
                                  arm
                                  :constraint-aware constraint-aware
                                  :calc-euclidean-distance t
                                  :euclidean-target-link target-link)
          for dist-without-offset = (reaching-length
                                     (object-handle-absolute
                                      obj handle
                                      :handle-offset-pose
                                      grasp-offset)
                                     arm
                                     :calc-euclidean-distance t
                                     :euclidean-target-link target-link)
          when (and dist-with-offset dist-without-offset)
            do (push (cons
                      arm
                      (list (cons "with-offset" dist-with-offset)
                            (cons "without-offset" dist-without-offset)))
                     distances))
    distances))

(defun nearest-of-handles (handle-distances)
  (let ((nearest-side nil)
        (nearest-handle nil))
    (loop for handle-data in handle-distances
          for nearer-side = (handle-nearer-p handle-data nearest-handle)
          when nearer-side
            do (setf nearest-side nearer-side)
               (setf nearest-handle handle-data))
    (cons nearest-side (car nearest-handle))))

(defun nearest-side-on-handle (handle)
  "Returns a list of the format `(nearest-side lowest-distance)' or
`nil' if no arm side was found from which the handle `handle' was
reachable."
  (let* ((nearest-handle nil)
         (nearest-side nil)
         (lowest-distance nil)
         (sides-distances (cdr handle)))
    (when (> (length sides-distances) 0)
      (loop for side-distances in sides-distances
            for side = (car side-distances)
            for distances = (cdr side-distances)
            for dist-without-offset = (cdr
                                       (assoc
                                        "without-offset"
                                        distances
                                        :test 'equal))
            when (or (not lowest-distance)
                     (< dist-without-offset lowest-distance))
              do (setf nearest-side side)
                 (setf nearest-handle handle)
                 (setf lowest-distance dist-without-offset)))
    (when (and nearest-side lowest-distance)
      (list nearest-side lowest-distance))))

(defun handle-nearer-p (handle-in-question handle-compare)
  "Checks to see if `handle-in-question' is actually nearer to the
robot than `handle-compare'. If that is the case, the nearest arm side
as saved in the respective handle variable is returned. If it is not,
`NIL' is returned."
  (let* ((dist-in-question (nearest-side-on-handle
                            handle-in-question))
         (dist-compare (nearest-side-on-handle
                        handle-compare))
         (side-in-question (first dist-in-question))
         (distance-in-question (second dist-in-question))
         (distance-compare (second dist-compare)))
    (cond ((and dist-in-question dist-compare)
           (when (< distance-in-question distance-compare)
             side-in-question))
          (dist-in-question
           side-in-question))))

(defun fail-on-no-nearest-handle (arm-handle)
  (unless arm-handle
    (cpl:fail 'manipulation-pose-unreachable))
  t)

(defun optimal-handle-assignment (avail-arms avail-handles min-handles
                                  &key max-handles)
  "Finds the assignment solution of arms to handles on a handles
object based in the available arms list `avail-arms', the reachable
handles list `avail-handles' (which is a handle-evaluation including
the `cost' to reach the respective handle with each gripper in reach)
and the minimum amount of handles to be used. The optional parameter
`max-handles' can be set to limit the solutions to those which use at
most that many handles/arms. The function returns a list of cons
cells, containint the identifier of the respective arm as `car' and
the handle to grasp with it as `cdr' element."
  (roslisp:ros-info (pr2-manipulation-process-module)
                    "Calculating optimal arm/handle combination for grasp.")
  (when (and (>= (length avail-arms) min-handles)
             (>= (length avail-handles) min-handles))
    (let* ((cheapest-assignment nil)
           (current-lowest-cost nil)
           (assignment-tree (assign-handles-tree avail-arms avail-handles 0))
           (assignment-lists (assign-handles-tree->list assignment-tree)))
      (loop for assignment-list in assignment-lists
            for assignment = (car assignment-list)
            for assign-count = (length assignment)
            for obj-value = (cdr assignment-list)
            when (and (or (not max-handles)
                          (<= assign-count max-handles))
                      (or (not current-lowest-cost)
                          (< obj-value current-lowest-cost)))
              do (setf cheapest-assignment assignment)
                 (setf current-lowest-cost obj-value))
      cheapest-assignment)))

(defun assign-handles-tree->list (assignment-tree)
  "This function onverts a handle-assignment tree into an appropriate
list which contains all paths of the tree, plus the respective cost
value of the last link (which represents the cosy for the whole
path). A list of lists of these assignments is returned."
  (let ((assignment-lists nil))
    (loop for branch in assignment-tree
          for current-assignment = (cdr (assoc :assignment branch))
          for current-arm-handle = (cons (first current-assignment)
                                         (second current-assignment))
          for current-obj-value = (cdr (assoc :obj-value branch))
          for next-assignments-tree = (cdr (assoc :next-assignments branch))
          for next-assignments-list = (assign-handles-tree->list
                                       next-assignments-tree)
          do (cond (next-assignments-tree
                    (push
                     (first
                      (mapcar (lambda (x)
                                (let ((xcar (car x))
                                      (xcdr (cdr x)))
                                  (cons (push current-arm-handle xcar) xcdr)))
                              next-assignments-list))
                     assignment-lists))
                   (t
                    (push (cons (list current-arm-handle) current-obj-value)
                          assignment-lists))))
    assignment-lists))

(defun assign-handles-tree (avail-arms avail-handles current-objective-value)
  "Recursive function that builds a tree of all possible grasp
solutions using the available arms `avail-arms' on the available
handles `avail-handles'. The `current-objective-value' keeps track of
the objective value calculated for the current path up to the current
parent node. It is initially zero."
  (let ((assignments nil))
    (when (> (length avail-arms) 0)
      (loop for handle in avail-handles
            do (loop for arm in avail-arms
                     for handle-reachable-by-arm = (eq
                                                    (not
                                                     (position
                                                      arm
                                                      (cdr handle)
                                                      :test
                                                      (lambda
                                                          (test-arm test-he)
                                                        (eq test-arm
                                                            (car test-he)))))
                                                    nil)
                     for handle-objective-value = (or
                                                   (cdr
                                                    (assoc
                                                     "without-offset"
                                                     (cdr (assoc arm
                                                                 (cdr handle)))
                                                     :test 'equal))
                                                   0)
                     for combined-objective-value = (+ current-objective-value
                                                       handle-objective-value)
                     when handle-reachable-by-arm
                       do (push
                           (list
                            (cons :assignment (cons arm handle))
                            (cons :obj-value combined-objective-value)
                            (cons :next-assignments
                                  (assign-handles-tree
                                   (remove arm avail-arms)
                                   (remove handle avail-handles)
                                   combined-objective-value)))
                           assignments))))
    assignments))
